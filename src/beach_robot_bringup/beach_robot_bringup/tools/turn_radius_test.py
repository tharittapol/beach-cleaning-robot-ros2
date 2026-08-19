"""Drive a single arc at a commanded (speed, radius) and record how tightly the
robot actually turns.

Companion analyzer: analyze_turn_radius.py (ros2 run beach_robot_bringup
turn_radius_analyze). This node only drives + records a rich per-tick CSV and
prints a live summary; deeper cross-run analysis happens offline.

Headline metrics (these need no positional accuracy, so slip does not corrupt
them):
  * yaw-rate fidelity = w_actual / w_cmd, with w_actual taken straight from the
    IMU gyro (/imu/data angular.z). This is the direct under-rotation number.
  * inside-front encoder speed (/enc_vel): inside = FL for left turns, FR for
    right turns. CLAUDE.md clean-turn criterion is inside front >= 0.12 m/s;
    below that the arc has degenerated toward a pivot/stall.

Radius is *derived* from R = v_mean / w_mean over the steady window (v from odom
twist, w from gyro). A least-squares circle fit on the odom x/y path is reported
only as a cross-check because odom x/y still carries translation slip. To anchor
the odom-radius bias once, run a --sweep-deg 360 arc and tape-measure the circle
diameter (same idea as the sand linear_scale tape calibration).

The arc is commanded through /cmd_vel so it passes through beach_wheel_mixer,
i.e. it includes the real mixer clamps (min_turn_radius 0.65, max_w/max_v 0.35).
Requested radii tighter than the mixer clamp are flagged so a clamp is not
mistaken for drivetrain slip.
"""

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

WHEEL_NAMES = ("fl", "fr", "rl", "rr")

# Mixer clamps (beach_wheel_mixer/config/mixer.yaml) used only to flag when a
# requested arc cannot physically be commanded and the mixer will widen it.
MIXER_MIN_TURN_RADIUS = 0.65
MIXER_MAX_W = 0.35
MIXER_MAX_V = 0.35
CLEAN_INSIDE_FRONT_MPS = 0.12
# A clean in-place pivot needs every wheel turning (rear floor is ~0.06 m/s).
PIVOT_MIN_WHEEL_MPS = 0.05


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(rad):
    return math.atan2(math.sin(rad), math.cos(rad))


def mean(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    return sum(vals) / len(vals)


def circle_fit_radius(points):
    """Kasa algebraic circle fit. Returns (radius, cx, cy) or (None, None, None)."""
    if len(points) < 5:
        return None, None, None
    n = len(points)
    sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
    for x, y in points:
        z = x * x + y * y
        sx += x
        sy += y
        sxx += x * x
        syy += y * y
        sxy += x * y
        sxz += x * z
        syz += y * z
        sz += z
    # Solve [sxx sxy sx; sxy syy sy; sx sy n] [a b c]^T = [sxz syz sz]
    a_mat = [
        [sxx, sxy, sx],
        [sxy, syy, sy],
        [sx, sy, float(n)],
    ]
    rhs = [sxz, syz, sz]
    sol = _solve3(a_mat, rhs)
    if sol is None:
        return None, None, None
    a, b, c = sol
    cx = a / 2.0
    cy = b / 2.0
    r2 = c + cx * cx + cy * cy
    if r2 <= 0.0:
        return None, None, None
    return math.sqrt(r2), cx, cy


def _solve3(m, rhs):
    a = [row[:] + [rhs[i]] for i, row in enumerate(m)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-12:
            return None
        a[col], a[piv] = a[piv], a[col]
        pivval = a[col][col]
        for j in range(col, 4):
            a[col][j] /= pivval
        for r in range(3):
            if r == col:
                continue
            factor = a[r][col]
            for j in range(col, 4):
                a[r][j] -= factor * a[col][j]
    return [a[0][3], a[1][3], a[2][3]]


class TurnRadiusTest(Node):
    def __init__(self, args):
        super().__init__("turn_radius_test")
        self.args = args
        self.started_mono = time.monotonic()
        self.pivot = False

        self.odom = None
        self.gyro_w = None
        self.wheel_cmd = None
        self.enc_vel = None

        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                        reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 20)
        self.create_subscription(Imu, args.imu_topic, self._imu_cb, be)
        self.create_subscription(Float32MultiArray, args.wheel_cmd_topic, self._wheel_cmd_cb, 20)
        self.create_subscription(Float32MultiArray, args.enc_vel_topic, self._enc_cb, 20)

        self.out_dir = Path(args.out_dir).expanduser()
        self.out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in args.label)
        self.traj_path = self.out_dir / f"turn_{stamp}_{safe}.csv"
        self.summary_path = self.out_dir / "turn_summary.csv"

        self.rows = []  # in-memory samples for end-of-run analysis

    # --- callbacks ---
    def _odom_cb(self, msg):
        self.odom = msg

    def _imu_cb(self, msg):
        self.gyro_w = float(msg.angular_velocity.z)

    def _wheel_cmd_cb(self, msg):
        self.wheel_cmd = [float(x) for x in msg.data[:4]]

    def _enc_cb(self, msg):
        self.enc_vel = [float(x) for x in msg.data[:4]]

    # --- helpers ---
    def _odom_pose(self):
        p = self.odom.pose.pose
        return float(p.position.x), float(p.position.y), yaw_from_quaternion(p.orientation)

    def stop_robot(self, count=12):
        stop = Twist()
        for _ in range(count):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.04)

    def wait_for_inputs(self):
        deadline = time.monotonic() + self.args.wait_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None and self.gyro_w is not None:
                return True
        return False

    def run(self):
        if not self.wait_for_inputs():
            self.get_logger().error(
                f"Missing inputs: odom={self.odom is not None} imu={self.gyro_w is not None}. "
                f"Check {self.args.odom_topic} and {self.args.imu_topic}.")
            self.stop_robot()
            return False

        self.pivot = bool(self.args.pivot) or self.args.radius <= 0.0
        if self.pivot:
            if self.args.omega <= 0.0:
                self.get_logger().error("Pivot mode (--pivot or --radius 0) needs --omega > 0 rad/s.")
                self.stop_robot()
                return False
            v = 0.0
            w_mag = abs(self.args.omega)
        else:
            v = abs(self.args.speed)
            w_mag = v / max(self.args.radius, 1e-6)
        sign = 1.0 if self.args.direction == "left" else -1.0
        w_cmd = sign * w_mag
        target_rad = math.radians(self.args.sweep_deg)

        clamp_notes = []
        if not self.pivot and self.args.radius < self.args.mixer_min_radius:
            clamp_notes.append(
                f"requested R={self.args.radius:.2f} < mixer min_turn_radius {self.args.mixer_min_radius}")
        if w_mag > self.args.mixer_max_w:
            clamp_notes.append(f"requested |w|={w_mag:.3f} > mixer max_w {self.args.mixer_max_w}")
        if v > MIXER_MAX_V:
            clamp_notes.append(f"requested v={v:.3f} > mixer max_v {MIXER_MAX_V}")
        widen = "mixer will pivot-clamp" if self.pivot else "mixer will widen the arc"
        for note in clamp_notes:
            self.get_logger().warn(f"CLAMP: {note} ({widen})")

        spec = "v=0 (in-place)" if self.pivot else f"R_cmd={self.args.radius:.2f}m v={v:.3f}"
        self.get_logger().info(
            f"{'PIVOT' if self.pivot else 'ARC'}: dir={self.args.direction} {spec} "
            f"w_cmd={w_cmd:+.3f}rad/s sweep={self.args.sweep_deg:.0f}deg")

        x0, y0, yaw0 = self._odom_pose()
        time.sleep(max(0.0, self.args.settle_sec))
        x0, y0, yaw0 = self._odom_pose()

        period = 1.0 / max(1.0, self.args.rate_hz)
        start = time.monotonic()
        next_tick = start
        prev_yaw = yaw0
        cum_yaw = 0.0
        last_report = 0
        result = "timeout"

        while rclpy.ok():
            now = time.monotonic()
            if now < next_tick:
                rclpy.spin_once(self, timeout_sec=min(0.02, next_tick - now))
                continue
            next_tick += period
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.odom is None:
                continue

            elapsed = now - start
            ramp = min(1.0, elapsed / self.args.ramp_sec) if self.args.ramp_sec > 0 else 1.0
            x, y, yaw = self._odom_pose()
            cum_yaw += normalize_angle(yaw - prev_yaw)
            prev_yaw = yaw

            ov = float(self.odom.twist.twist.linear.x)
            ow = float(self.odom.twist.twist.angular.z)
            self.rows.append({
                "elapsed_sec": elapsed,
                "cmd_v": v * ramp,
                "cmd_w": w_cmd * ramp,
                "ramp": ramp,
                "odom_x": x,
                "odom_y": y,
                "odom_yaw": yaw,
                "cum_yaw": cum_yaw,
                "odom_v": ov,
                "odom_w": ow,
                "gyro_w": self.gyro_w,
                "wheel_cmd": list(self.wheel_cmd) if self.wheel_cmd else None,
                "enc_vel": list(self.enc_vel) if self.enc_vel else None,
            })

            deg = abs(math.degrees(cum_yaw))
            if int(deg) // 30 > last_report:
                last_report = int(deg) // 30
                self.get_logger().info(
                    f"swept={deg:.0f}/{self.args.sweep_deg:.0f}deg gyro_w={self.gyro_w:+.3f} "
                    f"odom_v={ov:.3f}")

            if abs(cum_yaw) >= target_rad:
                result = "target_reached"
                break
            if elapsed > self.args.max_sec:
                result = "timeout"
                self.get_logger().error(f"Timeout: swept only {deg:.0f}deg")
                break

            cmd = Twist()
            cmd.linear.x = v * ramp
            cmd.angular.z = w_cmd * ramp
            self.cmd_pub.publish(cmd)

        self.stop_robot()
        xf, yf, yawf = self._odom_pose()
        self._finish(result, w_cmd, v, x0, y0, xf, yf, cum_yaw, clamp_notes)
        return result == "target_reached"

    def _finish(self, result, w_cmd, v_cmd, x0, y0, xf, yf, cum_yaw, clamp_notes):
        self._write_traj()

        # Steady window: skip the first settle_deg of yaw (ramp/transient).
        settle = math.radians(self.args.settle_deg)
        steady = [r for r in self.rows if abs(r["cum_yaw"]) >= settle]
        if len(steady) < 5:
            steady = self.rows

        gyro_w = mean(abs(r["gyro_w"]) for r in steady if r["gyro_w"] is not None)
        odom_v = mean(r["odom_v"] for r in steady)
        odom_w = mean(abs(r["odom_w"]) for r in steady)
        w_cmd_mag = abs(w_cmd)

        yaw_fidelity = (gyro_w / w_cmd_mag) if gyro_w and w_cmd_mag else None
        r_derived = (odom_v / gyro_w) if gyro_w and odom_v is not None and gyro_w > 1e-6 else None
        pts = [(r["odom_x"], r["odom_y"]) for r in steady]
        r_fit, _, _ = circle_fit_radius(pts)

        inside_idx = 0 if self.args.direction == "left" else 1  # FL or FR
        inside_label = "fl" if inside_idx == 0 else "fr"
        inside_vals = [r["enc_vel"][inside_idx] for r in steady
                       if r["enc_vel"] and len(r["enc_vel"]) > inside_idx]
        inside_min = min((abs(x) for x in inside_vals), default=None)
        inside_mean = mean(abs(x) for x in inside_vals) if inside_vals else None

        # Minimum |enc| across all 4 wheels (a true pivot needs every wheel turning).
        wheel_mins = []
        for i in range(4):
            vals = [abs(r["enc_vel"][i]) for r in steady
                    if r["enc_vel"] and len(r["enc_vel"]) > i]
            if vals:
                wheel_mins.append(min(vals))
        min_wheel_enc = min(wheel_mins) if wheel_mins else None

        closure = math.hypot(xf - x0, yf - y0)
        swept_deg = math.degrees(abs(cum_yaw))

        if self.pivot:
            # inside front reverses in a pivot, so it is not the gate; instead
            # require every wheel turning, and report center drift (lower=tighter).
            clean = min_wheel_enc is not None and min_wheel_enc >= PIVOT_MIN_WHEEL_MPS
            verdict = "CLEAN-PIVOT" if clean else "DEGENERATE(a wheel stalls)"
        else:
            clean = inside_min is not None and inside_min >= CLEAN_INSIDE_FRONT_MPS
            verdict = "CLEAN" if clean else "DEGENERATE(inside front stalls)"
        if clamp_notes:
            verdict += " [MIXER-CLAMPED]"

        def f(x, d=3):
            return f"{x:.{d}f}" if isinstance(x, (int, float)) else "-"

        mode = "PIVOT" if self.pivot else "ARC"
        lines = [
            "",
            f"=============== TURN SUMMARY ({mode}) ===============",
            f"result          : {result}   swept {swept_deg:.0f} deg",
            f"direction       : {self.args.direction}   inside front wheel = {inside_label.upper()}",
        ]
        if self.pivot:
            lines.append(f"cmd             : v=0  w_cmd={w_cmd_mag:.3f} rad/s (in-place pivot)")
        else:
            lines.append(f"R_cmd           : {self.args.radius:.3f} m   v_cmd={v_cmd:.3f}  "
                         f"w_cmd={w_cmd_mag:.3f} rad/s")
        lines += [
            "--- HEADLINE (slip-immune) ---",
            f"w_gyro (actual) : {f(gyro_w)} rad/s",
            f"yaw fidelity    : {f(yaw_fidelity)}   (w_gyro/w_cmd; DIAGNOSTIC that "
            f"shifts with turn_gain, not a 1.0 target)",
        ]
        if self.pivot:
            lines += [
                f"min wheel |enc| : {f(min_wheel_enc)} m/s  (clean >= {PIVOT_MIN_WHEEL_MPS}; "
                "every wheel must turn)",
                f"inside front {inside_label.upper():3}: min={f(inside_min)} m/s "
                "(reverses in a pivot — not the gate)",
                f"center drift    : {f(closure)} m over {swept_deg:.0f}deg "
                "(lower = tighter pivot; ideal ~0)",
                f"VERDICT         : {verdict}",
            ]
        else:
            lines += [
                f"inside front {inside_label.upper():3}: min={f(inside_min)}  mean={f(inside_mean)} m/s "
                f"(clean >= {CLEAN_INSIDE_FRONT_MPS})",
                "--- RADIUS (odom-based, cross-check) ---",
                f"R_derived v/w   : {f(r_derived)} m   (gyro w, odom v)",
                f"R_circle_fit    : {f(r_fit)} m",
                f"closure drift   : {f(closure)} m  (start->end; ~0 for full 360)",
                f"VERDICT         : {verdict}",
            ]
        if clamp_notes:
            lines.append("clamp           : " + "; ".join(clamp_notes))
        if not self.pivot and abs(self.args.sweep_deg - 360.0) < 1.0:
            lines.append("TAPE NOTE       : measure circle diameter with tape -> "
                         "R_tape; compare to R_derived to get odom bias.")
        lines.append("==============================================")
        lines.append(f"trajectory CSV  : {self.traj_path}")
        for ln in lines:
            self.get_logger().info(ln)

        self._append_summary({
            "wall_time": datetime.now().isoformat(timespec="seconds"),
            "label": self.args.label,
            "mode": "pivot" if self.pivot else "arc",
            "direction": self.args.direction,
            "R_cmd": 0.0 if self.pivot else self.args.radius,
            "v_cmd": v_cmd,
            "w_cmd": w_cmd_mag,
            "sweep_deg": self.args.sweep_deg,
            "result": result,
            "swept_deg": round(swept_deg, 1),
            "w_gyro": gyro_w,
            "yaw_fidelity": yaw_fidelity,
            "R_derived": r_derived,
            "R_circle_fit": r_fit,
            "inside_front": inside_label,
            "inside_front_min": inside_min,
            "inside_front_mean": inside_mean,
            "min_wheel_enc": min_wheel_enc,
            "clean": int(bool(clean)),
            "clamped": int(bool(clamp_notes)),
            "closure_m": closure,
            "traj_csv": self.traj_path.name,
        })

    def _write_traj(self):
        mode = "pivot" if self.pivot else "arc"
        fields = ["mode", "elapsed_sec", "cmd_v", "cmd_w", "ramp", "odom_x", "odom_y",
                  "odom_yaw", "cum_yaw", "odom_v", "odom_w", "gyro_w"]
        for p in ("wheel_cmd", "enc_vel"):
            fields += [f"{p}_{w}" for w in WHEEL_NAMES]
        with self.traj_path.open("w", newline="") as fh:
            wr = csv.DictWriter(fh, fieldnames=fields)
            wr.writeheader()
            for r in self.rows:
                row = {k: r[k] for k in (
                    "elapsed_sec", "cmd_v", "cmd_w", "ramp", "odom_x", "odom_y",
                    "odom_yaw", "cum_yaw", "odom_v", "odom_w", "gyro_w")}
                row["mode"] = mode
                for p in ("wheel_cmd", "enc_vel"):
                    arr = r[p]
                    for i, w in enumerate(WHEEL_NAMES):
                        row[f"{p}_{w}"] = arr[i] if arr and i < len(arr) else ""
                wr.writerow(row)

    def _append_summary(self, row):
        exists = self.summary_path.exists()
        with self.summary_path.open("a", newline="") as fh:
            wr = csv.DictWriter(fh, fieldnames=list(row.keys()))
            if not exists:
                wr.writeheader()
            wr.writerow(row)
        self.get_logger().info(f"summary appended -> {self.summary_path}")


def main():
    p = argparse.ArgumentParser(
        description="Drive one arc (or in-place pivot) and measure achieved turn radius / "
                    "yaw-rate fidelity / wheel behaviour.")
    p.add_argument("--radius", type=float, default=0.0,
                   help="commanded arc radius (m); <=0 means in-place pivot (needs --omega)")
    p.add_argument("--speed", type=float, default=0.25, help="commanded linear speed (m/s), arc mode")
    p.add_argument("--pivot", action="store_true", help="in-place pivot mode: v=0, command --omega")
    p.add_argument("--omega", type=float, default=0.0, help="pivot yaw rate (rad/s), pivot mode")
    p.add_argument("--direction", choices=("left", "right"), default="left")
    p.add_argument("--sweep-deg", type=float, default=180.0,
                   help="stop after this much yaw change; use 360 for arc tape/closure")
    p.add_argument("--mixer-min-radius", type=float, default=MIXER_MIN_TURN_RADIUS,
                   help="active mixer min_turn_radius, for clamp flagging (match the config you launched)")
    p.add_argument("--mixer-max-w", type=float, default=MIXER_MAX_W,
                   help="active mixer max_w, for clamp flagging")
    p.add_argument("--label", default="turn")
    p.add_argument("--odom-topic", default="/odometry/fusion_bno")
    p.add_argument("--imu-topic", default="/imu/data")
    p.add_argument("--cmd-topic", default="/cmd_vel")
    p.add_argument("--wheel-cmd-topic", default="/wheel_cmd")
    p.add_argument("--enc-vel-topic", default="/enc_vel")
    p.add_argument("--out-dir", default="~/beach_robot_logs/turn_tune")
    p.add_argument("--rate-hz", type=float, default=20.0)
    p.add_argument("--ramp-sec", type=float, default=0.6)
    p.add_argument("--settle-sec", type=float, default=0.3)
    p.add_argument("--settle-deg", type=float, default=20.0,
                   help="exclude first N deg of yaw from steady stats (transient)")
    p.add_argument("--max-sec", type=float, default=90.0)
    p.add_argument("--wait-sec", type=float, default=5.0)
    args = p.parse_args()

    rclpy.init()
    node = TurnRadiusTest(args)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
