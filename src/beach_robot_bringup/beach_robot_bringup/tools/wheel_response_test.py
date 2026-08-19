import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String


WHEEL_NAMES = ("fl", "fr", "rl", "rr")


class WheelResponseTest(Node):
    def __init__(self, args):
        super().__init__("wheel_response_test")
        self.args = args
        self.started_wall = time.time()
        self.started_mono = time.monotonic()

        self.latest_cmd_vel = None
        self.latest_cmd_vel_stamp = None
        self.latest_wheel_cmd = None
        self.latest_wheel_cmd_stamp = None
        self.latest_enc_vel = None
        self.latest_enc_vel_stamp = None
        self.latest_debug = {}
        self.latest_debug_stamp = None

        self.pub_cmd = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.pub_wheel_cmd = self.create_publisher(Float32MultiArray, args.wheel_cmd_topic, 10)
        self.pub_json = self.create_publisher(String, args.esp32_json_cmd_topic, 10)
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Twist,
            args.cmd_vel_topic,
            self._cmd_vel_cb,
            50,
        )
        self.create_subscription(
            Float32MultiArray,
            args.wheel_cmd_topic,
            self._wheel_cmd_cb,
            50,
        )
        self.create_subscription(
            Float32MultiArray,
            args.enc_vel_topic,
            self._enc_vel_cb,
            50,
        )
        self.create_subscription(
            String,
            args.esp32_debug_topic,
            self._debug_cb,
            debug_qos,
        )

        self.out_dir = Path(args.out_dir).expanduser()
        self.out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in args.label)
        self.csv_path = self.out_dir / f"wheel_response_{stamp}_{safe_label}.csv"
        self.meta_path = self.out_dir / f"wheel_response_{stamp}_{safe_label}.json"

        self.csv_file = self.csv_path.open("w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self._csv_fields())
        self.writer.writeheader()

    def _cmd_vel_cb(self, msg):
        self.latest_cmd_vel = (float(msg.linear.x), float(msg.angular.z))
        self.latest_cmd_vel_stamp = self.get_clock().now()

    def _wheel_cmd_cb(self, msg):
        self.latest_wheel_cmd = [float(x) for x in msg.data[:4]]
        self.latest_wheel_cmd_stamp = self.get_clock().now()

    def _enc_vel_cb(self, msg):
        self.latest_enc_vel = [float(x) for x in msg.data[:4]]
        self.latest_enc_vel_stamp = self.get_clock().now()

    def _debug_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON on ESP32 debug topic")
            return
        if not isinstance(data, dict):
            return
        self.latest_debug = data
        self.latest_debug_stamp = self.get_clock().now()

    def _csv_fields(self):
        fields = [
            "wall_time",
            "ros_time_sec",
            "elapsed_sec",
            "test",
            "phase",
            "cmd_v",
            "cmd_w",
            "cmd_vel_age_sec",
            "wheel_cmd_age_sec",
            "enc_vel_age_sec",
            "debug_age_sec",
        ]
        for prefix in (
            "ros_wheel_cmd",
            "ros_enc_vel",
            "dbg_wheel_cmd",
            "dbg_enc_vel_raw",
            "dbg_enc_vel_corr",
            "dbg_wheel_cnt",
            "dbg_u_ff",
            "dbg_u_pid",
            "dbg_motor_u",
            "dbg_pid_kp",
            "dbg_pid_ki",
            "dbg_vmax_mps",
            "dbg_use_closed_loop",
            "dbg_low_speed_target_max_mps",
        ):
            fields.extend(f"{prefix}_{wheel}" for wheel in WHEEL_NAMES)
        fields.extend((
            "dbg_cmd_age_ms",
            "dbg_enc_age_ms",
            "dbg_cmd_rx_count",
            "dbg_cmd_rx_interval_ms",
            "dbg_cmd_seq",
            "dbg_cmd_seq_gap",
            "dbg_usb_parse_errors",
            "dbg_usb_line_overflows",
            "dbg_in_place_turn",
            "dbg_moving_turn",
            "dbg_spin_start_boost",
            "dbg_wheel_test_active",
            "dbg_wheel_test_wheel",
            "dbg_wheel_test_direct_u",
            "dbg_wheel_test_closed_loop",
            "dbg_wheel_test_target",
            "dbg_wheel_test_remaining_ms",
        ))
        return fields

    def _age_sec(self, stamp, now):
        if stamp is None:
            return ""
        return f"{(now - stamp).nanoseconds / 1e9:.3f}"

    def _array_value(self, arr, idx):
        if not isinstance(arr, list) or idx >= len(arr):
            return ""
        try:
            return f"{float(arr[idx]):.6f}"
        except (TypeError, ValueError):
            return ""

    def _add_array(self, row, prefix, arr):
        for idx, wheel in enumerate(WHEEL_NAMES):
            row[f"{prefix}_{wheel}"] = self._array_value(arr, idx)

    def _sample(self, test_name, phase, cmd_v, cmd_w):
        now = self.get_clock().now()
        debug = self.latest_debug or {}

        row = {
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "ros_time_sec": f"{now.nanoseconds / 1e9:.6f}",
            "elapsed_sec": f"{time.monotonic() - self.started_mono:.3f}",
            "test": test_name,
            "phase": phase,
            "cmd_v": f"{cmd_v:.6f}",
            "cmd_w": f"{cmd_w:.6f}",
            "cmd_vel_age_sec": self._age_sec(self.latest_cmd_vel_stamp, now),
            "wheel_cmd_age_sec": self._age_sec(self.latest_wheel_cmd_stamp, now),
            "enc_vel_age_sec": self._age_sec(self.latest_enc_vel_stamp, now),
            "debug_age_sec": self._age_sec(self.latest_debug_stamp, now),
            "dbg_cmd_age_ms": debug.get("cmd_age_ms", ""),
            "dbg_enc_age_ms": debug.get("enc_age_ms", ""),
            "dbg_cmd_rx_count": debug.get("cmd_rx_count", ""),
            "dbg_cmd_rx_interval_ms": debug.get("cmd_rx_interval_ms", ""),
            "dbg_cmd_seq": debug.get("cmd_seq", ""),
            "dbg_cmd_seq_gap": debug.get("cmd_seq_gap", ""),
            "dbg_usb_parse_errors": debug.get("usb_parse_errors", ""),
            "dbg_usb_line_overflows": debug.get("usb_line_overflows", ""),
            "dbg_in_place_turn": debug.get("in_place_turn", ""),
            "dbg_moving_turn": debug.get("moving_turn", ""),
            "dbg_spin_start_boost": debug.get("spin_start_boost", ""),
        }

        wheel_test = debug.get("wheel_test", {})
        if isinstance(wheel_test, dict):
            row.update({
                "dbg_wheel_test_active": wheel_test.get("active", ""),
                "dbg_wheel_test_wheel": wheel_test.get("wheel", ""),
                "dbg_wheel_test_direct_u": wheel_test.get("direct_u", ""),
                "dbg_wheel_test_closed_loop": wheel_test.get("closed_loop", ""),
                "dbg_wheel_test_target": wheel_test.get("target", ""),
                "dbg_wheel_test_remaining_ms": wheel_test.get("remaining_ms", ""),
            })

        self._add_array(row, "ros_wheel_cmd", self.latest_wheel_cmd)
        self._add_array(row, "ros_enc_vel", self.latest_enc_vel)
        self._add_array(row, "dbg_wheel_cmd", debug.get("wheel_cmd"))
        self._add_array(row, "dbg_enc_vel_raw", debug.get("enc_vel_raw"))
        self._add_array(row, "dbg_enc_vel_corr", debug.get("enc_vel_corr"))
        self._add_array(row, "dbg_wheel_cnt", debug.get("wheel_cnt"))
        self._add_array(row, "dbg_u_ff", debug.get("u_ff"))
        self._add_array(row, "dbg_u_pid", debug.get("u_pid"))
        self._add_array(row, "dbg_motor_u", debug.get("motor_u"))
        self._add_array(row, "dbg_pid_kp", debug.get("pid_kp"))
        self._add_array(row, "dbg_pid_ki", debug.get("pid_ki"))
        self._add_array(row, "dbg_vmax_mps", debug.get("vmax_mps"))
        self._add_array(row, "dbg_use_closed_loop", debug.get("use_closed_loop"))
        self._add_array(row, "dbg_low_speed_target_max_mps", debug.get("low_speed_target_max_mps"))

        self.writer.writerow(row)
        self.csv_file.flush()

    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def _publish_wheel_cmd(self, wheel_cmd):
        msg = Float32MultiArray()
        msg.data = [float(x) for x in wheel_cmd]
        self.pub_wheel_cmd.publish(msg)

    def _direct_wheel_cmd(self, v, w):
        if abs(v) > 1e-6:
            raise ValueError("--direct-wheel-cmd currently supports spin-in-place tests only.")

        if w > 0.0:
            return (
                -self.args.direct_spin_front_mps,
                self.args.direct_spin_front_mps,
                -self.args.direct_spin_rear_mps,
                self.args.direct_spin_rear_mps,
            )
        if w < 0.0:
            return (
                self.args.direct_spin_front_mps,
                -self.args.direct_spin_front_mps,
                self.args.direct_spin_rear_mps,
                -self.args.direct_spin_rear_mps,
            )
        return (0.0, 0.0, 0.0, 0.0)

    def _send_esp32_json(self, data, repeat=3):
        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"))
        for _ in range(max(1, repeat)):
            self.pub_json.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for_debug(self, timeout_sec):
        if timeout_sec <= 0.0:
            return False

        start = time.monotonic()
        self.latest_debug_stamp = None
        while rclpy.ok() and (time.monotonic() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_debug_stamp is not None:
                return True
        return False

    def _latest_cmd_vel_or_zero(self):
        if self.latest_cmd_vel is None or self.latest_cmd_vel_stamp is None:
            return 0.0, 0.0

        now = self.get_clock().now()
        age_sec = (now - self.latest_cmd_vel_stamp).nanoseconds / 1e9
        if age_sec > self.args.manual_cmd_timeout_sec:
            return 0.0, 0.0
        return self.latest_cmd_vel

    def _classify_manual_cmd(self, v, w):
        active_v = abs(v) >= self.args.manual_active_v
        active_w = abs(w) >= self.args.manual_active_w
        if not active_v and not active_w:
            return "idle", "zero"
        if not active_v:
            return ("spin_left" if w > 0.0 else "spin_right"), "command"
        if not active_w:
            return ("forward" if v > 0.0 else "backward"), "command"
        if v < 0.0:
            return ("backward_curve_left" if w > 0.0 else "backward_curve_right"), "command"
        return ("curve_left" if w > 0.0 else "curve_right"), "command"

    def _run_manual_record(self, duration_sec):
        end_time = time.monotonic() + max(0.0, duration_sec)
        next_log = 0.0
        log_period = 1.0 / max(1.0, self.args.log_rate_hz)

        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.002)
            now_mono = time.monotonic()
            if now_mono >= next_log:
                v, w = self._latest_cmd_vel_or_zero()
                test_name, phase = self._classify_manual_cmd(v, w)
                self._sample(test_name, phase, v, w)
                next_log = now_mono + log_period

    def _run_phase(self, test_name, phase, v, w, duration_sec):
        end_time = time.monotonic() + max(0.0, duration_sec)
        next_pub = 0.0
        next_log = 0.0
        pub_period = 1.0 / max(1.0, self.args.publish_rate_hz)
        log_period = 1.0 / max(1.0, self.args.log_rate_hz)

        while rclpy.ok() and time.monotonic() < end_time:
            now_mono = time.monotonic()
            if now_mono >= next_pub:
                if self.args.direct_wheel_cmd:
                    self._publish_wheel_cmd(self._direct_wheel_cmd(v, w))
                else:
                    self._publish_cmd(v, w)
                next_pub = now_mono + pub_period
            if now_mono >= next_log:
                self._sample(test_name, phase, v, w)
                next_log = now_mono + log_period
            rclpy.spin_once(self, timeout_sec=0.002)

    def _publish_zero_burst(self, duration_sec=0.5):
        end_time = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end_time:
            if self.args.direct_wheel_cmd:
                self._publish_wheel_cmd((0.0, 0.0, 0.0, 0.0))
            else:
                self._publish_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def run(self, tests):
        metadata = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "csv_path": str(self.csv_path),
            "topics": {
                "cmd_vel": self.args.cmd_vel_topic,
                "wheel_cmd": self.args.wheel_cmd_topic,
                "enc_vel": self.args.enc_vel_topic,
                "esp32_debug": self.args.esp32_debug_topic,
                "esp32_json_cmd": self.args.esp32_json_cmd_topic,
            },
            "publish_rate_hz": self.args.publish_rate_hz,
            "log_rate_hz": self.args.log_rate_hz,
            "settle_sec": self.args.settle_sec,
            "step_sec": self.args.step_sec,
            "stop_sec": self.args.stop_sec,
            "manual_record": self.args.manual_record,
            "record_sec": self.args.record_sec,
            "manual_active_v": self.args.manual_active_v,
            "manual_active_w": self.args.manual_active_w,
            "tests": tests,
        }
        self.meta_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

        if self.args.no_esp32_debug:
            self._send_esp32_json({"dbg_enable": False}, repeat=5)
        else:
            self._send_esp32_json({"dbg_enable": True, "dbg_rate_ms": self.args.debug_rate_ms}, repeat=5)
            if self.args.wait_for_debug_sec > 0.0:
                if self._wait_for_debug(self.args.wait_for_debug_sec):
                    self.get_logger().info("ESP32 debug stream is active")
                else:
                    self.get_logger().warn(
                        "No ESP32 debug sample received before recording; CSV debug columns may be empty"
                    )

        try:
            self.get_logger().info(f"Writing wheel response CSV: {self.csv_path}")
            if self.args.manual_record:
                self.get_logger().info(
                    f"Manual record mode: drive with joy for {self.args.record_sec:.1f} seconds"
                )
                self._run_manual_record(self.args.record_sec)
            else:
                self._run_phase("startup", "zero", 0.0, 0.0, self.args.settle_sec)
                for test_name, v, w in tests:
                    if self.args.direct_wheel_cmd and abs(v) > 1e-6:
                        raise ValueError("--direct-wheel-cmd only supports spin_left/spin_right tests.")
                    self.get_logger().info(f"Running {test_name}: v={v:.3f} m/s w={w:.3f} rad/s")
                    self._run_phase(test_name, "settle_zero", 0.0, 0.0, self.args.settle_sec)
                    self._run_phase(test_name, "command", v, w, self.args.step_sec)
                    self._run_phase(test_name, "stop_zero", 0.0, 0.0, self.args.stop_sec)
        finally:
            if not self.args.manual_record:
                self._publish_zero_burst()
            self._send_esp32_json({"dbg_enable": False}, repeat=3)
            self.csv_file.close()
            self.get_logger().info(f"Saved CSV: {self.csv_path}")
            self.get_logger().info(f"Saved metadata: {self.meta_path}")


def build_tests(args):
    spin_linear = args.spin_linear
    all_tests = {
        "forward": (args.forward_speed, 0.0),
        "backward": (-args.forward_speed, 0.0),
        "spin_left": (spin_linear, args.spin_rate),
        "spin_right": (spin_linear, -args.spin_rate),
        "curve_left": (args.curve_speed, args.curve_rate),
        "curve_right": (args.curve_speed, -args.curve_rate),
    }

    selected = []
    requested = [name.strip() for name in args.tests.split(",") if name.strip()]
    for name in requested:
        if name not in all_tests:
            raise ValueError(f"Unknown test '{name}'. Available: {', '.join(all_tests)}")
        v, w = all_tests[name]
        selected.append((name, v, w))
    return selected


def parse_args():
    parser = argparse.ArgumentParser(description="Run wheel response tests and log ROS/ESP32 signals to CSV.")
    parser.add_argument("--label", default="lifted", help="Label appended to output filenames.")
    parser.add_argument("--out-dir", default="~/beach_robot_logs/wheel_response", help="Directory for CSV logs.")
    parser.add_argument("--tests", default="forward,backward,spin_left,spin_right,curve_left,curve_right")
    parser.add_argument("--forward-speed", type=float, default=0.17, help="m/s for forward/backward tests.")
    parser.add_argument("--spin-rate", type=float, default=0.20, help="rad/s for spin-in-place tests.")
    parser.add_argument(
        "--spin-linear",
        type=float,
        default=0.0,
        help="Small linear.x bias for spin tests. Keep <= mixer in_place_linear_threshold for creep-pivot tests.",
    )
    parser.add_argument("--curve-speed", type=float, default=0.12, help="m/s for curve tests.")
    parser.add_argument("--curve-rate", type=float, default=0.08, help="rad/s for curve tests.")
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--step-sec", type=float, default=4.0)
    parser.add_argument("--stop-sec", type=float, default=1.5)
    parser.add_argument("--publish-rate-hz", type=float, default=20.0)
    parser.add_argument("--log-rate-hz", type=float, default=50.0)
    parser.add_argument(
        "--manual-record",
        action="store_true",
        help="Only record live joy/manual driving; do not publish /cmd_vel or /wheel_cmd.",
    )
    parser.add_argument(
        "--record-sec",
        type=float,
        default=60.0,
        help="Duration for --manual-record.",
    )
    parser.add_argument(
        "--manual-active-v",
        type=float,
        default=0.02,
        help="Minimum |cmd_vel.linear.x| treated as active manual motion.",
    )
    parser.add_argument(
        "--manual-active-w",
        type=float,
        default=0.03,
        help="Minimum |cmd_vel.angular.z| treated as active manual motion.",
    )
    parser.add_argument(
        "--manual-cmd-timeout-sec",
        type=float,
        default=1.0,
        help="Treat /cmd_vel as zero if no command arrives within this time during --manual-record.",
    )
    parser.add_argument(
        "--direct-wheel-cmd",
        action="store_true",
        help="Publish /wheel_cmd directly for spin tests, bypassing /cmd_vel and wheel_mps_mixer.",
    )
    parser.add_argument(
        "--direct-spin-front-mps",
        type=float,
        default=0.0734,
        help="Front wheel m/s magnitude used with --direct-wheel-cmd spin tests.",
    )
    parser.add_argument(
        "--direct-spin-rear-mps",
        type=float,
        default=0.1179,
        help="Rear wheel m/s magnitude used with --direct-wheel-cmd spin tests.",
    )
    parser.add_argument(
        "--debug-rate-ms",
        type=int,
        default=1000,
        help="ESP32 debug interval in ms. Keep modest to avoid serial congestion.",
    )
    parser.add_argument(
        "--wait-for-debug-sec",
        type=float,
        default=2.0,
        help="After enabling ESP32 debug, wait up to this long for the first sample before recording.",
    )
    parser.add_argument("--no-esp32-debug", action="store_true")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--wheel-cmd-topic", default="/wheel_cmd")
    parser.add_argument("--enc-vel-topic", default="/enc_vel")
    parser.add_argument("--esp32-debug-topic", default="/esp32/debug")
    parser.add_argument("--esp32-json-cmd-topic", default="/esp32/json_cmd")
    return parser.parse_args()


def main():
    args = parse_args()
    tests = build_tests(args)
    rclpy.init()
    node = WheelResponseTest(args)
    try:
        node.run(tests)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
