import argparse
import csv
import math
from collections import defaultdict


WHEELS = ("fl", "fr", "rl", "rr")
FRONT = ("fl", "fr")
REAR = ("rl", "rr")


def to_float(value):
    if value in (None, ""):
        return None
    try:
        x = float(value)
    except ValueError:
        return None
    return x if math.isfinite(x) else None


def mean(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    return sum(vals) / len(vals)


def counter_delta(values):
    vals = [v for v in values if v is not None]
    if len(vals) < 2:
        return None
    first = vals[0]
    last = vals[-1]
    if last >= first:
        return last - first
    return last


def pct(count, total):
    if total <= 0:
        return None
    return 100.0 * count / total


def fmt(value, digits=3):
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


def split_segments(rows, max_gap_sec):
    segments = []
    current = []
    last_t = None
    for row in rows:
        t = to_float(row.get("elapsed_sec"))
        if current and t is not None and last_t is not None and (t - last_t) > max_gap_sec:
            segments.append(current)
            current = []
        current.append(row)
        if t is not None:
            last_t = t
    if current:
        segments.append(current)
    return segments


def trim_start(rows, settle_sec):
    if settle_sec <= 0.0 or not rows:
        return rows
    start = next((to_float(row.get("elapsed_sec")) for row in rows if to_float(row.get("elapsed_sec")) is not None), None)
    if start is None:
        return rows
    return [
        row for row in rows
        if to_float(row.get("elapsed_sec")) is None
        or (to_float(row.get("elapsed_sec")) - start) >= settle_sec
    ]


def duration(rows):
    ts = [to_float(row.get("elapsed_sec")) for row in rows]
    ts = [t for t in ts if t is not None]
    if not ts:
        return None
    return max(ts) - min(ts)


def column(prefix, wheel):
    return f"{prefix}_{wheel}"


def selected_value(row, primary_key, fallback_key=None):
    value = to_float(row.get(primary_key))
    if value is not None:
        return value
    if fallback_key is None:
        return None
    return to_float(row.get(fallback_key))


def analyze_segment(rows, args):
    wheel_stats = {}
    for wheel in WHEELS:
        samples = []
        for row in rows:
            if args.cmd_source == "debug":
                cmd = selected_value(row, column("dbg_wheel_cmd", wheel), column("ros_wheel_cmd", wheel))
            else:
                cmd = selected_value(row, column("ros_wheel_cmd", wheel), column("dbg_wheel_cmd", wheel))

            if args.enc_source == "debug":
                enc = selected_value(row, column("dbg_enc_vel_corr", wheel), column("ros_enc_vel", wheel))
            else:
                enc = selected_value(row, column("ros_enc_vel", wheel), column("dbg_enc_vel_corr", wheel))

            motor_u = to_float(row.get(column("dbg_motor_u", wheel)))
            if cmd is None or enc is None or abs(cmd) < args.min_cmd_mps:
                continue

            cmd_sign = 1.0 if cmd > 0.0 else -1.0
            enc_along_cmd = enc * cmd_sign
            samples.append((abs(cmd), enc_along_cmd, abs(enc), motor_u))

        cmd_mean = mean(s[0] for s in samples)
        enc_along_mean = mean(s[1] for s in samples)
        enc_abs_mean = mean(s[2] for s in samples)
        motor_abs_mean = mean(abs(s[3]) for s in samples if s[3] is not None)
        sign_bad = sum(1 for _, enc_along, _, _ in samples if enc_along < -args.sign_eps_mps)
        stall = sum(1 for _, enc_along, _, _ in samples if enc_along < args.stall_enc_mps)
        motor_zero = sum(1 for _, _, _, motor_u in samples if motor_u is not None and abs(motor_u) < args.motor_zero_eps)
        motor_count = sum(1 for _, _, _, motor_u in samples if motor_u is not None)

        wheel_stats[wheel] = {
            "n": len(samples),
            "cmd": cmd_mean,
            "enc": enc_along_mean,
            "enc_abs": enc_abs_mean,
            "ratio": (enc_along_mean / cmd_mean) if cmd_mean and enc_along_mean is not None else None,
            "err": (cmd_mean - enc_along_mean) if cmd_mean is not None and enc_along_mean is not None else None,
            "motor": motor_abs_mean,
            "stall_pct": pct(stall, len(samples)),
            "signbad_pct": pct(sign_bad, len(samples)),
            "motor0_pct": pct(motor_zero, motor_count),
        }

    return wheel_stats


def analyze_mode(rows, args):
    valid = []
    for row in rows:
        in_place = to_float(row.get("dbg_in_place_turn"))
        moving = to_float(row.get("dbg_moving_turn"))
        boost = to_float(row.get("dbg_spin_start_boost"))
        if in_place is None and moving is None and boost is None:
            continue
        debug_age = to_float(row.get("debug_age_sec"))
        if debug_age is not None and debug_age > args.max_debug_age_sec:
            continue
        valid.append((row, in_place, moving, boost, debug_age))

    if not valid:
        return {
            "n": 0,
            "fresh_pct": None,
            "inplace_pct": None,
            "moving_pct": None,
            "boost_pct": None,
            "debug_age": None,
            "cmd_vel_age": None,
            "wheel_cmd_age": None,
            "cmd_age_ms": None,
            "enc_age_ms": None,
            "cmd_rx_interval_ms": None,
            "cmd_seq_gap_delta": None,
            "usb_parse_errors_delta": None,
            "usb_line_overflows_delta": None,
        }

    fresh_count = len(valid)
    debug_age_mean = mean(item[4] for item in valid)
    return {
        "n": fresh_count,
        "fresh_pct": pct(fresh_count, len(rows)),
        "inplace_pct": pct(sum(1 for item in valid if item[1] is not None and item[1] >= 0.5), fresh_count),
        "moving_pct": pct(sum(1 for item in valid if item[2] is not None and item[2] >= 0.5), fresh_count),
        "boost_pct": pct(sum(1 for item in valid if item[3] is not None and item[3] >= 0.5), fresh_count),
        "debug_age": debug_age_mean,
        "cmd_vel_age": mean(to_float(row.get("cmd_vel_age_sec")) for row in rows),
        "wheel_cmd_age": mean(to_float(row.get("wheel_cmd_age_sec")) for row in rows),
        "cmd_age_ms": mean(to_float(item[0].get("dbg_cmd_age_ms")) for item in valid),
        "enc_age_ms": mean(to_float(item[0].get("dbg_enc_age_ms")) for item in valid),
        "cmd_rx_interval_ms": mean(to_float(item[0].get("dbg_cmd_rx_interval_ms")) for item in valid),
        "cmd_seq_gap_delta": counter_delta(to_float(item[0].get("dbg_cmd_seq_gap")) for item in valid),
        "usb_parse_errors_delta": counter_delta(to_float(item[0].get("dbg_usb_parse_errors")) for item in valid),
        "usb_line_overflows_delta": counter_delta(to_float(item[0].get("dbg_usb_line_overflows")) for item in valid),
    }


def pair_mean(wheel_stats, wheels, key):
    return mean(wheel_stats[w][key] for w in wheels)


def print_table(rows):
    widths = [max(len(str(row[i])) for row in rows) for i in range(len(rows[0]))]
    for idx, row in enumerate(rows):
        print("  ".join(str(cell).ljust(widths[i]) for i, cell in enumerate(row)))
        if idx == 0:
            print("  ".join("-" * widths[i] for i in range(len(row))))


def hint_for_segment(name, wheel_stats, args):
    front_ratio = pair_mean(wheel_stats, FRONT, "ratio")
    rear_ratio = pair_mean(wheel_stats, REAR, "ratio")
    front_enc = pair_mean(wheel_stats, FRONT, "enc")
    rear_enc = pair_mean(wheel_stats, REAR, "enc")
    hints = []

    if front_ratio is not None and rear_ratio is not None:
        if front_ratio > rear_ratio * args.imbalance_ratio:
            hints.append("front dominates: reduce front in-place gain/floor or add rear gain/floor")
        elif rear_ratio > front_ratio * args.imbalance_ratio:
            hints.append("rear dominates: reduce rear in-place gain/floor or add front gain/floor")

    if front_enc is not None and rear_enc is not None and min(front_enc, rear_enc) > 0.0:
        enc_ratio = front_enc / rear_enc
        if enc_ratio > args.imbalance_ratio:
            hints.append("front encoder speed is higher than rear")
        elif enc_ratio < 1.0 / args.imbalance_ratio:
            hints.append("rear encoder speed is higher than front")

    for wheel in WHEELS:
        stall_pct = wheel_stats[wheel]["stall_pct"]
        if stall_pct is not None and stall_pct >= args.warn_stall_pct:
            motor_abs = wheel_stats[wheel]["motor"]
            motor0_pct = wheel_stats[wheel]["motor0_pct"]
            if motor_abs is not None and motor_abs >= args.warn_motor_high_u:
                hints.append(
                    f"{wheel} stalls {stall_pct:.0f}% with |u|={motor_abs:.2f}: "
                    "inspect motor/driver/binding/traction before increasing floor"
                )
            elif motor0_pct is not None and motor0_pct >= args.warn_motor0_pct:
                hints.append(
                    f"{wheel} stalls {stall_pct:.0f}% and motor is often zero: "
                    "target/floor may be too low or PID is braking"
                )
            else:
                hints.append(f"{wheel} stalls {stall_pct:.0f}%: increase that wheel hold floor or inspect motor/traction")
        signbad_pct = wheel_stats[wheel]["signbad_pct"]
        if signbad_pct is not None and signbad_pct >= args.warn_signbad_pct:
            hints.append(f"{wheel} sign mismatch {signbad_pct:.0f}%: check encoder/motor sign or wheel dragging")

    if not hints:
        hints.append("balance looks usable; tune yaw rate/odom scale next")
    return f"{name}: " + "; ".join(hints)


def main():
    parser = argparse.ArgumentParser(description="Analyze spin-in-place front/rear wheel balance from wheel_response_test CSV.")
    parser.add_argument("csv_path")
    parser.add_argument("--phase", default="command")
    parser.add_argument("--enc-source", choices=("ros", "debug"), default="ros")
    parser.add_argument("--cmd-source", choices=("ros", "debug"), default="ros")
    parser.add_argument("--settle-sec", type=float, default=0.5)
    parser.add_argument("--min-segment-sec", type=float, default=1.0)
    parser.add_argument("--max-segment-gap-sec", type=float, default=0.5)
    parser.add_argument("--min-cmd-mps", type=float, default=0.02)
    parser.add_argument("--stall-enc-mps", type=float, default=0.02)
    parser.add_argument("--sign-eps-mps", type=float, default=0.01)
    parser.add_argument("--motor-zero-eps", type=float, default=0.003)
    parser.add_argument("--imbalance-ratio", type=float, default=1.25)
    parser.add_argument("--warn-stall-pct", type=float, default=25.0)
    parser.add_argument("--warn-signbad-pct", type=float, default=10.0)
    parser.add_argument("--warn-motor-high-u", type=float, default=0.30)
    parser.add_argument("--warn-motor0-pct", type=float, default=50.0)
    parser.add_argument(
        "--max-debug-age-sec",
        type=float,
        default=1.5,
        help="Ignore ESP32 debug mode samples older than this when reporting mode percentages.",
    )
    args = parser.parse_args()

    grouped = defaultdict(list)
    with open(args.csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if row.get("phase") != args.phase:
                continue
            test = row.get("test", "")
            if "spin" not in test:
                continue
            grouped[test].append(row)

    if not grouped:
        print("No spin command samples found. Record spin_left/spin_right with wheel_response_test first.")
        return

    mode_rows = [[
        "test", "seg", "dbg_n", "fresh%", "inplace%", "moving%", "boost%",
        "cmd_age", "rx_dt", "seq_gap_d", "parse_d", "overflow_d",
        "enc_age", "dbg_age", "cmdvel_age", "wheel_age",
    ]]
    output_rows = [[
        "test", "seg", "wheel", "n", "|cmd|", "enc_along", "enc/cmd",
        "err", "|u|", "stall%", "signbad%", "motor0%",
    ]]
    hints = []

    for test in sorted(grouped):
        segments = []
        for segment in split_segments(grouped[test], args.max_segment_gap_sec):
            segment = trim_start(segment, args.settle_sec)
            if not segment:
                continue
            seg_duration = duration(segment) or 0.0
            if seg_duration >= args.min_segment_sec:
                segments.append(segment)

        for seg_idx, segment in enumerate(segments, start=1):
            stats = analyze_segment(segment, args)
            mode = analyze_mode(segment, args)
            mode_rows.append([
                test,
                seg_idx,
                mode["n"],
                fmt(mode["fresh_pct"], 1),
                fmt(mode["inplace_pct"], 1),
                fmt(mode["moving_pct"], 1),
                fmt(mode["boost_pct"], 1),
                fmt(mode["cmd_age_ms"], 0),
                fmt(mode["cmd_rx_interval_ms"], 0),
                fmt(mode["cmd_seq_gap_delta"], 0),
                fmt(mode["usb_parse_errors_delta"], 0),
                fmt(mode["usb_line_overflows_delta"], 0),
                fmt(mode["enc_age_ms"], 0),
                fmt(mode["debug_age"]),
                fmt(mode["cmd_vel_age"]),
                fmt(mode["wheel_cmd_age"]),
            ])
            for wheel in WHEELS:
                s = stats[wheel]
                output_rows.append([
                    test,
                    seg_idx,
                    wheel,
                    s["n"],
                    fmt(s["cmd"]),
                    fmt(s["enc"]),
                    fmt(s["ratio"]),
                    fmt(s["err"]),
                    fmt(s["motor"]),
                    fmt(s["stall_pct"], 1),
                    fmt(s["signbad_pct"], 1),
                    fmt(s["motor0_pct"], 1),
                ])

            output_rows.append([
                test,
                seg_idx,
                "front",
                "-",
                fmt(pair_mean(stats, FRONT, "cmd")),
                fmt(pair_mean(stats, FRONT, "enc")),
                fmt(pair_mean(stats, FRONT, "ratio")),
                fmt(pair_mean(stats, FRONT, "err")),
                fmt(pair_mean(stats, FRONT, "motor")),
                "-",
                "-",
                "-",
            ])
            output_rows.append([
                test,
                seg_idx,
                "rear",
                "-",
                fmt(pair_mean(stats, REAR, "cmd")),
                fmt(pair_mean(stats, REAR, "enc")),
                fmt(pair_mean(stats, REAR, "ratio")),
                fmt(pair_mean(stats, REAR, "err")),
                fmt(pair_mean(stats, REAR, "motor")),
                "-",
                "-",
                "-",
            ])
            hints.append(hint_for_segment(f"{test} seg {seg_idx}", stats, args))
            if mode["inplace_pct"] is not None and mode["inplace_pct"] < 80.0:
                hints.append(
                    f"{test} seg {seg_idx}: ESP32 in_place_turn only {mode['inplace_pct']:.0f}% "
                    "of fresh debug samples; check creep-pivot detection before tuning floors"
                )

    print("Mode:")
    print_table(mode_rows)
    print()
    print_table(output_rows)
    print()
    print("Hints:")
    for hint in hints:
        print(f"- {hint}")


if __name__ == "__main__":
    main()
