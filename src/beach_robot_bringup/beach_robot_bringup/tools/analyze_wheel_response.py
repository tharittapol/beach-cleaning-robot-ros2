import argparse
import csv
import math
from collections import defaultdict


WHEEL_NAMES = ("fl", "fr", "rl", "rr")
DEFAULT_MAX_REASONABLE_CMD_MPS = 3.0
DEFAULT_MAX_REASONABLE_ENC_MPS = 5.0
DEFAULT_MOTOR_ZERO_EPS = 0.003
DEFAULT_MIN_CMD_MPS = 0.01
CMD_MISMATCH_MPS = 0.02
WHEEL_CIRC_M = {
    "fl": 2.0 * math.pi * 0.115,
    "fr": 2.0 * math.pi * 0.115,
    "rl": math.pi * 0.3202,
    "rr": math.pi * 0.3202,
}
COUNTS_PER_OUTPUT_REV = {
    "fl": 1440.0 * 2.92708333,
    "fr": 1440.0 * 2.97708333,
    "rl": 1440.0,
    "rr": 1440.0,
}


def to_float(value):
    if value is None or value == "":
        return None
    try:
        x = float(value)
    except ValueError:
        return None
    if not math.isfinite(x):
        return None
    return x


def sign_of(value, eps=1e-4):
    if value is None or abs(value) < eps:
        return 0
    return 1 if value > 0.0 else -1


def mean(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    return sum(vals) / len(vals)


def pct(count, total):
    if total <= 0:
        return None
    return 100.0 * count / total


def count_present(values):
    return sum(1 for value in values if value is not None)


def fmt(value, digits=3):
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


def reasonable_abs(value, max_abs):
    return value is not None and abs(value) <= max_abs


def select_command(ros_cmd, dbg_cmd, cmd_source, max_cmd_mps):
    if cmd_source == "debug":
        if reasonable_abs(dbg_cmd, max_cmd_mps):
            return dbg_cmd
        return ros_cmd if reasonable_abs(ros_cmd, max_cmd_mps) else None

    if reasonable_abs(ros_cmd, max_cmd_mps):
        return ros_cmd
    return dbg_cmd if reasonable_abs(dbg_cmd, max_cmd_mps) else None


def command_mismatch(cmd, dbg_cmd, max_cmd_mps):
    if dbg_cmd is None:
        return False
    return not reasonable_abs(dbg_cmd, max_cmd_mps) or abs(dbg_cmd - cmd) > CMD_MISMATCH_MPS


def select_encoder_values(ros_enc_vals, dbg_enc_vals, enc_source):
    if enc_source == "ros":
        return ros_enc_vals, "ros"
    if enc_source == "debug":
        return dbg_enc_vals, "dbg"

    if count_present(dbg_enc_vals) >= max(1, count_present(ros_enc_vals) // 2):
        return dbg_enc_vals, "dbg"
    return ros_enc_vals, "ros"


def debug_elapsed(row_elapsed, debug_age):
    if row_elapsed is None or debug_age is None:
        return row_elapsed
    return row_elapsed - debug_age


def split_segments(rows, max_gap_sec):
    if max_gap_sec <= 0.0:
        return [rows]

    segments = []
    current = []
    last_elapsed = None
    for row in rows:
        elapsed = to_float(row.get("elapsed_sec"))
        if (
            current
            and elapsed is not None
            and last_elapsed is not None
            and (elapsed - last_elapsed) > max_gap_sec
        ):
            segments.append(current)
            current = []
        current.append(row)
        if elapsed is not None:
            last_elapsed = elapsed

    if current:
        segments.append(current)
    return segments


def trim_segment_start(rows, settle_sec):
    if settle_sec <= 0.0 or not rows:
        return rows

    start_elapsed = None
    for row in rows:
        start_elapsed = to_float(row.get("elapsed_sec"))
        if start_elapsed is not None:
            break
    if start_elapsed is None:
        return rows

    return [
        row
        for row in rows
        if (to_float(row.get("elapsed_sec")) is None)
        or (to_float(row.get("elapsed_sec")) - start_elapsed) >= settle_sec
    ]


def segment_duration(samples, value_index):
    vals = [sample[value_index] for sample in samples if sample[value_index] is not None]
    if not vals:
        return None
    return max(vals) - min(vals)


def analyze(
    csv_path,
    phase,
    cmd_source,
    enc_source,
    max_cmd_mps,
    max_enc_mps,
    min_cmd_mps,
    motor_zero_eps,
    drop_cmd_mismatch,
    settle_sec,
    min_segment_sec,
    max_segment_gap_sec,
):
    groups = defaultdict(list)
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("phase") != phase:
                continue
            test = row.get("test", "")
            if not test:
                continue
            groups[test].append(row)

    summaries = []
    for test, rows in groups.items():
        row_segments = [
            trimmed
            for segment in split_segments(rows, max_segment_gap_sec)
            for trimmed in (trim_segment_start(segment, settle_sec),)
            if trimmed
        ]
        if min_segment_sec > 0.0:
            row_segments = [
                segment
                for segment in row_segments
                if (segment_duration([
                    (to_float(row.get("elapsed_sec")),)
                    for row in segment
                ], 0) or 0.0) >= min_segment_sec
            ]
        if not row_segments:
            continue

        for wheel in WHEEL_NAMES:
            ros_cmd_key = f"ros_wheel_cmd_{wheel}"
            dbg_cmd_key = f"dbg_wheel_cmd_{wheel}"
            ros_enc_key = f"ros_enc_vel_{wheel}"
            dbg_enc_key = f"dbg_enc_vel_corr_{wheel}"
            motor_key = f"dbg_motor_u_{wheel}"
            cnt_key = f"dbg_wheel_cnt_{wheel}"

            all_ros_enc_vals = []
            all_dbg_enc_vals = []
            for segment in row_segments:
                all_ros_enc_vals.extend(to_float(r.get(ros_enc_key)) for r in segment)
                all_dbg_enc_vals.extend(to_float(r.get(dbg_enc_key)) for r in segment)
            _, selected_enc_source = select_encoder_values(
                all_ros_enc_vals,
                all_dbg_enc_vals,
                enc_source,
            )

            samples = []
            for segment_idx, segment in enumerate(row_segments):
                elapsed_vals = [to_float(r.get("elapsed_sec")) for r in segment]
                debug_age_vals = [to_float(r.get("debug_age_sec")) for r in segment]
                debug_elapsed_vals = [
                    debug_elapsed(elapsed, debug_age)
                    for elapsed, debug_age in zip(elapsed_vals, debug_age_vals)
                ]
                ros_cmd_vals = [to_float(r.get(ros_cmd_key)) for r in segment]
                dbg_cmd_vals = [to_float(r.get(dbg_cmd_key)) for r in segment]
                ros_enc_vals = [to_float(r.get(ros_enc_key)) for r in segment]
                dbg_enc_vals = [to_float(r.get(dbg_enc_key)) for r in segment]
                motor_vals = [to_float(r.get(motor_key)) for r in segment]
                cnt_vals = [to_float(r.get(cnt_key)) for r in segment]
                enc_vals, _ = select_encoder_values(
                    ros_enc_vals,
                    dbg_enc_vals,
                    enc_source,
                )

                samples.extend(
                    (
                        segment_idx,
                        elapsed,
                        dbg_elapsed,
                        cmd,
                        dbg_cmd,
                        enc,
                        motor,
                        cnt,
                        command_mismatch(cmd, dbg_cmd, max_cmd_mps) if cmd is not None else False,
                    )
                    for elapsed, dbg_elapsed, ros_cmd, dbg_cmd, enc, motor, cnt in zip(
                        elapsed_vals,
                        debug_elapsed_vals,
                        ros_cmd_vals,
                        dbg_cmd_vals,
                        enc_vals,
                        motor_vals,
                        cnt_vals,
                    )
                    for cmd in (select_command(ros_cmd, dbg_cmd, cmd_source, max_cmd_mps),)
                    if enc is not None
                )
            raw_active = [
                (segment_idx, elapsed, dbg_elapsed, cmd, dbg_cmd, enc, motor, cnt, cmd_bad)
                for segment_idx, elapsed, dbg_elapsed, cmd, dbg_cmd, enc, motor, cnt, cmd_bad in samples
                if cmd is not None and abs(cmd) >= min_cmd_mps
            ]
            active = [
                sample
                for sample in raw_active
                if abs(sample[5]) <= max_enc_mps and (not drop_cmd_mismatch or not sample[8])
            ]
            if not active:
                continue

            abs_errors = [abs(cmd - enc) for _, _, _, cmd, _, enc, _, _, _ in active]
            signed_errors = [cmd - enc for _, _, _, cmd, _, enc, _, _, _ in active]
            abs_cmd = [abs(cmd) for _, _, _, cmd, _, _, _, _, _ in active]
            abs_enc = [abs(enc) for _, _, _, _, _, enc, _, _, _ in active]

            motor_samples = [motor for _, _, _, _, _, _, motor, _, _ in active if motor is not None]
            zero_motor = sum(1 for motor in motor_samples if abs(motor) < motor_zero_eps)
            sign_bad = sum(1 for _, _, _, cmd, _, enc, _, _, _ in active if sign_of(cmd) * sign_of(enc) < 0)
            dbg_cmd_samples = [
                (cmd, dbg_cmd, cmd_bad)
                for _, _, _, cmd, dbg_cmd, _, _, _, cmd_bad in raw_active
                if dbg_cmd is not None
            ]
            cmd_bad = sum(1 for _, _, is_bad in dbg_cmd_samples if is_bad)

            motor_flips = 0
            count_jumps = 0
            last_sign = 0
            last_cnt = None
            for _, _, _, _, _, _, motor, cnt, _ in active:
                if motor is None:
                    continue
                s = sign_of(motor, eps=motor_zero_eps)
                if s != 0 and last_sign != 0 and s != last_sign:
                    motor_flips += 1
                if s != 0:
                    last_sign = s
                if cnt is not None and last_cnt is not None and abs(cnt - last_cnt) > 5000:
                    count_jumps += 1
                if cnt is not None:
                    last_cnt = cnt

            ratio = None
            mean_abs_cmd = mean(abs_cmd)
            mean_abs_enc = mean(abs_enc)
            if mean_abs_cmd and mean_abs_cmd > 1e-6 and mean_abs_enc is not None:
                ratio = mean_abs_enc / mean_abs_cmd

            cmd_rev_sum = 0.0
            cmd_rev_present = False
            count_cmd_rev_sum = 0.0
            count_cmd_rev_present = False
            cnt_rev_sum = 0.0
            cnt_rev_present = False
            for segment_idx in sorted({sample[0] for sample in active}):
                segment_samples = [sample for sample in active if sample[0] == segment_idx]
                segment_abs_cmd = [abs(sample[3]) for sample in segment_samples]
                segment_mean_abs_cmd = mean(segment_abs_cmd)
                duration_sec = segment_duration(segment_samples, 1)
                if (
                    segment_mean_abs_cmd is not None
                    and duration_sec is not None
                    and WHEEL_CIRC_M[wheel] > 0.0
                ):
                    cmd_rev_sum += segment_mean_abs_cmd * duration_sec / WHEEL_CIRC_M[wheel]
                    cmd_rev_present = True

                cnt_values = [sample[7] for sample in segment_samples if sample[7] is not None]
                if len(cnt_values) >= 2 and COUNTS_PER_OUTPUT_REV[wheel] > 0.0:
                    segment_cnt_rev = abs(cnt_values[-1] - cnt_values[0]) / COUNTS_PER_OUTPUT_REV[wheel]
                    cnt_rev_sum += segment_cnt_rev
                    cnt_rev_present = True

                    count_duration_sec = segment_duration(segment_samples, 2)
                    if count_duration_sec is None:
                        count_duration_sec = duration_sec
                    if (
                        segment_mean_abs_cmd is not None
                        and count_duration_sec is not None
                        and WHEEL_CIRC_M[wheel] > 0.0
                    ):
                        count_cmd_rev_sum += (
                            segment_mean_abs_cmd * count_duration_sec / WHEEL_CIRC_M[wheel]
                        )
                        count_cmd_rev_present = True

            cmd_rev = cmd_rev_sum if cmd_rev_present else None
            count_cmd_rev = count_cmd_rev_sum if count_cmd_rev_present else None
            cnt_rev = cnt_rev_sum if cnt_rev_present else None

            cnt_ratio = None
            if count_cmd_rev is not None and count_cmd_rev > 1e-6 and cnt_rev is not None:
                cnt_ratio = cnt_rev / count_cmd_rev

            summaries.append({
                "test": test,
                "wheel": wheel,
                "samples": len(active),
                "raw_samples": len(raw_active),
                "drop_pct": pct(len(raw_active) - len(active), len(raw_active)),
                "cmd_abs_mean": mean_abs_cmd,
                "enc_abs_mean": mean_abs_enc,
                "ratio": ratio,
                "error_abs_mean": mean(abs_errors),
                "error_signed_mean": mean(signed_errors),
                "zero_motor_pct": pct(zero_motor, len(motor_samples)),
                "sign_bad_pct": pct(sign_bad, len(active)),
                "cmd_bad_pct": pct(cmd_bad, len(dbg_cmd_samples)),
                "motor_flips": motor_flips,
                "count_jumps": count_jumps,
                "enc_source": selected_enc_source,
                "cmd_rev": cmd_rev,
                "count_cmd_rev": count_cmd_rev,
                "cnt_rev": cnt_rev,
                "cnt_ratio": cnt_ratio,
            })

    return summaries


def print_summary(summaries):
    if not summaries:
        print("No command-phase samples found. Check the CSV phase column and /wheel_cmd, /enc_vel data.")
        return

    header = (
        "test",
        "wheel",
        "encsrc",
        "n",
        "drop%",
        "|cmd|",
        "|enc|",
        "enc/cmd",
        "cmdrev",
        "cntrev",
        "cnt/cmd",
        "mean|err|",
        "mean err",
        "motor0%",
        "signbad%",
        "cmdbad%",
        "uflips",
        "cjumps",
    )
    widths = (14, 5, 6, 5, 6, 8, 8, 8, 7, 7, 7, 9, 9, 8, 9, 8, 6, 6)
    print(" ".join(h.ljust(w) for h, w in zip(header, widths)))
    print(" ".join("-" * w for w in widths))

    for item in summaries:
        values = (
            item["test"],
            item["wheel"],
            item["enc_source"],
            str(item["samples"]),
            fmt(item["drop_pct"], 1),
            fmt(item["cmd_abs_mean"]),
            fmt(item["enc_abs_mean"]),
            fmt(item["ratio"]),
            fmt(item["count_cmd_rev"] if item["cnt_rev"] is not None else item["cmd_rev"], 2),
            fmt(item["cnt_rev"], 2),
            fmt(item["cnt_ratio"], 2),
            fmt(item["error_abs_mean"]),
            fmt(item["error_signed_mean"]),
            fmt(item["zero_motor_pct"], 1),
            fmt(item["sign_bad_pct"], 1),
            fmt(item["cmd_bad_pct"], 1),
            str(item["motor_flips"]),
            str(item["count_jumps"]),
        )
        print(" ".join(v.ljust(w) for v, w in zip(values, widths)))


def parse_args():
    parser = argparse.ArgumentParser(description="Analyze wheel_response_test CSV logs.")
    parser.add_argument("csv_path", help="CSV generated by wheel_response_test.")
    parser.add_argument("--phase", default="command", help="CSV phase to analyze.")
    parser.add_argument(
        "--cmd-source",
        choices=("ros", "debug"),
        default="ros",
        help="Command column used as the reference target. Defaults to ROS /wheel_cmd.",
    )
    parser.add_argument(
        "--enc-source",
        choices=("auto", "ros", "debug"),
        default="auto",
        help="Encoder column used for velocity stats. Defaults to auto.",
    )
    parser.add_argument(
        "--max-cmd-mps",
        type=float,
        default=DEFAULT_MAX_REASONABLE_CMD_MPS,
        help="Reject command samples above this absolute speed as corrupt.",
    )
    parser.add_argument(
        "--max-enc-mps",
        type=float,
        default=DEFAULT_MAX_REASONABLE_ENC_MPS,
        help="Reject encoder velocity samples above this absolute speed as corrupt.",
    )
    parser.add_argument(
        "--min-cmd-mps",
        type=float,
        default=DEFAULT_MIN_CMD_MPS,
        help="Only analyze wheel samples whose selected command magnitude is at least this value.",
    )
    parser.add_argument(
        "--motor-zero-eps",
        type=float,
        default=DEFAULT_MOTOR_ZERO_EPS,
        help="Treat abs(motor_u) below this as zero for motor0% and uflips.",
    )
    parser.add_argument(
        "--drop-cmd-mismatch",
        action="store_true",
        help="Drop rows where debug wheel_cmd differs from the selected ROS command.",
    )
    parser.add_argument(
        "--settle-sec",
        type=float,
        default=0.0,
        help="Drop this many seconds from the start of each contiguous command segment.",
    )
    parser.add_argument(
        "--min-segment-sec",
        type=float,
        default=0.0,
        help="Ignore contiguous command segments shorter than this duration after settling.",
    )
    parser.add_argument(
        "--max-segment-gap-sec",
        type=float,
        default=0.75,
        help="Split repeated manual commands when samples for the same label are separated by more than this gap.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    summaries = analyze(
        args.csv_path,
        args.phase,
        args.cmd_source,
        args.enc_source,
        args.max_cmd_mps,
        args.max_enc_mps,
        args.min_cmd_mps,
        args.motor_zero_eps,
        args.drop_cmd_mismatch,
        args.settle_sec,
        args.min_segment_sec,
        args.max_segment_gap_sec,
    )
    print_summary(summaries)


if __name__ == "__main__":
    main()
