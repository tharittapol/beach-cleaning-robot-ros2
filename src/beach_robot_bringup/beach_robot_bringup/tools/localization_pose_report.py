import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


DEFAULT_SOURCES = (
    ("wheel", "/wheel/odom", "position+yaw"),
    ("imu_bno", "/odometry/bno_imu_only", "yaw_only"),
    ("imu_zed", "/odometry/zed_imu_only", "yaw_only"),
    ("fusion_bno", "/odometry/fusion_bno", "position+yaw"),
    ("fusion_zed", "/odometry/fusion_zed", "position+yaw"),
    ("gnss", "/odometry/gps", "position_only"),
)


@dataclass
class PoseSample:
    t_sec: float
    x: float
    y: float
    yaw_rad: float


def normalize_angle(rad):
    return math.atan2(math.sin(rad), math.cos(rad))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_float(value):
    if value is None or value == "":
        return None
    try:
        x = float(value)
    except ValueError:
        return None
    if not math.isfinite(x):
        return None
    return x


def fmt(value, digits=6):
    if value is None:
        return ""
    return f"{value:.{digits}f}"


def pick_float(row, names):
    for name in names:
        value = parse_float(row.get(name))
        if value is not None:
            return value
    return None


def read_ground_truth(path, trial_id):
    if not path:
        return {}

    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        return {}

    row = None
    if trial_id:
        for candidate in rows:
            if candidate.get("trial_id") == trial_id:
                row = candidate
                break
    if row is None:
        row = rows[0]

    x0 = pick_float(row, ("x0_m", "gt_x0_m", "start_x_m"))
    y0 = pick_float(row, ("y0_m", "gt_y0_m", "start_y_m"))
    yaw0 = pick_float(row, ("yaw0_deg", "gt_yaw0_deg", "start_yaw_deg"))
    x1 = pick_float(row, ("x1_m", "gt_x1_m", "end_x_m"))
    y1 = pick_float(row, ("y1_m", "gt_y1_m", "end_y_m"))
    yaw1 = pick_float(row, ("yaw1_deg", "gt_yaw1_deg", "end_yaw_deg"))

    dx = pick_float(row, ("dx_m", "gt_dx_m"))
    dy = pick_float(row, ("dy_m", "gt_dy_m"))
    dyaw = pick_float(row, ("dyaw_deg", "gt_dyaw_deg"))

    if dx is None and x0 is not None and x1 is not None:
        dx = x1 - x0
    if dy is None and y0 is not None and y1 is not None:
        dy = y1 - y0
    if dyaw is None and yaw0 is not None and yaw1 is not None:
        dyaw = math.degrees(normalize_angle(math.radians(yaw1 - yaw0)))

    distance = pick_float(row, ("distance_m", "gt_distance_m"))
    if distance is None and dx is not None and dy is not None:
        distance = math.hypot(dx, dy)

    return {
        "trial_id": row.get("trial_id", trial_id or ""),
        "x0_m": x0,
        "y0_m": y0,
        "yaw0_deg": yaw0,
        "x1_m": x1,
        "y1_m": y1,
        "yaw1_deg": yaw1,
        "dx_m": dx,
        "dy_m": dy,
        "distance_m": distance,
        "dyaw_deg": dyaw,
    }


def ground_truth_delta(gt, fallback_distance):
    dx = gt.get("dx_m")
    dy = gt.get("dy_m")
    dyaw = gt.get("dyaw_deg")
    distance = gt.get("distance_m")

    if distance is None:
        distance = fallback_distance
    if dx is None:
        dx = distance if distance is not None else None
    if dy is None:
        dy = 0.0 if dx is not None else None
    if dyaw is None:
        dyaw = 0.0

    return dx, dy, distance, dyaw


def read_bag_odometry(bag_path, sources):
    # ROS imports stay local so this module can still be linted/compiled outside
    # a sourced ROS environment.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    wanted_by_topic = {
        topic: (source, role)
        for source, topic, role in sources
        if topic_types.get(topic) == "nav_msgs/msg/Odometry"
    }
    message_classes = {
        topic: get_message(topic_type)
        for topic, topic_type in topic_types.items()
        if topic in wanted_by_topic
    }

    samples = {source: [] for source, _, _ in sources}
    topics_by_source = {source: topic for source, topic, _ in sources}
    roles_by_source = {source: role for source, _, role in sources}

    bag_start_ns = None
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if bag_start_ns is None:
            bag_start_ns = timestamp_ns
        if topic not in wanted_by_topic:
            continue

        msg = deserialize_message(data, message_classes[topic])
        pose = msg.pose.pose
        source, _ = wanted_by_topic[topic]
        samples[source].append(
            PoseSample(
                t_sec=(timestamp_ns - bag_start_ns) / 1e9,
                x=float(pose.position.x),
                y=float(pose.position.y),
                yaw_rad=yaw_from_quaternion(pose.orientation),
            )
        )

    return samples, topics_by_source, roles_by_source


def filter_samples(samples, start_sec, end_sec, settle_sec):
    filtered = []
    for sample in samples:
        if start_sec is not None and sample.t_sec < start_sec:
            continue
        if end_sec is not None and sample.t_sec > end_sec:
            continue
        filtered.append(sample)

    if not filtered:
        return filtered

    min_t = filtered[0].t_sec + settle_sec
    return [sample for sample in filtered if sample.t_sec >= min_t]


def interpolate_angle(a, b, ratio):
    return normalize_angle(a + normalize_angle(b - a) * ratio)


def sample_at_time(samples, t_sec):
    if not samples:
        return None
    if t_sec <= samples[0].t_sec:
        return samples[0]
    if t_sec >= samples[-1].t_sec:
        return samples[-1]

    lo = 0
    hi = len(samples) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if samples[mid].t_sec <= t_sec:
            lo = mid
        else:
            hi = mid

    a = samples[lo]
    b = samples[hi]
    span = b.t_sec - a.t_sec
    ratio = 0.0 if span <= 0.0 else (t_sec - a.t_sec) / span
    return PoseSample(
        t_sec=t_sec,
        x=a.x + (b.x - a.x) * ratio,
        y=a.y + (b.y - a.y) * ratio,
        yaw_rad=interpolate_angle(a.yaw_rad, b.yaw_rad, ratio),
    )


def common_window(filtered_samples_by_source):
    starts = [samples[0].t_sec for samples in filtered_samples_by_source.values() if samples]
    ends = [samples[-1].t_sec for samples in filtered_samples_by_source.values() if samples]
    if not starts or not ends:
        return None, None
    t0 = max(starts)
    t1 = min(ends)
    if t1 <= t0:
        return min(starts), max(ends)
    return t0, t1


def meter_values(distance, step):
    if distance is None or distance <= 0.0 or step <= 0.0:
        return []
    values = []
    m = 0.0
    while m < distance:
        values.append(m)
        m += step
    if not values or abs(values[-1] - distance) > 1e-6:
        values.append(distance)
    return values


def gt_pose_at_meter(meter, gt_dx, gt_dy, gt_distance, gt_dyaw):
    if gt_distance is None or gt_distance <= 0.0:
        ratio = 0.0
    else:
        ratio = max(0.0, min(1.0, meter / gt_distance))
    return (
        (gt_dx or 0.0) * ratio,
        (gt_dy or 0.0) * ratio,
        (gt_dyaw or 0.0) * ratio,
    )


def along_cross_errors(err_x, err_y, gt_dx, gt_dy, gt_distance):
    if gt_distance is None or gt_distance <= 1e-9 or gt_dx is None or gt_dy is None:
        return None, None
    ux = gt_dx / gt_distance
    uy = gt_dy / gt_distance
    along = err_x * ux + err_y * uy
    cross = err_x * -uy + err_y * ux
    return along, cross


def summarize_source(source, topic, role, raw_samples, gt, start_sec, end_sec, settle_sec):
    samples = filter_samples(raw_samples, start_sec, end_sec, settle_sec)
    if not samples:
        return {
            "source": source,
            "topic": topic,
            "role": role,
            "n": 0,
        }

    first = samples[0]
    last = samples[-1]
    dx = last.x - first.x
    dy = last.y - first.y
    distance = math.hypot(dx, dy)
    dyaw_deg = math.degrees(normalize_angle(last.yaw_rad - first.yaw_rad))

    gt_dx = gt.get("dx_m")
    gt_dy = gt.get("dy_m")
    gt_distance = gt.get("distance_m")
    gt_dyaw = gt.get("dyaw_deg")

    position_reliable = role in ("position+yaw", "position_only")
    yaw_reliable = role in ("position+yaw", "yaw_only")

    return {
        "source": source,
        "topic": topic,
        "role": role,
        "n": len(samples),
        "t0_sec": first.t_sec,
        "t1_sec": last.t_sec,
        "duration_sec": last.t_sec - first.t_sec,
        "x0_m": first.x,
        "y0_m": first.y,
        "yaw0_deg": math.degrees(first.yaw_rad),
        "x1_m": last.x,
        "y1_m": last.y,
        "yaw1_deg": math.degrees(last.yaw_rad),
        "dx_m": dx,
        "dy_m": dy,
        "distance_m": distance,
        "dyaw_deg": dyaw_deg,
        "gt_dx_m": gt_dx,
        "gt_dy_m": gt_dy,
        "gt_distance_m": gt_distance,
        "gt_dyaw_deg": gt_dyaw,
        "err_dx_m": dx - gt_dx if position_reliable and gt_dx is not None else None,
        "err_dy_m": dy - gt_dy if position_reliable and gt_dy is not None else None,
        "err_distance_m": distance - gt_distance
        if position_reliable and gt_distance is not None
        else None,
        "err_dyaw_deg": math.degrees(
            normalize_angle(math.radians(dyaw_deg - gt_dyaw))
        )
        if yaw_reliable and gt_dyaw is not None
        else None,
    }


def build_meter_rows(
    samples_by_source,
    topics_by_source,
    roles_by_source,
    gt,
    trial_id,
    start_sec,
    end_sec,
    settle_sec,
    straight_distance_m,
    meter_step_m,
):
    gt_dx, gt_dy, gt_distance, gt_dyaw = ground_truth_delta(gt, straight_distance_m)
    meters = meter_values(gt_distance, meter_step_m)
    if not meters:
        return [], {}, (None, None)

    filtered = {
        source: filter_samples(samples, start_sec, end_sec, settle_sec)
        for source, samples in samples_by_source.items()
    }
    t0, t1 = common_window(filtered)
    if t0 is None or t1 is None:
        return [], filtered, (None, None)

    rows = []
    for source, samples in filtered.items():
        if not samples:
            continue
        role = roles_by_source[source]
        topic = topics_by_source[source]
        origin = sample_at_time(samples, t0)
        if origin is None:
            continue
        position_reliable = role in ("position+yaw", "position_only")
        yaw_reliable = role in ("position+yaw", "yaw_only")

        for meter in meters:
            ratio = 0.0 if gt_distance <= 0.0 else meter / gt_distance
            t_sec = t0 + (t1 - t0) * ratio
            sample = sample_at_time(samples, t_sec)
            if sample is None:
                continue

            x_rel = sample.x - origin.x
            y_rel = sample.y - origin.y
            yaw_rel = math.degrees(normalize_angle(sample.yaw_rad - origin.yaw_rad))
            gt_x, gt_y, gt_yaw = gt_pose_at_meter(meter, gt_dx, gt_dy, gt_distance, gt_dyaw)

            err_x = x_rel - gt_x if position_reliable else None
            err_y = y_rel - gt_y if position_reliable else None
            err_along, err_cross = (
                along_cross_errors(err_x, err_y, gt_dx, gt_dy, gt_distance)
                if position_reliable
                else (None, None)
            )
            err_yaw = (
                math.degrees(normalize_angle(math.radians(yaw_rel - gt_yaw)))
                if yaw_reliable
                else None
            )

            rows.append({
                "trial_id": trial_id,
                "meter_m": meter,
                "source": source,
                "topic": topic,
                "role": role,
                "t_sec": t_sec,
                "x_rel_m": x_rel,
                "y_rel_m": y_rel,
                "yaw_rel_deg": yaw_rel,
                "gt_x_m": gt_x,
                "gt_y_m": gt_y,
                "gt_yaw_deg": gt_yaw,
                "err_x_m": err_x,
                "err_y_m": err_y,
                "err_along_m": err_along,
                "err_cross_m": err_cross,
                "err_yaw_deg": err_yaw,
            })

    return rows, filtered, (t0, t1)


def write_summary(path, rows, trial_id):
    fieldnames = (
        "trial_id",
        "source",
        "topic",
        "role",
        "n",
        "t0_sec",
        "t1_sec",
        "duration_sec",
        "x0_m",
        "y0_m",
        "yaw0_deg",
        "x1_m",
        "y1_m",
        "yaw1_deg",
        "dx_m",
        "dy_m",
        "distance_m",
        "dyaw_deg",
        "gt_dx_m",
        "gt_dy_m",
        "gt_distance_m",
        "gt_dyaw_deg",
        "err_dx_m",
        "err_dy_m",
        "err_distance_m",
        "err_dyaw_deg",
    )
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            out = {"trial_id": trial_id or ""}
            out.update(row)
            writer.writerow({
                key: fmt(out.get(key)) if isinstance(out.get(key), float) else out.get(key, "")
                for key in fieldnames
            })


def write_meter_table(path, rows):
    fieldnames = (
        "trial_id",
        "meter_m",
        "source",
        "topic",
        "role",
        "t_sec",
        "x_rel_m",
        "y_rel_m",
        "yaw_rel_deg",
        "gt_x_m",
        "gt_y_m",
        "gt_yaw_deg",
        "err_x_m",
        "err_y_m",
        "err_along_m",
        "err_cross_m",
        "err_yaw_deg",
    )
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: fmt(row.get(key)) if isinstance(row.get(key), float) else row.get(key, "")
                for key in fieldnames
            })


def write_samples(path, samples_by_source, topics_by_source, roles_by_source, start_sec, end_sec, settle_sec):
    fieldnames = (
        "source",
        "topic",
        "role",
        "t_sec",
        "x_m",
        "y_m",
        "yaw_deg",
        "x_rel_m",
        "y_rel_m",
        "yaw_rel_deg",
    )
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for source, raw_samples in samples_by_source.items():
            samples = filter_samples(raw_samples, start_sec, end_sec, settle_sec)
            if not samples:
                continue
            first = samples[0]
            for sample in samples:
                writer.writerow({
                    "source": source,
                    "topic": topics_by_source[source],
                    "role": roles_by_source[source],
                    "t_sec": fmt(sample.t_sec),
                    "x_m": fmt(sample.x),
                    "y_m": fmt(sample.y),
                    "yaw_deg": fmt(math.degrees(sample.yaw_rad)),
                    "x_rel_m": fmt(sample.x - first.x),
                    "y_rel_m": fmt(sample.y - first.y),
                    "yaw_rel_deg": fmt(math.degrees(normalize_angle(sample.yaw_rad - first.yaw_rad))),
                })


def decimate(points, max_points=700):
    if len(points) <= max_points:
        return points
    step = max(1, len(points) // max_points)
    reduced = points[::step]
    if reduced[-1] != points[-1]:
        reduced.append(points[-1])
    return reduced


def svg_escape(value):
    return str(value).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def write_pose_svg(
    path,
    filtered_samples_by_source,
    topics_by_source,
    roles_by_source,
    meter_rows,
    gt,
    straight_distance_m,
    t_window,
):
    width = 1000
    height = 700
    margin = 70
    colors = {
        "wheel": "#d95f02",
        "fusion_bno": "#1b9e77",
        "fusion_zed": "#7570b3",
        "gnss": "#e7298a",
        "imu_bno": "#666666",
        "imu_zed": "#999999",
    }

    gt_dx, gt_dy, gt_distance, _ = ground_truth_delta(gt, straight_distance_m)
    points_by_source = {}
    all_points = [(0.0, 0.0)]
    if gt_dx is not None and gt_dy is not None:
        all_points.append((gt_dx, gt_dy))

    t0, t1 = t_window
    for source, samples in filtered_samples_by_source.items():
        if roles_by_source[source] not in ("position+yaw", "position_only"):
            continue
        if not samples:
            continue
        origin = sample_at_time(samples, t0 if t0 is not None else samples[0].t_sec)
        if origin is None:
            continue
        pts = []
        for sample in samples:
            if t0 is not None and sample.t_sec < t0:
                continue
            if t1 is not None and sample.t_sec > t1:
                continue
            pts.append((sample.x - origin.x, sample.y - origin.y))
        points_by_source[source] = decimate(pts)
        all_points.extend(pts)

    if not all_points:
        return

    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    pad_x = max(0.5, (max_x - min_x) * 0.08)
    pad_y = max(0.5, (max_y - min_y) * 0.20)
    min_x -= pad_x
    max_x += pad_x
    min_y -= pad_y
    max_y += pad_y

    def sx(x):
        return margin + (x - min_x) / max(max_x - min_x, 1e-9) * (width - 2 * margin)

    def sy(y):
        return height - margin - (y - min_y) / max(max_y - min_y, 1e-9) * (height - 2 * margin)

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Arial,sans-serif;font-size:14px}.small{font-size:11px}.axis{stroke:#cccccc;stroke-width:1}.gt{stroke:#111;stroke-width:3;stroke-dasharray:8 6;fill:none}.path{fill:none;stroke-width:2.5}.marker{stroke:white;stroke-width:1}</style>',
        f'<text x="{margin}" y="34" font-size="22">Straight-line localization pose report</text>',
    ]

    # Axes.
    lines.append(f'<line class="axis" x1="{sx(min_x)}" y1="{sy(0)}" x2="{sx(max_x)}" y2="{sy(0)}"/>')
    lines.append(f'<line class="axis" x1="{sx(0)}" y1="{sy(min_y)}" x2="{sx(0)}" y2="{sy(max_y)}"/>')
    lines.append(f'<text class="small" x="{sx(max_x) - 70:.1f}" y="{sy(0) - 8:.1f}">x rel (m)</text>')
    lines.append(f'<text class="small" x="{sx(0) + 8:.1f}" y="{sy(max_y) + 18:.1f}">y rel (m)</text>')

    if gt_dx is not None and gt_dy is not None:
        lines.append(f'<polyline class="gt" points="{sx(0):.1f},{sy(0):.1f} {sx(gt_dx):.1f},{sy(gt_dy):.1f}"/>')
        if gt_distance is not None and gt_distance > 0.0:
            for row in meter_rows:
                if row.get("source") != "wheel":
                    continue
                gx = row.get("gt_x_m")
                gy = row.get("gt_y_m")
                meter = row.get("meter_m")
                lines.append(f'<circle cx="{sx(gx):.1f}" cy="{sy(gy):.1f}" r="3.5" fill="#111"/>')
                lines.append(f'<text class="small" x="{sx(gx) - 5:.1f}" y="{sy(gy) - 9:.1f}">{meter:.0f}</text>')

    for source, pts in points_by_source.items():
        if len(pts) < 2:
            continue
        color = colors.get(source, "#333333")
        point_text = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in pts)
        lines.append(f'<polyline class="path" points="{point_text}" stroke="{color}"/>')

    for row in meter_rows:
        source = row.get("source")
        if roles_by_source.get(source) not in ("position+yaw", "position_only"):
            continue
        color = colors.get(source, "#333333")
        x = row.get("x_rel_m")
        y = row.get("y_rel_m")
        meter = row.get("meter_m")
        if x is None or y is None or meter is None:
            continue
        lines.append(f'<circle class="marker" cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="4" fill="{color}"/>')

    legend_x = width - 250
    legend_y = 60
    lines.append(f'<rect x="{legend_x - 12}" y="{legend_y - 25}" width="220" height="170" fill="white" stroke="#dddddd"/>')
    lines.append(f'<text x="{legend_x}" y="{legend_y - 5}">Legend</text>')
    legend_items = [("ground truth", "#111", "dashed")]
    legend_items.extend((source, colors.get(source, "#333333"), "solid") for source in points_by_source)
    for idx, (label, color, style) in enumerate(legend_items):
        y = legend_y + 20 + idx * 22
        dash = ' stroke-dasharray="7 5"' if style == "dashed" else ""
        lines.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 35}" y2="{y}" stroke="{color}" stroke-width="3"{dash}/>')
        lines.append(f'<text class="small" x="{legend_x + 45}" y="{y + 4}">{svg_escape(label)}</text>')

    lines.append(f'<text class="small" x="{margin}" y="{height - 24}">Meter markers use the common trial time window. IMU-only sources are yaw-only and are not drawn as x-y trajectories.</text>')
    lines.append("</svg>")

    with open(path, "w") as f:
        f.write("\n".join(lines))


def default_output_path(bag_path):
    bag = Path(bag_path)
    if bag.is_dir():
        return bag / "pose_report_summary.csv"
    return bag.with_name(f"{bag.stem}_pose_report_summary.csv")


def parse_source(value):
    parts = value.split(":")
    if len(parts) == 2:
        source, topic = parts
        role = "position+yaw"
    elif len(parts) == 3:
        source, topic, role = parts
    else:
        raise argparse.ArgumentTypeError("Use source:/topic or source:/topic:role")
    if role not in ("position+yaw", "yaw_only", "position_only"):
        raise argparse.ArgumentTypeError("role must be position+yaw, yaw_only, or position_only")
    return source, topic, role


def main():
    parser = argparse.ArgumentParser(
        description="Export report-ready localization pose summaries from a ROS 2 bag."
    )
    parser.add_argument("bag", help="ROS 2 bag directory or .db3 path.")
    parser.add_argument("--trial-id", default="", help="Trial id to write and match in ground truth CSV.")
    parser.add_argument("--ground-truth", help="CSV with trial_id and dx_m/dy_m/dyaw_deg or start/end pose columns.")
    parser.add_argument("--out", help="Output summary CSV path.")
    parser.add_argument("--samples-out", help="Optional per-sample CSV for plotting trajectories.")
    parser.add_argument("--meters-out", help="Optional per-meter CSV table for straight-line report.")
    parser.add_argument("--plot-out", help="Optional SVG x-y trajectory plot with 1 m markers.")
    parser.add_argument("--straight-distance-m", type=float, help="Ground-truth straight distance if no ground truth CSV is provided.")
    parser.add_argument("--meter-step-m", type=float, default=1.0, help="Distance step for --meters-out and --plot-out.")
    parser.add_argument("--start-sec", type=float, help="Crop start time, seconds from bag start.")
    parser.add_argument("--end-sec", type=float, help="Crop end time, seconds from bag start.")
    parser.add_argument("--settle-sec", type=float, default=0.0, help="Trim this many seconds after crop start.")
    parser.add_argument(
        "--source",
        action="append",
        type=parse_source,
        help="Override/add source mapping as source:/topic[:role]. May be repeated.",
    )
    args = parser.parse_args()

    sources = tuple(args.source) if args.source else DEFAULT_SOURCES
    samples_by_source, topics_by_source, roles_by_source = read_bag_odometry(args.bag, sources)
    gt = read_ground_truth(args.ground_truth, args.trial_id)
    trial_id = args.trial_id or gt.get("trial_id", "")

    rows = [
        summarize_source(
            source,
            topic,
            role,
            samples_by_source[source],
            gt,
            args.start_sec,
            args.end_sec,
            args.settle_sec,
        )
        for source, topic, role in sources
    ]

    out_path = Path(args.out) if args.out else default_output_path(args.bag)
    write_summary(out_path, rows, trial_id)

    if args.samples_out:
        write_samples(
            Path(args.samples_out),
            samples_by_source,
            topics_by_source,
            roles_by_source,
            args.start_sec,
            args.end_sec,
            args.settle_sec,
        )

    meter_rows, filtered, t_window = build_meter_rows(
        samples_by_source,
        topics_by_source,
        roles_by_source,
        gt,
        trial_id,
        args.start_sec,
        args.end_sec,
        args.settle_sec,
        args.straight_distance_m,
        args.meter_step_m,
    )
    if args.meters_out:
        write_meter_table(Path(args.meters_out), meter_rows)
    if args.plot_out:
        write_pose_svg(
            Path(args.plot_out),
            filtered,
            topics_by_source,
            roles_by_source,
            meter_rows,
            gt,
            args.straight_distance_m,
            t_window,
        )

    print(f"Wrote {out_path}")
    if args.samples_out:
        print(f"Wrote {args.samples_out}")
    if args.meters_out:
        print(f"Wrote {args.meters_out}")
    if args.plot_out:
        print(f"Wrote {args.plot_out}")


if __name__ == "__main__":
    main()
