#!/usr/bin/env python3
"""
Reads a ROS2 bag from a boustrophedon coverage run and produces CSV tables
suitable for coverage-accuracy reporting and verification.

Output files in <out_dir>/:
  pose_trajectory.csv   — robot pose time series with phase labels
  cmd_vel.csv           — navigation commands (/cmd_vel)
  wheel_speeds.csv      — wheel cmd + encoder velocities
  lane_tracking.csv     — per-lane planned vs actual y, accuracy, timing
  turn_tracking.csv     — per-turn timing and y-offset error
  coverage_summary.csv  — key-value mission statistics

Usage:
  ros2 run beach_robot_bringup coverage_bag_report <bag_path> [--out <dir>]
"""
import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


# ─── helpers ──────────────────────────────────────────────────────────────────

def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _lerp(a, b, r):
    return a + (b - a) * r


def fmt(v, d=4):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return ''
    return f'{v:.{d}f}'


# ─── data classes ─────────────────────────────────────────────────────────────

@dataclass
class PosePoint:
    t: float
    x: float
    y: float
    yaw: float
    phase: str = ''   # filled in after phase detection


@dataclass
class CmdVelPoint:
    t: float
    linear_x: float
    angular_z: float


@dataclass
class WheelPoint:
    t: float
    FL: float
    FR: float
    RL: float
    RR: float


@dataclass
class LaneSeg:
    idx: int
    direction: str        # 'forward' | 'backward'
    t_start: float
    t_end: float
    x_start: float
    x_end: float
    y_start: float
    y_end: float
    y_mean: float
    y_rms_within: float   # RMS of y deviations within the straight segment
    planned_y: float = float('nan')


@dataclass
class TurnSeg:
    idx: int
    t_start: float
    t_end: float
    y_start: float
    y_end: float
    planned_y_end: float = float('nan')


# ─── bag reading ──────────────────────────────────────────────────────────────

_WANTED_TYPES = {
    '/odometry/fusion_bno':   'nav_msgs/msg/Odometry',
    '/wheel/odom':       'nav_msgs/msg/Odometry',
    '/cmd_vel':          'geometry_msgs/msg/Twist',
    '/wheel_cmd':        'std_msgs/msg/Float32MultiArray',
    '/enc_vel':          'std_msgs/msg/Float32MultiArray',
    '/coverage/path':    'nav_msgs/msg/Path',
    '/plan':             'nav_msgs/msg/Path',
    '/local_plan':       'nav_msgs/msg/Path',
}


def read_bag(bag_path: str):
    """Open a ROS2 bag (sqlite3 or mcap) and return raw data per topic."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = Path(bag_path)
    storage_id = 'mcap' if list(bag_dir.glob('*.mcap')) else 'sqlite3'

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    active = {
        topic: type_
        for topic, type_ in _WANTED_TYPES.items()
        if topic in topic_types
    }
    msg_classes = {topic: get_message(type_) for topic, type_ in active.items()}

    data = {topic: [] for topic in active}
    bag_start_ns = None

    while reader.has_next():
        topic, raw, ts_ns = reader.read_next()
        if bag_start_ns is None:
            bag_start_ns = ts_ns
        if topic not in active:
            continue
        msg = deserialize_message(raw, msg_classes[topic])
        t = (ts_ns - bag_start_ns) / 1e9
        data[topic].append((t, msg))

    return data, active


# ─── extraction ───────────────────────────────────────────────────────────────

def extract_poses(data) -> List[PosePoint]:
    pts = []
    for t, msg in data.get('/odometry/fusion_bno', []):
        p = msg.pose.pose
        pts.append(PosePoint(
            t=t,
            x=float(p.position.x),
            y=float(p.position.y),
            yaw=yaw_from_quaternion(
                float(p.orientation.x), float(p.orientation.y),
                float(p.orientation.z), float(p.orientation.w),
            ),
        ))
    return pts


def extract_cmd_vel(data) -> List[CmdVelPoint]:
    return [
        CmdVelPoint(t=t, linear_x=float(msg.linear.x), angular_z=float(msg.angular.z))
        for t, msg in data.get('/cmd_vel', [])
    ]


def extract_wheel(data, topic: str) -> List[WheelPoint]:
    pts = []
    for t, msg in data.get(topic, []):
        v = list(msg.data)
        if len(v) >= 4:
            pts.append(WheelPoint(t=t, FL=v[0], FR=v[1], RL=v[2], RR=v[3]))
    return pts


def extract_planned_lane_ys(data) -> List[float]:
    """
    Extract planned lane y values from /coverage/path.
    Lane segments are horizontal (yaw ≈ 0 or π), so we look for poses
    with near-horizontal heading and cluster their y values.
    """
    msgs = data.get('/coverage/path', [])
    if not msgs:
        return []

    _, path = msgs[-1]   # use the last-published path

    horizontal_ys = []
    for ps in path.poses:
        q = ps.pose.orientation
        yaw = yaw_from_quaternion(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        if abs(normalize_angle(yaw)) < 0.5 or abs(normalize_angle(yaw - math.pi)) < 0.5:
            horizontal_ys.append(round(float(ps.pose.position.y), 3))

    if not horizontal_ys:
        return []

    # Cluster by proximity (within 0.1 m)
    horizontal_ys.sort()
    clusters = []
    current = [horizontal_ys[0]]
    for y in horizontal_ys[1:]:
        if y - current[-1] < 0.10:
            current.append(y)
        else:
            clusters.append(sum(current) / len(current))
            current = [y]
    clusters.append(sum(current) / len(current))

    return sorted(clusters)


# ─── phase detection ──────────────────────────────────────────────────────────

_ANG_TURN_THRESH = 0.15   # rad/s — above this is classified as turning
_MIN_STRAIGHT_S = 2.0     # s — minimum straight phase duration


def _pose_at(poses: List[PosePoint], t: float) -> Optional[PosePoint]:
    if not poses:
        return None
    if t <= poses[0].t:
        return poses[0]
    if t >= poses[-1].t:
        return poses[-1]
    lo, hi = 0, len(poses) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if poses[mid].t <= t:
            lo = mid
        else:
            hi = mid
    a, b = poses[lo], poses[hi]
    span = b.t - a.t
    r = 0.0 if span < 1e-9 else (t - a.t) / span
    return PosePoint(
        t=t,
        x=_lerp(a.x, b.x, r),
        y=_lerp(a.y, b.y, r),
        yaw=normalize_angle(a.yaw + normalize_angle(b.yaw - a.yaw) * r),
    )


def detect_phases(
    poses: List[PosePoint],
    cmd_vels: List[CmdVelPoint],
    min_straight_sec: float = _MIN_STRAIGHT_S,
) -> Tuple[List[LaneSeg], List[TurnSeg]]:
    """
    Segment the run into straight-line lanes and arc turns using cmd_vel.
    Straight = |angular_z| < threshold sustained for ≥ min_straight_sec.
    """
    if not cmd_vels or not poses:
        return [], []

    # Step 1 — raw phase sequence from cmd_vel
    raw: List[Tuple[float, float, str]] = []   # (t_start, t_end, type)
    seg_t = cmd_vels[0].t
    seg_type = 'TURN' if abs(cmd_vels[0].angular_z) > _ANG_TURN_THRESH else 'STRAIGHT'

    for i in range(1, len(cmd_vels)):
        cv = cmd_vels[i]
        ptype = 'TURN' if abs(cv.angular_z) > _ANG_TURN_THRESH else 'STRAIGHT'
        if ptype != seg_type:
            raw.append((seg_t, cv.t, seg_type))
            seg_t = cv.t
            seg_type = ptype
    raw.append((seg_t, cmd_vels[-1].t, seg_type))

    # Step 2 — merge short STRAIGHT blips into adjacent TURN
    merged: List[List] = []
    for seg in raw:
        t0, t1, st = seg
        dur = t1 - t0
        if st == 'STRAIGHT' and dur < min_straight_sec:
            # absorb short straight into surrounding turn
            if merged and merged[-1][2] == 'TURN':
                merged[-1][1] = t1
                continue
            else:
                st = 'TURN'
        if merged and merged[-1][2] == st:
            merged[-1][1] = t1
        else:
            merged.append([t0, t1, st])

    # Step 3 — build lane / turn objects
    lanes: List[LaneSeg] = []
    turns: List[TurnSeg] = []
    lane_idx = turn_idx = 0

    for t0, t1, st in merged:
        seg_poses = [p for p in poses if t0 <= p.t <= t1]
        if len(seg_poses) < 2:
            p0 = _pose_at(poses, t0)
            p1 = _pose_at(poses, t1)
            seg_poses = [p0, p1] if p0 and p1 else []
        if not seg_poses:
            continue

        if st == 'STRAIGHT':
            ys = [p.y for p in seg_poses]
            mean_y = sum(ys) / len(ys)
            rms_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys) / len(ys))
            dx = seg_poses[-1].x - seg_poses[0].x
            lanes.append(LaneSeg(
                idx=lane_idx,
                direction='forward' if dx >= 0 else 'backward',
                t_start=seg_poses[0].t,
                t_end=seg_poses[-1].t,
                x_start=seg_poses[0].x,
                x_end=seg_poses[-1].x,
                y_start=seg_poses[0].y,
                y_end=seg_poses[-1].y,
                y_mean=mean_y,
                y_rms_within=rms_y,
            ))
            for p in seg_poses:
                p.phase = f'straight_{lanes[-1].direction}'
            lane_idx += 1
        else:
            turns.append(TurnSeg(
                idx=turn_idx,
                t_start=seg_poses[0].t,
                t_end=seg_poses[-1].t,
                y_start=seg_poses[0].y,
                y_end=seg_poses[-1].y,
            ))
            for p in seg_poses:
                p.phase = 'turn'
            turn_idx += 1

    # Tag poses not in any segment as 'idle'
    for p in poses:
        if not p.phase:
            p.phase = 'idle'

    return lanes, turns


def match_planned_ys(lanes: List[LaneSeg], planned_ys: List[float]):
    if not planned_ys:
        return
    for lane in lanes:
        lane.planned_y = min(planned_ys, key=lambda py: abs(py - lane.y_mean))


def match_turn_planned_ys(turns: List[TurnSeg], lanes: List[LaneSeg]):
    for turn in turns:
        following = [l for l in lanes if l.t_start >= turn.t_end]
        if following:
            turn.planned_y_end = following[0].planned_y


# ─── CSV writers ──────────────────────────────────────────────────────────────

def write_pose_trajectory(poses: List[PosePoint], cmd_vels: List[CmdVelPoint], out_dir: str) -> str:
    cv_idx = 0
    rows = []
    for p in poses:
        while cv_idx + 1 < len(cmd_vels) and cmd_vels[cv_idx + 1].t <= p.t:
            cv_idx += 1
        cv = cmd_vels[cv_idx] if cmd_vels else None
        rows.append({
            't_sec':              fmt(p.t, 4),
            'x_m':               fmt(p.x, 4),
            'y_m':               fmt(p.y, 4),
            'yaw_rad':           fmt(p.yaw, 4),
            'yaw_deg':           fmt(math.degrees(p.yaw), 2),
            'phase':             p.phase,
            'cmd_linear_x_ms':   fmt(cv.linear_x, 4) if cv else '',
            'cmd_angular_z_rads': fmt(cv.angular_z, 4) if cv else '',
        })

    out = os.path.join(out_dir, 'pose_trajectory.csv')
    with open(out, 'w', newline='') as f:
        if rows:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
    return out


def write_cmd_vel(cmd_vels: List[CmdVelPoint], out_dir: str) -> str:
    out = os.path.join(out_dir, 'cmd_vel.csv')
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_sec', 'linear_x_ms', 'angular_z_rads'])
        for cv in cmd_vels:
            w.writerow([fmt(cv.t, 4), fmt(cv.linear_x, 4), fmt(cv.angular_z, 4)])
    return out


def write_wheel_speeds(wheel_cmd: List[WheelPoint], enc_vel: List[WheelPoint], out_dir: str) -> str:
    out = os.path.join(out_dir, 'wheel_speeds.csv')
    tagged = [(wp, 'wheel_cmd') for wp in wheel_cmd] + [(wp, 'enc_vel') for wp in enc_vel]
    tagged.sort(key=lambda x: x[0].t)
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_sec', 'source', 'FL_ms', 'FR_ms', 'RL_ms', 'RR_ms'])
        for wp, src in tagged:
            w.writerow([fmt(wp.t, 4), src,
                        fmt(wp.FL, 4), fmt(wp.FR, 4), fmt(wp.RL, 4), fmt(wp.RR, 4)])
    return out


def write_lane_tracking(lanes: List[LaneSeg], out_dir: str) -> str:
    out = os.path.join(out_dir, 'lane_tracking.csv')
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'lane_idx', 'direction',
            'planned_y_m',
            'actual_start_y_m', 'actual_end_y_m', 'actual_mean_y_m',
            'y_err_start_m', 'y_err_end_m', 'y_err_mean_m',
            'y_rms_within_lane_m',
            'start_time_sec', 'end_time_sec', 'duration_sec',
            'start_x_m', 'end_x_m',
        ])
        for lane in lanes:
            py = lane.planned_y
            has_plan = not math.isnan(py)
            w.writerow([
                lane.idx, lane.direction,
                fmt(py) if has_plan else '',
                fmt(lane.y_start), fmt(lane.y_end), fmt(lane.y_mean),
                fmt(lane.y_start - py) if has_plan else '',
                fmt(lane.y_end   - py) if has_plan else '',
                fmt(lane.y_mean  - py) if has_plan else '',
                fmt(lane.y_rms_within),
                fmt(lane.t_start, 2), fmt(lane.t_end, 2),
                fmt(lane.t_end - lane.t_start, 2),
                fmt(lane.x_start), fmt(lane.x_end),
            ])
    return out


def write_turn_tracking(turns: List[TurnSeg], out_dir: str) -> str:
    out = os.path.join(out_dir, 'turn_tracking.csv')
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'turn_idx',
            'start_time_sec', 'end_time_sec', 'duration_sec',
            'y_before_m',
            'planned_y_after_m', 'actual_y_after_m',
            'y_error_m',
        ])
        for turn in turns:
            py = turn.planned_y_end
            has_plan = not math.isnan(py)
            w.writerow([
                turn.idx,
                fmt(turn.t_start, 2), fmt(turn.t_end, 2),
                fmt(turn.t_end - turn.t_start, 2),
                fmt(turn.y_start),
                fmt(py) if has_plan else '',
                fmt(turn.y_end),
                fmt(turn.y_end - py) if has_plan else '',
            ])
    return out


def write_coverage_summary(
    lanes: List[LaneSeg],
    turns: List[TurnSeg],
    poses: List[PosePoint],
    cmd_vels: List[CmdVelPoint],
    out_dir: str,
) -> str:
    total_dur = (poses[-1].t - poses[0].t) if len(poses) >= 2 else 0.0

    y_errs = [abs(l.y_mean - l.planned_y) for l in lanes if not math.isnan(l.planned_y)]
    mean_err  = sum(y_errs) / len(y_errs) if y_errs else float('nan')
    max_err   = max(y_errs) if y_errs else float('nan')
    rms_err   = math.sqrt(sum(e ** 2 for e in y_errs) / len(y_errs)) if y_errs else float('nan')

    lane_durs = [l.t_end - l.t_start for l in lanes]
    turn_durs = [t.t_end - t.t_start for t in turns]

    lin_vels = [abs(cv.linear_x) for cv in cmd_vels if abs(cv.linear_x) > 0.01]
    mean_vel = sum(lin_vels) / len(lin_vels) if lin_vels else 0.0

    # Estimate covered area from trajectory bounding box
    if poses:
        xs = [p.x for p in poses]
        ys = [p.y for p in poses]
        area_bbox = (max(xs) - min(xs)) * (max(ys) - min(ys))
    else:
        area_bbox = 0.0

    rows = [
        ('total_duration_sec',     fmt(total_dur, 2),              's'),
        ('num_lanes_executed',     str(len(lanes)),                 '-'),
        ('num_turns_executed',     str(len(turns)),                 '-'),
        ('mean_lane_y_error_m',    fmt(mean_err),                   'm'),
        ('max_lane_y_error_m',     fmt(max_err),                    'm'),
        ('rms_lane_y_error_m',     fmt(rms_err),                    'm'),
        ('mean_lane_duration_sec', fmt(sum(lane_durs) / len(lane_durs), 2) if lane_durs else '', 's'),
        ('mean_turn_duration_sec', fmt(sum(turn_durs) / len(turn_durs), 2) if turn_durs else '', 's'),
        ('mean_linear_vel_ms',     fmt(mean_vel),                   'm/s'),
        ('trajectory_bbox_area_m2', fmt(area_bbox, 2),              'm²'),
        ('total_pose_samples',     str(len(poses)),                 '-'),
        ('total_cmdvel_samples',   str(len(cmd_vels)),              '-'),
    ]

    out = os.path.join(out_dir, 'coverage_summary.csv')
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['metric', 'value', 'unit'])
        w.writerows(rows)
    return out


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Generate CSV report tables from a coverage run bag.'
    )
    parser.add_argument('bag_path', help='Path to ROS2 bag directory')
    parser.add_argument('--out', default='',
                        help='Output directory (default: <bag_path>_report)')
    parser.add_argument('--min-lane-sec', type=float, default=_MIN_STRAIGHT_S,
                        help=f'Min duration to classify as a lane (default: {_MIN_STRAIGHT_S}s)')
    parser.add_argument('--turn-thresh', type=float, default=0.15,
                        help='|angular_z| rad/s above which a phase counts as a turn '
                             '(default 0.15; use ~0.08 for wide/gentle sand arcs, e.g. R>=1.5 m '
                             'where w = v/R is only ~0.14 rad/s)')
    args = parser.parse_args()

    global _ANG_TURN_THRESH
    _ANG_TURN_THRESH = args.turn_thresh

    bag_path = args.bag_path.rstrip('/')
    out_dir = args.out or (bag_path + '_report')
    os.makedirs(out_dir, exist_ok=True)

    print(f'Reading bag: {bag_path}')
    data, active_topics = read_bag(bag_path)
    found = list(active_topics.keys())
    print(f'Topics found: {found}')

    poses     = extract_poses(data)
    cmd_vels  = extract_cmd_vel(data)
    wheel_cmd = extract_wheel(data, '/wheel_cmd')
    enc_vel   = extract_wheel(data, '/enc_vel')
    planned_ys = extract_planned_lane_ys(data)

    print(f'Pose samples     : {len(poses)}')
    print(f'cmd_vel samples  : {len(cmd_vels)}')
    print(f'wheel_cmd samples: {len(wheel_cmd)}')
    print(f'enc_vel samples  : {len(enc_vel)}')
    print(f'Planned lane ys  : {[round(y, 3) for y in planned_ys]}')

    if not poses:
        print('ERROR: /odometry/fusion_bno not in bag — cannot analyse.', file=sys.stderr)
        sys.exit(1)
    if not cmd_vels:
        print('WARNING: /cmd_vel not in bag — phase detection will be skipped.',
              file=sys.stderr)

    lanes, turns = detect_phases(poses, cmd_vels, args.min_lane_sec)
    match_planned_ys(lanes, planned_ys)
    match_turn_planned_ys(turns, lanes)

    print(f'Lanes detected   : {len(lanes)}')
    print(f'Turns detected   : {len(turns)}')

    written = []
    written.append(write_pose_trajectory(poses, cmd_vels, out_dir))
    if cmd_vels:
        written.append(write_cmd_vel(cmd_vels, out_dir))
    if wheel_cmd or enc_vel:
        written.append(write_wheel_speeds(wheel_cmd, enc_vel, out_dir))
    if lanes:
        written.append(write_lane_tracking(lanes, out_dir))
    if turns:
        written.append(write_turn_tracking(turns, out_dir))
    written.append(write_coverage_summary(lanes, turns, poses, cmd_vels, out_dir))

    print(f'\nOutput → {out_dir}/')
    for p in written:
        print(f'  {os.path.basename(p)}')


if __name__ == '__main__':
    main()
