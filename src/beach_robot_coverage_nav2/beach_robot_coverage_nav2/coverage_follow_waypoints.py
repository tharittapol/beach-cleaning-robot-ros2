#!/usr/bin/env python3
import math
from dataclasses import dataclass
from functools import partial
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import FollowWaypoints, NavigateToPose, NavigateThroughPoses, FollowPath, Spin
from nav_msgs.msg import Odometry, Path
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rclpy.qos import DurabilityPolicy, QoSProfile

from .obstacle_detector import FrontBoxMonitor


def quaternion_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass
class RectArea:
    origin_x: float = 0.0
    origin_y: float = 0.0
    width: float = 5.0
    height: float = 3.6
    yaw: float = 0.0


class CoverageFollowWaypoints(Node):
    def __init__(self):
        super().__init__('coverage_follow_waypoints')

        self.declare_parameter('area.origin_x', 0.0)
        self.declare_parameter('area.origin_y', 0.0)
        self.declare_parameter('area.width', 5.0)
        self.declare_parameter('area.height', 3.6)
        self.declare_parameter('area.yaw', 0.0)

        self.declare_parameter('pattern', 'boustrophedon')  # boustrophedon|spiral
        self.declare_parameter('tool_width', 0.60)
        self.declare_parameter('overlap', 0.0)
        self.declare_parameter('lane_spacing', 0.60)
        self.declare_parameter('lane_center_offset', 0.30)
        self.declare_parameter('auto_widen_lanes_for_turn', False)
        self.declare_parameter('boundary_margin', 0.0)   # 0 → first lane at area origin = robot spawn pose
        self.declare_parameter('waypoint_step', 0.50)
        self.declare_parameter('prealign_at_lane_entry', False)  # Spin-to-align disabled: on this asymmetric robot the in-place spin drifts the centre ~0.3 m, so it traded heading tilt for a position offset and didn't reduce the lane error (run 190708). Kept as an opt-in.
        self.declare_parameter('prealign_min_angle_deg', 10.0)  # skip the spin below this heading error
        self.declare_parameter('turn_style', 'arc')  # arc|corner
        self.declare_parameter('turn_radius', 2.10)
        self.declare_parameter('minimum_turn_radius', 2.10)

        self.declare_parameter('action_name', 'follow_waypoints')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('preview_path_topic', '/coverage/path')
        self.declare_parameter('autostart', True)
        self.declare_parameter('start_delay_sec', 5.0)
        # Cut an arc turn early when the heading reaches the next lane (handles a tight,
        # under-following arc). Set False to instead FOLLOW THE FULL arc/teardrop path to its
        # endpoint — cleaner once turn_radius matches the robot, and it avoids the cancel+resend
        # race that can skip the lane right after a cut.
        self.declare_parameter('enable_turn_yaw_cut', True)
        self._yaw_cut_enabled = bool(self.get_parameter('enable_turn_yaw_cut').value)

        # --- multipass coverage (interleaved passes for 100% coverage) ---
        # num_passes>1: lay fine lanes at lane_spacing/num_passes (= tool_width) and run
        # them as N interleaved passes; in-pass lanes stay lane_spacing apart (arc turns),
        # the offsets fill the gaps. Between passes the robot loops outside the work area.
        self.declare_parameter('num_passes', 1)
        self.declare_parameter(
            'coverage_path_mode', 'same_direction_loops')
        self.declare_parameter('upper_return_count', 3)
        self.declare_parameter('deadhead_style', 'outside')   # outside|direct|rounded
        self.declare_parameter('deadhead_clearance', 0.9)
        # Seconds to actively hold cmd_vel=0 between the 'direct' deadhead segments (after each
        # pivot/drive) so the robot fully STOPS and the heading settles before the next goal-checked
        # Spin / the lane. 0 = no settle. (The old rounded stop-and-go chunking was removed —
        # rolling-curve reversals can't be made safe on tile; 'direct' uses in-place pivots.)
        self.declare_parameter('deadhead_settle_sec', 0.8)

        # --- auto-mode obstacle stop (ZED straight front box) ---
        self.declare_parameter('obstacle_stop.enabled', True)
        self.declare_parameter('obstacle_stop.cloud_topic', '/zed/filtered_cloud')
        self.declare_parameter('obstacle_stop.camera_forward_offset', 0.71245)
        self.declare_parameter('obstacle_stop.min_forward_distance', 0.25)
        self.declare_parameter('obstacle_stop.stop_distance', 2.0)
        self.declare_parameter('obstacle_stop.box_width', 0.8)
        self.declare_parameter('obstacle_stop.min_z', 0.12)
        self.declare_parameter('obstacle_stop.max_z', 1.5)
        self.declare_parameter('obstacle_stop.min_points', 1000)
        self.declare_parameter('obstacle_stop.clear_time_sec', 3.0)
        self.declare_parameter('obstacle_stop.beep_period_sec', 1.0)
        self.declare_parameter('obstacle_stop.beep_duration_sec', 0.4)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('buzzer_topic', 'buzzer_duration')

        self.area = RectArea(
            origin_x=float(self.get_parameter('area.origin_x').value),
            origin_y=float(self.get_parameter('area.origin_y').value),
            width=float(self.get_parameter('area.width').value),
            height=float(self.get_parameter('area.height').value),
            yaw=float(self.get_parameter('area.yaw').value),
        )

        self.pattern = str(self.get_parameter('pattern').value).lower()
        self.tool_width = max(0.01, float(self.get_parameter('tool_width').value))
        self.overlap = min(0.90, max(0.0, float(self.get_parameter('overlap').value)))
        self.lane_spacing_param = float(self.get_parameter('lane_spacing').value)
        self.lane_center_offset = max(
            0.0, float(self.get_parameter('lane_center_offset').value))
        self.auto_widen_lanes = self._as_bool(self.get_parameter('auto_widen_lanes_for_turn').value)
        self.margin = max(0.0, float(self.get_parameter('boundary_margin').value))
        self.waypoint_step = max(0.10, float(self.get_parameter('waypoint_step').value))
        self._prealign_enabled = bool(self.get_parameter('prealign_at_lane_entry').value)
        self._prealign_min_angle = math.radians(
            max(0.0, float(self.get_parameter('prealign_min_angle_deg').value)))
        self.turn_style = str(self.get_parameter('turn_style').value).lower()
        self.turn_radius = max(0.0, float(self.get_parameter('turn_radius').value))
        self.minimum_turn_radius = max(
            2.10, float(self.get_parameter('minimum_turn_radius').value))
        self.num_passes = max(1, int(self.get_parameter('num_passes').value))
        self.coverage_path_mode = str(
            self.get_parameter('coverage_path_mode').value).lower()
        if self.coverage_path_mode not in (
                'same_direction_loops', 'teardrop', 'multipass_boustrophedon'):
            self.get_logger().warn(
                f'Unknown coverage_path_mode={self.coverage_path_mode!r}; '
                'using same_direction_loops.')
            self.coverage_path_mode = 'same_direction_loops'
        if (
            self.coverage_path_mode == 'same_direction_loops'
            and self.turn_radius < self.minimum_turn_radius
        ):
            self.get_logger().warn(
                f'Increasing turn_radius {self.turn_radius:.2f}→'
                f'{self.minimum_turn_radius:.2f}m for comfortable return loops.')
            self.turn_radius = self.minimum_turn_radius
        self.upper_return_count = max(
            0, int(self.get_parameter('upper_return_count').value))
        self.deadhead_style = str(self.get_parameter('deadhead_style').value).lower()
        self.deadhead_clearance = max(
            float(self.get_parameter('deadhead_clearance').value), self.turn_radius)
        self.deadhead_settle_sec = max(0.0, float(self.get_parameter('deadhead_settle_sec').value))
        self.lane_spacing = self._effective_lane_spacing()

        self.action_name = str(self.get_parameter('action_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.preview_path_topic = str(self.get_parameter('preview_path_topic').value)

        self.obstacle_enabled = self._as_bool(self.get_parameter('obstacle_stop.enabled').value)
        self.obstacle_cloud_topic = str(self.get_parameter('obstacle_stop.cloud_topic').value)
        self.obstacle_camera_forward_offset = max(
            0.0, float(self.get_parameter('obstacle_stop.camera_forward_offset').value))
        self.obstacle_min_forward_distance = float(
            self.get_parameter('obstacle_stop.min_forward_distance').value)
        self.obstacle_stop_distance = float(self.get_parameter('obstacle_stop.stop_distance').value)
        self.obstacle_box_width = float(self.get_parameter('obstacle_stop.box_width').value)
        self.obstacle_min_z = float(self.get_parameter('obstacle_stop.min_z').value)
        self.obstacle_max_z = float(self.get_parameter('obstacle_stop.max_z').value)
        self.obstacle_min_points = int(self.get_parameter('obstacle_stop.min_points').value)
        self.obstacle_clear_time_sec = float(self.get_parameter('obstacle_stop.clear_time_sec').value)
        self.beep_period_sec = max(0.2, float(self.get_parameter('obstacle_stop.beep_period_sec').value))
        self.beep_duration_sec = float(self.get_parameter('obstacle_stop.beep_duration_sec').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.buzzer_topic = str(self.get_parameter('buzzer_topic').value)

        # Pre-compute geometry (used by both preview and nav)
        self._x0 = self.margin
        self._x1 = max(self.margin, self.area.width - self.margin)
        same_dir = (self.coverage_path_mode == 'same_direction_loops')
        if same_dir:
            self._y0 = self.lane_center_offset
            self._y1 = max(self._y0, self.area.height - self.lane_center_offset)
        else:
            # teardrop / multipass: lay lanes across the FULL area [margin, height-margin].
            # The scoop is allowed to overhang the top/bottom edges (that is fine and is what
            # makes coverage 100%). The first executed lane sits at y=margin, so the robot
            # is spawned on that line (spawn_y = area_origin_y + margin) and drives straight
            # immediately — no opening turn down onto a lower lane.
            self._y0 = self.margin
            self._y1 = max(self._y0, self.area.height - self.margin)
        # _lane_ys is the lane y's in execution order; _lane_pass[i] is which pass lane i
        # belongs to (a change in _lane_pass between consecutive lanes ⇒ a between-pass
        # deadhead instead of an in-pass arc turn).
        #
        # same_direction keeps its historic _lane_values layout; teardrop/multipass use
        # _even_lane_values so every fine lane is EQUALLY spaced (no uneven last gap) and the
        # spacing is capped at tool_width → neighbouring tool strips always touch → 100%.
        lane_gen = self._lane_values if same_dir else self._even_lane_values
        if self.num_passes > 1:
            fine = lane_gen(self._y0, self._y1, self.lane_spacing / self.num_passes)
            self._lane_ys = []
            self._lane_pass = []
            for p in range(self.num_passes):
                for y in fine[p::self.num_passes]:
                    self._lane_ys.append(y)
                    self._lane_pass.append(p)
        else:
            self._lane_ys = lane_gen(self._y0, self._y1, self.lane_spacing)
            self._lane_pass = [0] * len(self._lane_ys)

        # Report the achieved fine spacing (even modes guarantee equal spacing + full cover).
        if len(self._lane_ys) > 1 and not same_dir:
            span = self._y1 - self._y0
            n_fine = len(set(round(y, 4) for y in self._lane_ys)) - 1
            fine_sp = span / n_fine if n_fine > 0 else span
            covers = 'overlap' if fine_sp < self.tool_width - 1e-6 else (
                'touching' if abs(fine_sp - self.tool_width) <= 1e-6 else 'GAPS')
            self.get_logger().info(
                f'even lanes: {len(self._lane_ys)} lanes, fine spacing={fine_sp:.3f}m '
                f'(tool_width={self.tool_width:.2f}m → {covers}), '
                f'first lane y={self._lane_ys[0]:.3f} span y=[{self._y0:.2f},{self._y1:.2f}]')

        # Action clients
        self._follow_ac = ActionClient(self, FollowWaypoints, self.action_name)
        self._nav_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._through_ac = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        # Lanes go through FollowPath (controller follows the EXACT lane line, no NavFn reshaping).
        self._follow_path_ac = ActionClient(self, FollowPath, 'follow_path')
        # One-shot in-place align (Spin behavior) to square the heading at each lane entry.
        self._spin_ac = ActionClient(self, Spin, 'spin')

        # Robot pose from /odometry/fusion_bno = the EKF (wheel+BNO) output the stack actually
        # publishes (/odometry/local is NOT produced here). self._odom feeds the yaw-cut turn
        # completion, actual-y after a turn, and obstacle resume — all dead if this is empty.
        self._odom: Optional[Odometry] = None
        self.create_subscription(Odometry, '/odometry/fusion_bno', self._odom_cb, 10)

        # Preview publishers
        preview_qos = QoSProfile(depth=1)
        preview_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, self.preview_path_topic, preview_qos)
        self.path_pub_viz = self.create_publisher(Path, self.preview_path_topic + '_viz', 10)
        self._preview_poses: Optional[List[PoseStamped]] = None
        # Area-boundary outline (cyan rectangle of the defined coverage area, in map frame).
        # Deliberately NOT green — green is the coverage path colour. Latched so RViz catches it.
        self.area_marker_pub = self.create_publisher(
            Marker, '/coverage/area_boundary', preview_qos)

        # Navigation state
        self._lane_idx = 0
        self._forward = True
        self._current_y: float = self._lane_ys[0] if self._lane_ys else 0.0
        self._started = False
        self._turn_y_target: float = 0.0
        # Turn yaw monitoring (cancel arc goal when yaw reaches next-lane heading)
        self._in_turn: bool = False
        self._turn_triggered: bool = False
        self._turn_target_yaw: float = 0.0
        self._turn_start_time = None
        self._turn_yaw_direction: float = 1.0
        self._turn_last_yaw: Optional[float] = None
        self._turn_yaw_progress: float = 0.0
        self._turn_required_yaw_delta: Optional[float] = None
        self._turn_goal_handle = None
        self._turn_is_deadhead: bool = False   # the yaw-cut handler routes to _after_deadhead
        self._lane_goal_handle = None
        # Between-pass deadhead bookkeeping
        self._deadhead_next_y: float = 0.0
        self._deadhead_new_forward: bool = True
        # 'direct' deadhead segment runner: a list of ('pivot', yaw_map) / ('drive', a, b)
        # steps executed one at a time (pivot = goal-checked Spin, drive = FollowPath line).
        self._dh_segments: List = []
        self._dh_index: int = 0
        self._dh_settle_timer = None
        self._dh_settle_ticks: int = 0

        # Obstacle-stop state.
        # _nav_epoch tags every goal we send; a result whose epoch != current is stale
        # (its goal was cancelled for an obstacle) and must not advance the state machine.
        self._phase = 'idle'            # 'idle' | 'lane' | 'turn' | 'deadhead'
        self._nav_epoch = 0
        self._paused = False
        self._obstacle_present = False
        self._clear_start = None

        # Obstacle-stop IO (cmd_vel override + buzzer + ZED straight front-box monitor)
        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._buzzer_pub = self.create_publisher(Float32, self.buzzer_topic, 10)
        if self.obstacle_enabled:
            self._monitor = FrontBoxMonitor(
                self, self._on_obstacle_update,
                cloud_topic=self.obstacle_cloud_topic,
                min_forward_distance=(
                    self.obstacle_camera_forward_offset
                    + self.obstacle_min_forward_distance
                ),
                stop_distance=(
                    self.obstacle_camera_forward_offset
                    + self.obstacle_stop_distance
                ),
                box_width=self.obstacle_box_width,
                min_z=self.obstacle_min_z,
                max_z=self.obstacle_max_z,
                min_points=self.obstacle_min_points,
            )
            # Always-on timers; they only act while _paused.
            self.create_timer(1.0 / 20.0, self._override_cmd_cb)
            self.create_timer(self.beep_period_sec, self._beep_cb)
            self.get_logger().info(
                f'Obstacle-stop enabled: camera_x={self.obstacle_min_forward_distance:.2f}..'
                f'{self.obstacle_stop_distance:.2f}m '
                f'camera_offset={self.obstacle_camera_forward_offset:.5f}m '
                f'box_width={self.obstacle_box_width:.2f}m resume after '
                f'{self.obstacle_clear_time_sec:.0f}s clear')

        if self._as_bool(self.get_parameter('autostart').value):
            delay = float(self.get_parameter('start_delay_sec').value)
            self.create_timer(delay, self._start_once)
        else:
            self._started = True
            self.publish_preview()

        # Republish viz path every 3 s for late-joining RViz2
        self.create_timer(3.0, self._republish_viz)

        self.get_logger().info(
            f'Coverage planner: pattern={self.pattern} '
            f'area={self.area.width:.2f}×{self.area.height:.2f}m '
            f'passes={self.num_passes} lanes={len(self._lane_ys)} '
            f'path_mode={self.coverage_path_mode} '
            f'in-pass spacing={self.lane_spacing:.2f}m '
            f'turn_radius={self.turn_radius:.2f}m '
            f'deadhead={self.deadhead_style}'
        )

    # ---- helpers ----

    def _as_bool(self, value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    @staticmethod
    def _norm_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @classmethod
    def _angle_diff(cls, to_angle: float, from_angle: float) -> float:
        return cls._norm_angle(to_angle - from_angle)

    @classmethod
    def _directed_angle_delta(cls, from_angle: float, to_angle: float, direction: float) -> float:
        delta = cls._angle_diff(to_angle, from_angle)
        if direction >= 0.0:
            if delta < 0.0:
                delta += 2.0 * math.pi
        elif delta > 0.0:
            delta -= 2.0 * math.pi
        return delta

    @staticmethod
    def _yaw_from_odom(msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _effective_lane_spacing(self) -> float:
        spacing = self.lane_spacing_param
        if spacing <= 0.0:
            spacing = self.tool_width * (1.0 - self.overlap)
        spacing = max(0.05, spacing)
        min_arc = 2.0 * self.turn_radius
        if self.turn_style == 'arc' and self.turn_radius > 0.0:
            if self.coverage_path_mode == 'same_direction_loops':
                return spacing
            if self.auto_widen_lanes and spacing < min_arc:
                self.get_logger().warn(
                    f'Widening lane_spacing {spacing:.2f}→{min_arc:.2f}m (2×turn_radius).')
                spacing = min_arc
            elif spacing < min_arc:
                self.get_logger().warn(
                    f'lane_spacing={spacing:.2f}m < 2×turn_radius={min_arc:.2f}m. '
                    'Turns may become skid turns. Add auto_widen_lanes_for_turn:=true if needed.')
        return spacing

    # Yaw error within this threshold (rad) -> arc done, start next lane immediately
    _TURN_YAW_THRESHOLD = 0.25   # ≈ 14° — triggers at ~95% of arc completion

    def _odom_cb(self, msg: Odometry):
        self._odom = msg
        if (self._yaw_cut_enabled and self._in_turn
                and not self._turn_triggered and not self._paused):
            self._check_turn_yaw(msg)

    def _check_turn_yaw(self, msg: Odometry):
        if self._turn_start_time is None or self._paused:
            return
        yaw = self._yaw_from_odom(msg)
        if self._turn_last_yaw is None:
            self._turn_last_yaw = yaw
            self._turn_yaw_progress = 0.0
            self._turn_required_yaw_delta = self._directed_angle_delta(
                yaw, self._turn_target_yaw, self._turn_yaw_direction)
        else:
            self._turn_yaw_progress += self._angle_diff(yaw, self._turn_last_yaw)
            self._turn_last_yaw = yaw

        elapsed = (self.get_clock().now() - self._turn_start_time).nanoseconds / 1e9
        if elapsed < 1.0:   # ignore first 1 s — let the arc begin
            return

        err = abs(self._angle_diff(yaw, self._turn_target_yaw))
        overshot = self._turn_has_overshot_yaw_target()

        if err < self._TURN_YAW_THRESHOLD or overshot:
            self._turn_triggered = True
            self._in_turn = False
            reason = 'overshoot' if overshot and err >= self._TURN_YAW_THRESHOLD else 'yaw'
            required = self._turn_required_yaw_delta
            required_deg = math.degrees(required) if required is not None else float('nan')
            self.get_logger().info(
                f'Arc done ({reason}): yaw={math.degrees(yaw):.1f}°  '
                f'target={math.degrees(self._turn_target_yaw):.1f}°  '
                f'err={math.degrees(err):.1f}°  '
                f'progress={math.degrees(self._turn_yaw_progress):.1f}°/'
                f'{required_deg:.1f}°  → starting next lane')
            if self._turn_goal_handle is not None:
                self._turn_goal_handle.cancel_goal_async()
                self._turn_goal_handle = None
            if self._turn_is_deadhead:
                self._after_deadhead()
            else:
                self._after_turn()

    def _turn_has_overshot_yaw_target(self) -> bool:
        required = self._turn_required_yaw_delta
        if required is None:
            return False
        if required > 0.0:
            return self._turn_yaw_progress >= required
        if required < 0.0:
            return self._turn_yaw_progress <= required
        return False

    # ---- geometry ----

    def _rot(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        c = math.cos(yaw); s = math.sin(yaw)
        return (c*x - s*y, s*x + c*y)

    def _to_map(self, lx: float, ly: float) -> Tuple[float, float]:
        rx, ry = self._rot(lx, ly, self.area.yaw)
        return (self.area.origin_x + rx, self.area.origin_y + ry)

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        q = quaternion_from_yaw(yaw)
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return ps

    def _sample_line(self, p0, p1, step):
        x0, y0 = p0; x1, y1 = p1
        dx, dy = x1-x0, y1-y0
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return [(x0, y0)]
        n = max(1, int(L / max(step, 0.1)))
        pts = [(x0 + dx*(k/n), y0 + dy*(k/n)) for k in range(n)]
        pts.append((x1, y1))
        return pts

    def _lane_values(self, start, stop, spacing):
        vals = []
        y = start
        while y <= stop + 1e-6:
            vals.append(y)
            y += spacing
        if vals and (stop - vals[-1]) > (0.5 * spacing):
            vals.append(stop)
        if not vals:
            vals.append(start)
        return vals

    def _even_lane_values(self, start, stop, target_spacing):
        """Lanes from start..stop INCLUSIVE, all equally spaced (no uneven last gap).

        The actual spacing is span/n where n is chosen so it (a) tracks target_spacing and
        (b) never exceeds tool_width — so adjacent tool strips always touch/overlap and the
        band [start − tool/2, stop + tool/2] is fully covered → 100% coverage of the area
        (the scoop is allowed to overhang the start/stop edges)."""
        span = stop - start
        if span <= 1e-6:
            return [start]
        ts = target_spacing if target_spacing > 1e-6 else span
        n = max(1,
                int(round(span / ts)),
                int(math.ceil(span / self.tool_width - 1e-6)))
        s = span / n
        return [start + i * s for i in range(n + 1)]

    def _append_line_points(self, out, p0, p1, yaw):
        for idx, (lx, ly) in enumerate(self._sample_line(p0, p1, self.waypoint_step)):
            if out and idx == 0 and self._same_xy(out[-1], (lx, ly)):
                continue
            out.append((lx, ly, yaw))

    def _same_xy(self, point, xy, eps=1e-6):
        return abs(point[0] - xy[0]) < eps and abs(point[1] - xy[1]) < eps

    def _turn_arc(self, x_end, y_cur, y_next, forward):
        """Arc curve from lane end to next-lane start — alternates direction (S-shape)."""
        dy = y_next - y_cur
        r = abs(dy) / 2.0 if abs(dy) > 1e-6 else 0.0
        if r <= 0.05:
            yaw = 0.0 if forward else math.pi
            return [(x_end, y_next, yaw)]

        # forward: arc bulges right (+x beyond x_end); backward: bulges left (-x)
        sign = 1.0 if forward else -1.0
        cy = (y_cur + y_next) / 2.0
        steps = max(8, int(math.ceil(math.pi * r / max(self.waypoint_step, 0.1))))
        theta0 = -math.pi / 2.0 if dy >= 0.0 else math.pi / 2.0
        theta1 = math.pi / 2.0 if dy >= 0.0 else -math.pi / 2.0
        pts = []
        for i in range(1, steps + 1):
            th = theta0 + (theta1 - theta0) * (i / steps)
            lx = x_end + sign * r * math.cos(th)
            ly = cy + r * math.sin(th)
            yaw = th + math.pi / 2.0 if forward else math.pi / 2.0 - th
            pts.append((lx, ly, yaw))
        return pts

    def _round_corners(self, corners, r, step):
        """Replace each vertex of a polyline with a tangent arc (radius ≤ r) so the
        deadhead is a smooth curve, never a hard 90° corner. Radius is clamped to what the
        adjacent legs allow (end legs use their full length, shared legs half), so a tight
        inter-pass gap degrades to a smaller arc instead of a sharp corner. → [(x,y,yaw),...]."""
        pts = []
        n = len(corners)
        if n == 0:
            return pts
        if n <= 2:
            a, b = corners[0], corners[-1]
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            return [(x, y, yaw) for (x, y) in self._sample_line(a, b, step)]

        start = corners[0]            # start of the current straight run (moves to each arc exit)
        for i in range(1, n - 1):
            v, nxt = corners[i], corners[i + 1]
            ix, iy = v[0] - start[0], v[1] - start[1]
            lin = math.hypot(ix, iy)
            ox, oy = nxt[0] - v[0], nxt[1] - v[1]
            lout = math.hypot(ox, oy)
            if lin < 1e-6 or lout < 1e-6:
                continue
            ix, iy = ix / lin, iy / lin
            ox, oy = ox / lout, oy / lout
            phi = math.atan2(ix * oy - iy * ox, ix * ox + iy * oy)   # signed heading change
            yaw_in = math.atan2(iy, ix)
            if abs(phi) < 1e-3:
                continue                                            # collinear → no corner
            tan_h = math.tan(abs(phi) / 2.0)
            out_avail = lout if (i + 1 == n - 1) else 0.5 * lout
            rr = min(r, lin / tan_h, out_avail / tan_h) if tan_h > 1e-6 else 0.0
            t = rr * tan_h
            if rr < 0.05 or t < 1e-3:                               # too tight to fillet
                for (x, y) in self._sample_line(start, v, step):
                    pts.append((x, y, yaw_in))
                start = v
                continue
            t_in = (v[0] - ix * t, v[1] - iy * t)
            t_out = (v[0] + ox * t, v[1] + oy * t)
            for (x, y) in self._sample_line(start, t_in, step):
                pts.append((x, y, yaw_in))
            sgn = 1.0 if phi > 0 else -1.0                          # +1 = left turn
            cx, cy = t_in[0] - iy * sgn * rr, t_in[1] + ix * sgn * rr
            a0 = math.atan2(t_in[1] - cy, t_in[0] - cx)
            asteps = max(3, int(math.ceil(rr * abs(phi) / max(step, 0.1))))
            for k in range(1, asteps + 1):
                ang = a0 + phi * (k / asteps)
                pts.append((cx + rr * math.cos(ang), cy + rr * math.sin(ang),
                            ang + sgn * math.pi / 2.0))
            start = t_out
        last = corners[-1]
        yaw_f = math.atan2(last[1] - start[1], last[0] - start[0])
        for (x, y) in self._sample_line(start, last, step):
            pts.append((x, y, yaw_f))
        return pts

    # ---- preview path (S-shape with arc curves) ----

    def generate_coverage_poses(self) -> List[PoseStamped]:
        if self.pattern in ('boustrophedon', 'lawnmower', 'zigzag'):
            if self.coverage_path_mode == 'same_direction_loops':
                return self._same_direction_preview(self._lane_ys)
            return self._boustrophedon_preview(self._lane_ys)
        if self.pattern in ('spiral', 'inward_spiral', 'outer_to_inner'):
            return self.generate_spiral_poses()
        self.get_logger().warn(f'Unknown pattern={self.pattern!r}; using boustrophedon.')
        return self._boustrophedon_preview(self._lane_ys)

    def _same_direction_preview(self, lane_ys: List[float]) -> List[PoseStamped]:
        """All lanes run +X; return loops enter every next lane at x=0 heading +X."""
        poses: List[PoseStamped] = []
        for lane_idx, lane_y in enumerate(lane_ys):
            for lx, ly in self._sample_line(
                    (self._x0, lane_y), (self._x1, lane_y), self.waypoint_step):
                mx, my = self._to_map(lx, ly)
                poses.append(self._pose(mx, my, self.area.yaw))

            if lane_idx >= len(lane_ys) - 1:
                break

            loop = self._same_direction_return_loop(
                lane_y, lane_ys[lane_idx + 1], lane_idx)
            for lx, ly, lyaw in loop:
                mx, my = self._to_map(lx, ly)
                poses.append(self._pose(mx, my, self.area.yaw + lyaw))
        return poses

    def _boustrophedon_preview(self, lane_ys: List[float]) -> List[PoseStamped]:
        """Full route preview: lanes + in-pass arc turns + between-pass deadhead loops.

        Mirrors the navigation dispatch (same nearest-side rule), so /coverage/path[_viz]
        shows the true planned route. With num_passes==1 every transition is an arc, i.e.
        the original single-pass S-shape."""
        poses: List[PoseStamped] = []
        forward = True
        for i, ly in enumerate(lane_ys):
            xs, xe = self._lane_x_bounds(i, forward)
            p0, p1 = (xs, ly), (xe, ly)
            yaw_lane = 0.0 if forward else math.pi

            for lx, ly2 in self._sample_line(p0, p1, self.waypoint_step):
                mx, my = self._to_map(lx, ly2)
                poses.append(self._pose(mx, my, self.area.yaw + yaw_lane))

            if i >= len(lane_ys) - 1:
                break

            end_x = p1[0]
            next_y = lane_ys[i + 1]
            same_pass = self._lane_pass[i + 1] == self._lane_pass[i]
            if same_pass:
                # in-pass arc turn (S-shape)
                for lx, lya, lyaw in self._turn_arc(end_x, ly, next_y, forward):
                    mx, my = self._to_map(lx, lya)
                    poses.append(self._pose(mx, my, self.area.yaw + lyaw))
                forward = not forward
            else:
                # Between-pass transition: local teardrop on the same end, or a rounded
                # perimeter route that enters the next pass from the opposite end.
                _rail_x, new_forward = self._deadhead_start(end_x)
                start_x = self._lane_x_bounds(i + 1, new_forward)[0]
                for lx, lya, lyaw in self._deadhead_path((end_x, ly), (start_x, next_y)):
                    mx, my = self._to_map(lx, lya)
                    poses.append(self._pose(mx, my, self.area.yaw + lyaw))
                forward = new_forward
        return poses

    def publish_preview(self, poses=None):
        if poses is None:
            poses = self.generate_coverage_poses()
        self._preview_poses = poses
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        self.path_pub.publish(path)
        self.path_pub_viz.publish(path)
        self.publish_area_boundary()
        self.get_logger().info(
            f'Preview path published ({len(poses)} poses, mode={self.coverage_path_mode})')

    def _area_boundary_marker(self) -> Marker:
        """Cyan LINE_STRIP outlining the defined coverage rectangle (map frame).

        Corners are the area-local (0,0)→(w,0)→(w,h)→(0,h) box mapped to map via
        self._to_map (honours area origin + yaw). NOT green — green is the path colour.
        """
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'coverage_area'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color = ColorRGBA(r=0.0, g=0.7, b=1.0, a=1.0)  # cyan
        m.pose.orientation.w = 1.0
        corners = [(0.0, 0.0), (self.area.width, 0.0),
                   (self.area.width, self.area.height), (0.0, self.area.height),
                   (0.0, 0.0)]  # closed loop
        for lx, ly in corners:
            mx, my = self._to_map(lx, ly)
            m.points.append(Point(x=float(mx), y=float(my), z=0.0))
        return m

    def publish_area_boundary(self):
        self.area_marker_pub.publish(self._area_boundary_marker())

    def _republish_viz(self):
        if self._preview_poses is not None:
            path = Path()
            path.header.frame_id = self.frame_id
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = self._preview_poses
            self.path_pub_viz.publish(path)
        self.publish_area_boundary()

    # ---- navigation: boustrophedon state machine ----
    #
    # Flow per lane:
    #   _run_lane       →  NavigateThroughPoses (continuous, full lane incl. endpoint p1 = arc start)
    #   _on_lane_done   →  _run_turn (NavigateThroughPoses with arc waypoints)
    #   _run_turn       →  starts _in_turn=True, monitors yaw via _check_turn_yaw
    #
    # Arc completes via whichever fires first:
    #   (a) _check_turn_yaw: yaw reaches/overshoots next-lane heading → cancel goal,
    #       call _after_turn
    #   (b) _on_turn_done: NavigateThroughPoses reaches arc endpoint normally
    #
    # _after_turn: reads actual robot y from /odometry/local → _current_y → _run_lane
    # Arc radius = (y_target - y_cur) / 2 (adapts when lane y drifts).

    def _start_once(self):
        if self._started:
            return
        self._started = True

        self.publish_preview()

        if self.pattern not in ('boustrophedon', 'lawnmower', 'zigzag'):
            self._start_static()
            return

        self.get_logger().info('Waiting for Nav2 action servers...')
        if not self._follow_ac.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('FollowWaypoints action server not available.')
            return
        if not self._through_ac.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('NavigateThroughPoses action server not available.')
            return
        if not self._follow_path_ac.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('FollowPath action server not available.')
            return

        self._lane_idx = 0
        self._forward = True
        self._current_y = self._lane_ys[0]
        self.get_logger().info(
            f'Starting coverage: {len(self._lane_ys)} lanes, '
            f'mode={self.coverage_path_mode}')
        self._run_lane()

    def _start_static(self):
        """Fallback for spiral / unknown patterns: send all waypoints at once."""
        self.get_logger().info('Waiting for Nav2 FollowWaypoints...')
        if not self._follow_ac.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('FollowWaypoints action server not available.')
            return
        poses = self.generate_coverage_poses()
        self.get_logger().info(f'Sending {len(poses)} {self.pattern} waypoints...')
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self._follow_ac.send_goal_async(goal)

    def _lane_x_bounds(self, idx, forward):
        """(x_start, x_end) for lane ``idx``. When deadhead_style=='rounded', the lane end
        that abuts a between-pass deadhead is pulled in by turn_radius so the rounded
        deadhead arc starts/ends before the rail and never bulges outside the work area
        (the lane corner is traded for the arc). In-pass (arc-turn) ends and every other
        deadhead style keep the full rail, so behaviour is unchanged unless 'rounded'."""
        inset = self.turn_radius if self.deadhead_style == 'rounded' else 0.0
        n = len(self._lane_ys)
        leading = inset and idx > 0 and self._lane_pass[idx] != self._lane_pass[idx - 1]
        trailing = inset and idx < n - 1 and self._lane_pass[idx + 1] != self._lane_pass[idx]
        s_in = inset if leading else 0.0
        e_in = inset if trailing else 0.0
        if forward:
            return self._x0 + s_in, self._x1 - e_in
        return self._x1 - s_in, self._x0 + e_in

    def _run_lane(self, start_x=None, start_y=None):
        """Drive the current lane. ``start_x``/``start_y`` override the lane start (the robot's
        actual pose after a turn / on obstacle resume).

        GENTLE MERGE (no pivots): when ``start_y`` is given and differs from the planned grid y,
        the lane is built as a shallow diagonal from (start_x, start_y) that CONVERGES to the
        planned y at the far end — not a horizontal line at the planned y. A rolling RPP turn on
        tile overshoots the next lane (rear slip → bigger radius), so a horizontal lane at the
        planned y presents a hard cross-track step the controller corrects with a big sustained
        w → the robot over-rotates and loops (runs 182318/184958). Approaching the grid along a
        ~9° diagonal needs only small steering, so the robot 'drives straight and nudges onto the
        lane' (the requested behaviour) instead of spinning."""
        self._phase = 'lane'
        y = self._current_y

        default_start, x_end = self._lane_x_bounds(self._lane_idx, self._forward)
        x_start = default_start if start_x is None else start_x
        yaw_lane = 0.0 if self._forward else math.pi
        y_begin = y if start_y is None else start_y
        p0, p1 = (x_start, y_begin), (x_end, y)
        if abs(y_begin - y) > 1e-3:
            self.get_logger().info(
                f'  merge: lane starts at actual y={y_begin:.3f} → converges to grid '
                f'y={y:.3f} over x[{x_start:.2f}→{x_end:.2f}] '
                f'(slope {math.degrees(math.atan2(y - y_begin, x_end - x_start)):+.1f}°)')

        pts = self._sample_line(p0, p1, self.waypoint_step)

        poses = [
            self._pose(*self._to_map(lx, ly), self.area.yaw + yaw_lane)
            for lx, ly in pts
        ]
        arrow = '→' if self._forward else '←'
        resumed = '' if start_x is None else ' (resumed)'
        self.get_logger().info(
            f'Lane {self._lane_idx + 1}/{len(self._lane_ys)} '
            f'y={y:.3f} {arrow}  {len(poses)} waypoints{resumed}')

        self._nav_epoch += 1
        epoch = self._nav_epoch
        # Square the heading at the lane start before driving. A tight in-pass turn leaves the robot
        # ~20 deg tilted (tile under-rotation); without this the FollowPath controller over-corrects
        # and the robot drifts ~0.3 m off the lane the whole way (run 175516). This is a one-shot
        # Spin at the NODE level — NOT global use_rotate_to_heading, which also fires mid in-pass arc
        # and stalls the turns into a pivot-fest (run 184209). Skipped if already aligned / no odom.
        target_yaw = self._norm_angle(self.area.yaw + yaw_lane)
        if (self._prealign_enabled and self._odom is not None
                and self._spin_ac.server_is_ready()):
            delta = self._angle_diff(target_yaw, self._yaw_from_odom(self._odom))
            if abs(delta) >= self._prealign_min_angle:
                self.get_logger().info(
                    f'Pre-align: spin {math.degrees(delta):+.0f}° to lane heading before driving.')
                spin_goal = Spin.Goal()
                spin_goal.target_yaw = float(delta)
                spin_goal.time_allowance = Duration(sec=15)
                fut = self._spin_ac.send_goal_async(spin_goal)
                fut.add_done_callback(
                    partial(self._on_prealign_accepted, epoch=epoch, poses=poses))
                return
        self._send_lane_path(poses, epoch)

    def _send_lane_path(self, poses, epoch):
        """FollowPath: the controller (RPP) tracks the EXACT lane line, with NO global planner in the
        loop (NavFn used to reshape robot->lane-end into a ~0.3 m offset; see run 165847). Obstacle
        avoidance still works via the local costmap inside the controller."""
        if epoch != self._nav_epoch:
            return
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        goal = FollowPath.Goal()
        goal.path = path
        future = self._follow_path_ac.send_goal_async(goal)
        future.add_done_callback(partial(self._on_lane_accepted, epoch=epoch))

    def _on_prealign_accepted(self, future, epoch, poses):
        handle = future.result()
        if epoch != self._nav_epoch:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().warn('Pre-align Spin rejected; driving lane without it.')
            self._send_lane_path(poses, epoch)
            return
        handle.get_result_async().add_done_callback(
            partial(self._on_prealign_done, epoch=epoch, poses=poses))

    def _on_prealign_done(self, future, epoch, poses):
        # Proceed to the lane whether or not the spin fully reached target — FollowPath finishes any
        # small residual alignment. Skip only if paused/superseded.
        if epoch != self._nav_epoch or self._paused:
            return
        self._send_lane_path(poses, epoch)

    def _on_lane_accepted(self, future, epoch):
        handle = future.result()
        if epoch != self._nav_epoch:
            # Paused/superseded before this goal was accepted: cancel the orphan so the
            # controller stops pursuing it. A dropped ClientGoalHandle is NOT auto-cancelled
            # — without this the robot would keep driving against the stop override.
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().error('Lane goal rejected by FollowPath server.')
            self._phase = 'idle'
            return
        self._lane_goal_handle = handle
        handle.get_result_async().add_done_callback(
            partial(self._on_lane_done, epoch=epoch))

    def _on_lane_done(self, future, epoch):
        if epoch != self._nav_epoch or self._paused:
            return  # stale result from a cancelled goal — do not advance
        self._lane_goal_handle = None
        if not self._goal_succeeded(future, 'Lane'):
            return
        self._lane_idx += 1
        if self._lane_idx >= len(self._lane_ys):
            self.get_logger().info('Coverage complete!')
            self._phase = 'idle'
            return
        if self.coverage_path_mode == 'same_direction_loops':
            self._run_deadhead()
            return
        # Same pass → in-pass turn; new pass → between-pass deadhead. With deadhead_style=='direct'
        # BOTH go through the pivot maneuver (_run_deadhead → pivot→straight→pivot). EVERY lane
        # transition is a ~180° heading reversal, and on tile a rolling RPP arc can't do that without
        # over-rotating into a loop: it spiralled the between-pass deadhead (runs 135832/180308/182318)
        # AND looped an in-pass arc turn (run 184958: 0.4→1.6 ran a near-full circle out to x=5.8,
        # y=2.3 and drove the robot out of the area). Goal-checked Spins physically can't over-rotate,
        # so 'direct' uses them for every reversal. Other styles keep the smooth in-pass arc.
        same_pass = self._lane_pass[self._lane_idx] == self._lane_pass[self._lane_idx - 1]
        if same_pass and self.deadhead_style != 'direct':
            self._run_turn()
        else:
            self._run_deadhead()

    def _run_turn(self):
        """Navigate arc waypoints via NavigateThroughPoses.

        Arc completes via ONE of two paths (whichever fires first):
          (a) _check_turn_yaw: yaw reaches/overshoots next-lane heading → cancel goal,
              start lane
          (b) _on_turn_done:  NavigateThroughPoses reaches arc endpoint normally
        """
        self._phase = 'turn'
        y_cur = self._current_y
        y_target = self._lane_ys[self._lane_idx]
        x_end = self._x1 if self._forward else self._x0

        self._turn_y_target = y_target
        start_yaw_local = 0.0 if self._forward else math.pi
        target_yaw_local = math.pi if self._forward else 0.0
        planned_start_yaw = self._norm_angle(self.area.yaw + start_yaw_local)
        self._turn_target_yaw = self._norm_angle(self.area.yaw + target_yaw_local)
        self._turn_yaw_direction = 1.0 if self._forward else -1.0
        start_yaw = self._yaw_from_odom(self._odom) if self._odom is not None else planned_start_yaw
        self._turn_last_yaw = start_yaw
        self._turn_yaw_progress = 0.0
        self._turn_required_yaw_delta = self._directed_angle_delta(
            start_yaw, self._turn_target_yaw, self._turn_yaw_direction)
        self._in_turn = True
        self._turn_triggered = False
        self._turn_is_deadhead = False
        self._turn_start_time = self.get_clock().now()
        self._turn_goal_handle = None

        arc_pts = self._turn_arc(x_end, y_cur, y_target, self._forward)
        poses = [
            self._pose(*self._to_map(lx, ly), self.area.yaw + lyaw)
            for lx, ly, lyaw in arc_pts
        ]

        # FollowPath (not NavigateThroughPoses): the controller tracks the EXACT arc, with no NavFn
        # reshaping. NavFn used to over-command rotation past the lane tangent at the arc exit, so the
        # robot over-rotated and overshot the next lane by ~0.3 m (run 192647). Following the exact arc
        # whose end is tangent to the lane should let RPP stop rotating when aligned. The yaw-cut still
        # cancels this goal when the heading target is reached.
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses

        goal = FollowPath.Goal()
        goal.path = path

        self.get_logger().info(
            f'Turn arc → lane {self._lane_idx + 1}: '
            f'{len(poses)} arc poses  y={y_cur:.3f}→{y_target:.3f}  '
            f'r={(abs(y_target - y_cur) / 2.0):.3f}m  '
            f'target_yaw={math.degrees(self._turn_target_yaw):.1f}°')

        self._nav_epoch += 1
        epoch = self._nav_epoch
        future = self._follow_path_ac.send_goal_async(goal)
        future.add_done_callback(partial(self._on_turn_accepted, epoch=epoch))

    def _on_turn_accepted(self, future, epoch):
        handle = future.result()
        if epoch != self._nav_epoch:
            if handle.accepted:
                handle.cancel_goal_async()  # superseded before accept — cancel the orphan
            return
        if not handle.accepted:
            self.get_logger().error('Turn arc goal rejected; coverage halted.')
            self._in_turn = False
            self._phase = 'idle'
            return
        self._turn_goal_handle = handle
        handle.get_result_async().add_done_callback(
            partial(self._on_turn_done, epoch=epoch))

    def _on_turn_done(self, future, epoch):
        if epoch != self._nav_epoch or self._paused:
            return
        if self._turn_triggered:
            return
        self._in_turn = False
        if not self._goal_succeeded(future, 'Turn arc'):
            return
        # Normal goal completion — yaw check didn't fire first.
        self._after_turn()

    def _after_turn(self):
        """Start the next lane on the PLANNED grid y, not the robot's actual post-turn y.

        The arc exit overshoots (the robot's y at turn-end is the overshoot peak, e.g. 1.71 for a
        1.5 lane); building the next lane there put the lane AT the overshoot and the robot then drifted
        off its own lane (run 202230). Using the planned grid y keeps lanes on-grid so the FollowPath
        controller pulls the robot back toward the true lane. We still log the turn-end error."""
        planned_y = self._turn_y_target
        if self._odom is not None:
            actual_y = self._odom.pose.pose.position.y
            self.get_logger().info(
                f'Turn done: planned_y={planned_y:.3f}  actual_y={actual_y:.3f}  '
                f'Δ={actual_y - planned_y:+.3f}m  → next lane at PLANNED y (on-grid)')
        else:
            self.get_logger().warn('No odometry — using planned y for next lane.')
        self._current_y = planned_y

        self._forward = not self._forward
        # Gentle merge from the robot's ACTUAL post-turn pose (overshoot peak) onto the grid lane.
        sx = sy = None
        if self._odom is not None:
            sx = self._odom.pose.pose.position.x
            sy = self._odom.pose.pose.position.y
        self._run_lane(start_x=sx, start_y=sy)

    # ---- between-pass deadhead (multipass coverage) ----
    #
    # Between two passes the lanes are only tool_width apart, too tight for an in-pass arc.
    # Two selectable paths are available:
    #   teardrop: local minimum-radius loop on the same end rail
    #   multipass_boustrophedon: rounded route around the work-area perimeter, entering the
    #     next pass from the opposite end
    # Both use NavigateThroughPoses, like the in-pass arc turns.

    def _sample_arc(self, c, p0, p1, rho, r, step, out):
        """Append points along the arc on circle (centre c, radius r) from p0 to p1, turning
        rho=+1 CCW / -1 CW. Each point is (x, y, yaw=tangent heading)."""
        a0 = math.atan2(p0[1] - c[1], p0[0] - c[0])
        a1 = math.atan2(p1[1] - c[1], p1[0] - c[0])
        sweep = (a1 - a0) % (2.0 * math.pi) if rho > 0 else -((a0 - a1) % (2.0 * math.pi))
        if abs(sweep) < 1e-9:
            return
        n = max(2, int(math.ceil(r * abs(sweep) / max(step, 0.1))))
        tang = math.pi / 2.0 if rho > 0 else -math.pi / 2.0
        for k in range(1, n + 1):
            ang = a0 + sweep * (k / n)
            out.append((c[0] + r * math.cos(ang), c[1] + r * math.sin(ang), ang + tang))

    def _same_direction_return_loop(self, ya, yb, transition_idx):
        """Closest same-side return whose main straight is parallel to the lanes.

        Transitions after the first ``upper_return_count`` lanes loop above the work
        rectangle; the remaining transitions loop below it. Four exact-radius quarter arcs
        connect short vertical straights to one horizontal return straight. The outside Y
        rail is computed independently for every transition, so it stays as close as the
        radius and next-lane Y permit.
        """
        r = self.turn_radius
        upper = transition_idx < self.upper_return_count
        out = [(self._x1, ya, 0.0)]

        if upper:
            outside_y = max(ya, yb) + 2.0 * r
            self._sample_arc(
                (self._x1, ya + r), (self._x1, ya), (self._x1 + r, ya + r),
                1.0, r, self.waypoint_step, out)
            self._append_line_points(
                out, (self._x1 + r, ya + r), (self._x1 + r, outside_y - r),
                math.pi / 2.0)
            self._sample_arc(
                (self._x1, outside_y - r), (self._x1 + r, outside_y - r),
                (self._x1, outside_y), 1.0, r, self.waypoint_step, out)
            self._append_line_points(
                out, (self._x1, outside_y), (self._x0, outside_y), math.pi)
            self._sample_arc(
                (self._x0, outside_y - r), (self._x0, outside_y),
                (self._x0 - r, outside_y - r), 1.0, r, self.waypoint_step, out)
            self._append_line_points(
                out, (self._x0 - r, outside_y - r), (self._x0 - r, yb + r),
                -math.pi / 2.0)
            self._sample_arc(
                (self._x0, yb + r), (self._x0 - r, yb + r), (self._x0, yb),
                1.0, r, self.waypoint_step, out)
        else:
            outside_y = min(ya, yb) - 2.0 * r
            self._sample_arc(
                (self._x1, ya - r), (self._x1, ya), (self._x1 + r, ya - r),
                -1.0, r, self.waypoint_step, out)
            self._append_line_points(
                out, (self._x1 + r, ya - r), (self._x1 + r, outside_y + r),
                -math.pi / 2.0)
            self._sample_arc(
                (self._x1, outside_y + r), (self._x1 + r, outside_y + r),
                (self._x1, outside_y), -1.0, r, self.waypoint_step, out)
            self._append_line_points(
                out, (self._x1, outside_y), (self._x0, outside_y), math.pi)
            self._sample_arc(
                (self._x0, outside_y + r), (self._x0, outside_y),
                (self._x0 - r, outside_y + r), -1.0, r, self.waypoint_step, out)
            self._append_line_points(
                out, (self._x0 - r, outside_y + r), (self._x0 - r, yb - r),
                math.pi / 2.0)
            self._sample_arc(
                (self._x0, yb - r), (self._x0 - r, yb - r), (self._x0, yb),
                -1.0, r, self.waypoint_step, out)
        return out

    def _teardrop(self, xe, ya, yb, s, r, step):
        """Half-circle (D-curve) that repositions a lane end at (xe, ya) heading outward
        (s=+1 → +x, s=-1 → -x) to (xe, yb) heading inward. Radius = |dy|/2 — the unique
        semicircle that fits both endpoints. Symmetric with the in-pass arc turns on the
        opposite rail. Bulge = |dy|/2 outward (vs (√3+1)·r ≈ 4× larger for the old LRL)."""
        d_local = (yb - ya) if s > 0 else (ya - yb)

        def to_global(lx, ly, lyaw):
            return (xe + lx, ya + ly, lyaw) if s > 0 else (xe - lx, ya - ly, lyaw + math.pi)

        if abs(d_local) < 1e-6:
            return [(xe, yb, math.pi if s > 0 else 0.0)]
        loc = [(0.0, 0.0, 0.0)]
        self._sample_arc((0.0, d_local / 2.0), (0.0, 0.0), (0.0, d_local),
                         1.0 if d_local > 0 else -1.0, abs(d_local) / 2.0, step, loc)
        return [to_global(*p) for p in loc]

    def _deadhead_start(self, end_x):
        if self.coverage_path_mode == 'same_direction_loops':
            return self._x0, True
        at_right = abs(end_x - self._x1) < abs(end_x - self._x0)
        if self.coverage_path_mode == 'multipass_boustrophedon':
            # Enter the next pass from the opposite end after following the outside perimeter.
            return (self._x0, True) if at_right else (self._x1, False)
        # Local teardrop: return to the same end, facing into the next lane.
        return (self._x1, False) if at_right else (self._x0, True)

    def _perimeter_deadhead(self, a, b):
        """Forward-only rounded route outside the work rectangle from lane end a to b."""
        xa, ya = a
        xb, yb = b
        clearance = self.deadhead_clearance
        sa = 1.0 if abs(xa - self._x1) < abs(xa - self._x0) else -1.0
        sb = 1.0 if abs(xb - self._x1) < abs(xb - self._x0) else -1.0
        xa_outer = xa + sa * clearance
        xb_outer = xb + sb * clearance

        lower_y = min(self._y0, ya, yb) - clearance
        upper_y = max(self._y1, ya, yb) + clearance
        lower_cost = abs(ya - lower_y) + abs(yb - lower_y)
        upper_cost = abs(ya - upper_y) + abs(yb - upper_y)
        outside_y = lower_y if lower_cost <= upper_cost else upper_y

        corners = [
            a,
            (xa_outer, ya),
            (xa_outer, outside_y),
            (xb_outer, outside_y),
            (xb_outer, yb),
            b,
        ]
        return self._round_corners(corners, self.turn_radius, self.waypoint_step)

    def _deadhead_path(self, a, b):
        """Lane-end a=(xa,ya) → next-pass start b=(xb,yb)."""
        xa, ya = a
        xb, yb = b
        if self.deadhead_style == 'rounded':
            # Stay inside the work area: descend along the near rail, joined to both lane
            # ends by exact turn_radius arcs. a/b are already pulled in by R (see
            # _lane_x_bounds), so the arcs just reach the rail and never bulge outside.
            # Needs same-rail ends (coverage_path_mode:=teardrop).
            rail_x = self._x0 if abs(xa - self._x0) < abs(xa - self._x1) else self._x1
            corners = [a, (rail_x, ya), (rail_x, yb), b]
            return self._round_corners(corners, self.turn_radius, self.waypoint_step)
        if self.deadhead_style == 'direct':
            yaw = math.atan2(yb - ya, xb - xa)
            return [(lx, ly, yaw) for lx, ly in self._sample_line(a, b, self.waypoint_step)]
        if self.coverage_path_mode == 'multipass_boustrophedon':
            return self._perimeter_deadhead(a, b)
        xe = xa
        s = 1.0 if abs(xe - self._x1) < abs(xe - self._x0) else -1.0   # +1 outward = +x
        return self._teardrop(xe, ya, yb, s, self.turn_radius, self.waypoint_step)

    def _run_deadhead(self):
        self._phase = 'deadhead'
        # End of the lane we just finished (idx-1), and start of the next lane (idx);
        # both honour the rounded-style inset so the deadhead connects to the real ends.
        end_x = self._lane_x_bounds(self._lane_idx - 1, self._forward)[1]
        a = (end_x, self._current_y)
        next_y = self._lane_ys[self._lane_idx]
        _rail_x, new_forward = self._deadhead_start(end_x)
        start_x = self._lane_x_bounds(self._lane_idx, new_forward)[0]
        self._deadhead_next_y = next_y
        self._deadhead_new_forward = new_forward

        # DEFINITIVE between-pass fix (tile, runs 135832/180308/182318 all spiralled at the SAME
        # pass1→pass2 +x reversal): a rolling curve tracked by RPP CANNOT do the ~180° heading
        # reversal here. On tile the rear wheels slip, so RPP's heading correction while moving
        # doesn't stop in time → the robot over-rotates past the lane heading and spirals off the
        # area — every time, regardless of FollowPath/chunking/settle/start_x. The robust answer is
        # to NEVER reverse heading with a rolling RPP curve: use 'direct', which does the reversal
        # with in-place goal-checked Spins (Spin watches BNO yaw and STOPS at the target → it
        # physically cannot over-rotate). In-pass arc turns are small-angle and keep working, so
        # only the between-pass transition is pivot-based.
        if (self.deadhead_style == 'direct'
                and self.coverage_path_mode != 'same_direction_loops'):
            self._run_deadhead_direct(a, (start_x, next_y))
            return

        if self.coverage_path_mode == 'same_direction_loops':
            pts = self._same_direction_return_loop(
                self._current_y, next_y, self._lane_idx - 1)
        else:
            pts = self._deadhead_path(a, (start_x, next_y))
        poses = [
            self._pose(*self._to_map(lx, ly), self.area.yaw + lyaw)
            for lx, ly, lyaw in pts
        ]
        loop_bounds = ''
        if self.coverage_path_mode == 'same_direction_loops':
            side = 'upper' if (self._lane_idx - 1) < self.upper_return_count else 'lower'
            loop_bounds = (
                f' side={side} local_y={min(p[1] for p in pts):.2f}..'
                f'{max(p[1] for p in pts):.2f}m')

        self.get_logger().info(
            f'Deadhead → pass {self._lane_pass[self._lane_idx] + 1} '
            f'lane {self._lane_idx + 1}/{len(self._lane_ys)}: {len(poses)} poses  '
            f'({a[0]:.2f},{a[1]:.2f})→({start_x:.2f},{next_y:.2f})  '
            f'mode={self.coverage_path_mode} style={self.deadhead_style}{loop_bounds}')

        # FollowPath (not NavigateThroughPoses): track the exact rounded deadhead, no NavFn reshaping.
        # The deadhead used to hand off to the next lane with NavFn still commanding rotation, so the
        # robot over-rotated the entry into the next lane and looped off the area (run 210056, the
        # pass1->pass2 3.1->1.1 deadhead). Following the exact path lets RPP stop rotating when aligned.
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        goal = FollowPath.Goal()
        goal.path = path

        # Arm the yaw-cut for the rounded/teardrop deadhead too (not just in-pass arc turns). Its
        # bottom arc is a ~90° reversal onto the lane heading, and on the +x side that is a RIGHT
        # (CW) turn the robot over-rotates (run 202612: reached heading -179°, basically aligned to
        # 180°, but the curve still commanded cmd_w -1.31 and it spun past into a loop). Cutting the
        # instant the heading reaches the lane heading hands off to the gentle merge lane BEFORE the
        # curve can over-rotate. err-only cut (no overshoot/direction logic — the deadhead sweeps a
        # big arc). Skip for same_direction_loops (those loop outside, no single lane-heading target).
        if self.coverage_path_mode != 'same_direction_loops':
            self._turn_is_deadhead = True
            self._turn_target_yaw = self._norm_angle(
                self.area.yaw + (0.0 if new_forward else math.pi))
            self._turn_triggered = False
            self._turn_required_yaw_delta = None     # err-threshold cut only
            self._turn_last_yaw = (
                self._yaw_from_odom(self._odom) if self._odom is not None
                else self._turn_target_yaw)
            self._turn_yaw_progress = 0.0
            self._turn_start_time = self.get_clock().now()
            self._in_turn = True

        self._nav_epoch += 1
        epoch = self._nav_epoch
        future = self._follow_path_ac.send_goal_async(goal)
        future.add_done_callback(partial(self._on_deadhead_accepted, epoch=epoch))

    def _on_deadhead_accepted(self, future, epoch):
        handle = future.result()
        if epoch != self._nav_epoch:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().error('Deadhead goal rejected; coverage halted.')
            self._phase = 'idle'
            return
        self._turn_goal_handle = handle
        handle.get_result_async().add_done_callback(
            partial(self._on_deadhead_done, epoch=epoch))

    def _on_deadhead_done(self, future, epoch):
        if epoch != self._nav_epoch or self._paused:
            return
        # The yaw-cut already fired and called _after_deadhead — this is the cancelled goal's
        # CANCELED result coming back. Ignore it (without this guard _goal_succeeded would see
        # not-SUCCEEDED and halt coverage right after a successful cut).
        if self._turn_triggered:
            return
        self._turn_goal_handle = None
        self._in_turn = False
        if not self._goal_succeeded(future, 'Deadhead'):
            return
        self._after_deadhead()

    def _goal_succeeded(self, future, label):
        """Advance coverage only after Nav2 explicitly reports success."""
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(
                f'{label} result unavailable ({exc}); coverage halted.')
            self._phase = 'idle'
            self._in_turn = False
            return False

        if status == GoalStatus.STATUS_SUCCEEDED:
            return True

        self.get_logger().error(
            f'{label} ended with action status={status}; coverage halted instead of '
            'advancing to the next lane.')
        self._phase = 'idle'
        self._in_turn = False
        return False

    def _after_deadhead(self):
        # Start the new pass at the exact planned lane y (keeps the interleave gap-free).
        self._current_y = self._deadhead_next_y
        self._forward = self._deadhead_new_forward
        # Begin the lane at the robot's ACTUAL x, not the planned (inset) lane start. The rounded
        # deadhead leaves the robot ~at the rail inset but slightly SHORT of it (tile under-rotation):
        # run 180308 the robot stopped at x=4.57, near-aligned on lane y=0.8, but the planned lane
        # start was the inset x=4.4 — BEHIND it. RPP then saw the path start essentially on top of /
        # behind the robot → carrot nearly perpendicular → commanded max angular (cmd_w ramped to the
        # -2.5 saturation) → the small ~20° entry tilt was over-rotated into a CW spiral off the area.
        # Starting where the robot actually is puts the first carrot a normal lookahead AHEAD, so RPP
        # only needs the gentle ~20° heading correction. (area frame is axis-aligned with map at origin,
        # area_yaw 0 — same assumption used by obstacle-resume and _after_turn's actual-y read.)
        sx = sy = None
        if self._odom is not None:
            sx = self._odom.pose.pose.position.x
            sy = self._odom.pose.pose.position.y
        self._run_lane(start_x=sx, start_y=sy)

    # ---- 'direct' deadhead: pivot → straight → pivot (Plan B, tile) ----
    #
    # Build the between-pass reposition as discrete, goal-checked steps instead of one curved
    # FollowPath. For the teardrop same-rail case a=(x,ya) and b=(x,yb) share an x, so the
    # maneuver is: pivot to face the travel direction (±90° toward yb) → drive the straight
    # a→b → pivot to the next lane heading (0 or π). Each Spin stops at its target heading
    # (its own goal check), so the robot physically cannot over-rotate into a spiral; the
    # rear-slip that ran away on the rounded arc only costs a little centre drift per pivot,
    # which the following straight / lane FollowPath then corrects.
    def _run_deadhead_direct(self, a, b):
        xa, ya = a
        xb, yb = b
        h_next = self._norm_angle(
            self.area.yaw + (0.0 if self._deadhead_new_forward else math.pi))
        segs: List = []
        if math.hypot(xb - xa, yb - ya) > 1e-3:
            travel = self._norm_angle(
                self.area.yaw + math.atan2(yb - ya, xb - xa))
            segs.append(('pivot', travel))
            segs.append(('drive', a, b))
        segs.append(('pivot', h_next))
        self._dh_segments = segs
        self._dh_index = 0
        self.get_logger().info(
            f'Deadhead(direct) → pass {self._lane_pass[self._lane_idx] + 1} '
            f'lane {self._lane_idx + 1}/{len(self._lane_ys)}: '
            f'({xa:.2f},{ya:.2f})→({xb:.2f},{yb:.2f}) {len(segs)} segments '
            f'[{", ".join(s[0] for s in segs)}]')
        self._run_dh_segment()

    def _run_dh_segment(self):
        if self._paused:
            return
        if self._dh_index >= len(self._dh_segments):
            self._after_deadhead()
            return
        seg = self._dh_segments[self._dh_index]
        self._nav_epoch += 1
        epoch = self._nav_epoch
        if seg[0] == 'pivot':
            self._dh_send_pivot(seg[1], epoch)
        else:
            self._dh_send_drive(seg[1], seg[2], epoch)

    def _dh_send_pivot(self, target_yaw_map, epoch):
        if not self._spin_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Spin server unavailable for deadhead pivot; coverage halted.')
            self._phase = 'idle'
            return
        cur = self._yaw_from_odom(self._odom) if self._odom is not None else target_yaw_map
        delta = self._angle_diff(target_yaw_map, cur)
        self.get_logger().info(
            f'  seg {self._dh_index + 1}/{len(self._dh_segments)} pivot '
            f'{math.degrees(delta):+.0f}° → heading {math.degrees(target_yaw_map):.0f}°')
        goal = Spin.Goal()
        goal.target_yaw = float(delta)
        goal.time_allowance = Duration(sec=30)
        future = self._spin_ac.send_goal_async(goal)
        future.add_done_callback(partial(self._on_dh_seg_accepted, epoch=epoch))

    def _dh_send_drive(self, a, b, epoch):
        local_yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        poses = [
            self._pose(*self._to_map(lx, ly), self.area.yaw + local_yaw)
            for lx, ly in self._sample_line(a, b, self.waypoint_step)
        ]
        self.get_logger().info(
            f'  seg {self._dh_index + 1}/{len(self._dh_segments)} drive '
            f'({a[0]:.2f},{a[1]:.2f})→({b[0]:.2f},{b[1]:.2f}) {len(poses)} poses')
        self._dh_send_followpath(poses, epoch)

    def _dh_send_followpath(self, poses, epoch):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        goal = FollowPath.Goal()
        goal.path = path
        future = self._follow_path_ac.send_goal_async(goal)
        future.add_done_callback(partial(self._on_dh_seg_accepted, epoch=epoch))

    def _on_dh_seg_accepted(self, future, epoch):
        handle = future.result()
        if epoch != self._nav_epoch:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().error('Deadhead segment rejected; coverage halted.')
            self._phase = 'idle'
            return
        self._turn_goal_handle = handle
        handle.get_result_async().add_done_callback(
            partial(self._on_dh_seg_done, epoch=epoch))

    def _on_dh_seg_done(self, future, epoch):
        if epoch != self._nav_epoch or self._paused:
            return
        self._turn_goal_handle = None
        # 'pivot' (a Spin) is lenient — it may report non-SUCCEEDED when it stops just inside the
        # yaw tolerance; we re-measure yaw on the next segment anyway, so advance regardless. The
        # 'drive' straight must succeed (it positions the robot for the final pivot / lane).
        seg_kind = self._dh_segments[self._dh_index][0]
        if seg_kind == 'drive' and not self._goal_succeeded(future, 'Deadhead drive'):
            return
        if seg_kind == 'pivot':
            try:
                status = future.result().status
                if status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().warn(
                        f'Deadhead pivot {self._dh_index + 1} ended status={status} '
                        '(not SUCCEEDED); continuing — next step re-measures yaw.')
            except Exception:
                pass
        self._dh_index += 1
        # Settle between every direct segment: actively hold cmd_vel=0 so the robot comes fully to
        # REST (heading stops drifting) before the next pivot/drive and before the lane. Cheap
        # insurance that each goal-checked Spin starts from a clean standstill.
        if self.deadhead_settle_sec > 0.0:
            self._dh_start_settle(epoch)
        else:
            self._run_dh_segment()

    def _dh_start_settle(self, epoch):
        self._dh_settle_ticks = max(1, int(round(self.deadhead_settle_sec / 0.1)))
        self._cmd_pub.publish(Twist())
        self._dh_settle_timer = self.create_timer(
            0.1, partial(self._dh_settle_tick, epoch=epoch))

    def _dh_settle_tick(self, epoch):
        if self._dh_settle_timer is not None:
            self._dh_settle_timer.cancel()
            self._dh_settle_timer = None
        if epoch != self._nav_epoch or self._paused:
            return
        self._cmd_pub.publish(Twist())   # hold zero
        self._dh_settle_ticks -= 1
        if self._dh_settle_ticks > 0:
            self._dh_settle_timer = self.create_timer(
                0.1, partial(self._dh_settle_tick, epoch=epoch))
        else:
            self._run_dh_segment()

    # ---- auto-mode obstacle stop ----
    #
    # The ZED front-box monitor calls _on_obstacle_update on every cloud (~10 Hz).
    # We only pause during a LANE (not a turn): turns are short, and mid-arc the robot
    # is bulged out to the side so there is no clean "remaining arc" to resume — if an
    # obstacle persists through the turn, the pause engages the instant the next lane
    # starts. On pause we cancel the active Nav2 goal (so the progress checker can't
    # abort and skip a lane), zero /cmd_vel for a crisp stop, and beep. On 3 s of clear
    # we re-send the lane from the robot's current x.

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_obstacle_update(self, present: bool, nearest_x):
        self._obstacle_present = present
        if self._paused:
            if present:
                self._clear_start = None          # restart the clear timer
            else:
                now = self._now_sec()
                if self._clear_start is None:
                    self._clear_start = now
                elif (now - self._clear_start) >= self.obstacle_clear_time_sec:
                    self._resume_after_clear()
        else:
            if present and self._started and self._phase == 'lane':
                self._pause_for_obstacle(nearest_x)

    def _pause_for_obstacle(self, nearest_x):
        self._paused = True
        self._clear_start = None
        # Invalidate any in-flight goal's result callback, then cancel it.
        self._nav_epoch += 1
        if self._lane_goal_handle is not None:
            self._lane_goal_handle.cancel_goal_async()
            self._lane_goal_handle = None
        if self._turn_goal_handle is not None:
            self._turn_goal_handle.cancel_goal_async()
            self._turn_goal_handle = None
        self._in_turn = False
        dist = f'{nearest_x:.2f} m' if nearest_x is not None else 'unknown'
        self.get_logger().warn(f'Obstacle at {dist} → STOP + buzzer; pausing coverage.')

    def _resume_after_clear(self):
        self._paused = False
        self._clear_start = None
        self.get_logger().info(
            f'Path clear ≥ {self.obstacle_clear_time_sec:.0f}s → resuming lane.')
        start_x = None
        if self._odom is not None:
            start_x = self._odom.pose.pose.position.x
        self._run_lane(start_x=start_x)

    def _override_cmd_cb(self):
        # While paused, hold the robot stopped (also feeds the mixer's 0.3 s watchdog).
        if self._paused:
            self._cmd_pub.publish(Twist())

    def _beep_cb(self):
        if self._paused:
            msg = Float32()
            msg.data = float(self.beep_duration_sec)
            self._buzzer_pub.publish(msg)

    # ---- spiral (static, not dynamically adjusted) ----

    def generate_spiral_poses(self) -> List[PoseStamped]:
        spacing = self.lane_spacing
        x0 = self._x0
        x1 = self._x1
        y0 = self.margin
        y1 = max(self.margin, self.area.height - self.margin)
        local_points = []
        while (x1 - x0) > 1e-6 and (y1 - y0) > 1e-6:
            self._append_line_points(local_points, (x0, y0), (x1, y0), 0.0)
            self._append_line_points(local_points, (x1, y0), (x1, y1), math.pi / 2.0)
            self._append_line_points(local_points, (x1, y1), (x0, y1), math.pi)
            next_y0 = y0 + spacing
            if next_y0 >= y1:
                break
            self._append_line_points(local_points, (x0, y1), (x0, next_y0), -math.pi / 2.0)
            x0 += spacing; y0 += spacing; x1 -= spacing; y1 -= spacing
            if x0 > x1 or y0 > y1:
                break
            self._append_line_points(local_points, (x0 - spacing, y0), (x0, y0), 0.0)
        poses = []
        for lx, ly, lyaw in local_points:
            mx, my = self._to_map(lx, ly)
            poses.append(self._pose(mx, my, self.area.yaw + lyaw))
        return poses


def main():
    rclpy.init()
    node = CoverageFollowWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
