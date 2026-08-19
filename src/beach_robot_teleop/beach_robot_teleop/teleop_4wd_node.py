import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class Teleop4WDSkid(Node):
    def __init__(self):
        super().__init__('teleop_4wd')

        # ---------------- Parameters ----------------
        self.declare_parameter('max_linear', 0.35)   # sand: match mixer max_v (drives smooth 0.15-0.35)
        self.declare_parameter('max_angular', 0.35)  # sand: match mixer max_w

        # Joystick axes (F710 XInput usually: left_x=0, left_y=1)
        self.declare_parameter('axis_linear', 1)     # left stick Y
        self.declare_parameter('axis_angular', 0)    # left stick X
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)

        # invert + deadzone
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('deadzone_linear', 0.08)
        self.declare_parameter('deadzone_angular', 0.08)
        self.declare_parameter('expo_linear', 1.0)
        self.declare_parameter('expo_angular', 1.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('joy_timeout_sec', 0.5)
        self.declare_parameter('linear_slew_rate', 0.35)
        self.declare_parameter('angular_slew_rate', 0.90)
        self.declare_parameter('linear_decel_rate', 3.0)
        self.declare_parameter('angular_decel_rate', 4.0)
        self.declare_parameter('spin_snap_enabled', True)
        self.declare_parameter('spin_snap_angular_min', 0.25)
        self.declare_parameter('spin_snap_linear_ratio', 0.45)

        # Deadman (LB)
        self.declare_parameter('enable_button', 4)

        self.declare_parameter('button_a', 0)
        self.declare_parameter('button_b', 1)
        self.declare_parameter('button_x', 2)
        self.declare_parameter('button_y', 3)

        self.declare_parameter('mode_hold_sec', 3.0)
        self.declare_parameter('estop_hold_sec', 3.0)

        self.declare_parameter('auto_start_service', 'auto_clean/start')
        self.declare_parameter('cmd_vel_out', 'cmd_vel')

        # ----- read parameters -----
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.axis_linear = int(self.get_parameter('axis_linear').value)
        self.axis_angular = int(self.get_parameter('axis_angular').value)
        self.scale_linear = float(self.get_parameter('scale_linear').value)
        self.scale_angular = float(self.get_parameter('scale_angular').value)

        self.invert_linear = bool(self.get_parameter('invert_linear').value)
        self.invert_angular = bool(self.get_parameter('invert_angular').value)
        self.deadzone_linear = float(self.get_parameter('deadzone_linear').value)
        self.deadzone_angular = float(self.get_parameter('deadzone_angular').value)
        self.expo_linear = float(self.get_parameter('expo_linear').value)
        self.expo_angular = float(self.get_parameter('expo_angular').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.joy_timeout_sec = float(self.get_parameter('joy_timeout_sec').value)
        self.linear_slew_rate = float(self.get_parameter('linear_slew_rate').value)
        self.angular_slew_rate = float(self.get_parameter('angular_slew_rate').value)
        self.linear_decel_rate = float(self.get_parameter('linear_decel_rate').value)
        self.angular_decel_rate = float(self.get_parameter('angular_decel_rate').value)
        self.spin_snap_enabled = bool(self.get_parameter('spin_snap_enabled').value)
        self.spin_snap_angular_min = float(self.get_parameter('spin_snap_angular_min').value)
        self.spin_snap_linear_ratio = float(self.get_parameter('spin_snap_linear_ratio').value)

        self.enable_button = int(self.get_parameter('enable_button').value)

        self.button_a = int(self.get_parameter('button_a').value)
        self.button_b = int(self.get_parameter('button_b').value)
        self.button_x = int(self.get_parameter('button_x').value)
        self.button_y = int(self.get_parameter('button_y').value)

        self.mode_hold_sec = float(self.get_parameter('mode_hold_sec').value)
        self.estop_hold_sec = float(self.get_parameter('estop_hold_sec').value)

        self.auto_start_service_name = self.get_parameter('auto_start_service').value
        self.cmd_vel_out_topic = self.get_parameter('cmd_vel_out').value

        # ---------------- State ----------------
        self.manual_mode = True
        self.vibration_on = False
        self.estop_active = False
        self.deadman_active = False

        self.prev_a = False
        self.prev_b = False
        self.prev_x = False
        self.prev_y = False

        self.a_press_start = None
        self.a_hold_toggled = False
        self.y_press_start = None
        self.last_joy_time = None
        self.last_publish_time = self._now()
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0

        # ---------------- ROS IO ----------------
        self.sub_joy = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.pub_vel_cmd = self.create_publisher(Twist, self.cmd_vel_out_topic, 10)
        self.pub_vibration = self.create_publisher(Bool, 'vibration_enable', 10)
        self.pub_mode = self.create_publisher(String, 'control_mode', 10)
        self.pub_estop = self.create_publisher(Bool, 'e_stop', 10)

        self.auto_start_client = self.create_client(Trigger, self.auto_start_service_name)
        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.publish_timer = self.create_timer(period, self.publish_timer_cb)

        self.get_logger().info(
            f'Teleop4WDSkid started: max_linear={self.max_linear:.3f} '
            f'max_angular={self.max_angular:.3f} '
            f'publish_rate={self.publish_rate_hz:.1f} Hz'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _apply_deadzone(self, x: float, dz: float) -> float:
        if abs(x) < dz:
            return 0.0
        # optional: rescale outside deadzone to keep full range
        # sign * (|x|-dz)/(1-dz)
        sign = 1.0 if x >= 0.0 else -1.0
        return sign * (abs(x) - dz) / max(1e-6, (1.0 - dz))

    def _clamp(self, x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def _shape_axis(self, x: float, expo: float) -> float:
        expo = max(1.0, expo)
        if x == 0.0 or expo == 1.0:
            return x
        return math.copysign(abs(x) ** expo, x)

    def _slew(self, current: float, target: float, rate: float, dt: float) -> float:
        step = max(0.0, rate) * max(0.0, dt)
        return current + self._clamp(target - current, -step, step)

    def _slew_rate_for(self, current: float, target: float, accel_rate: float, decel_rate: float) -> float:
        sign_change = current * target < 0.0
        reducing_magnitude = abs(target) < abs(current)
        return decel_rate if sign_change or reducing_magnitude else accel_rate

    def _set_target_zero(self):
        self.target_v = 0.0
        self.target_w = 0.0

    def _publish_cmd(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_vel_cmd.publish(cmd)

    def start_auto_clean(self):
        if self.manual_mode:
            self.get_logger().warn('B pressed but robot is in MANUAL mode')
            return
        if self.estop_active:
            self.get_logger().warn('Cannot start auto cleaning: E-STOP is active')
            return
        if not self.auto_start_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(f'Service "{self.auto_start_service_name}" not available')
            return
        future = self.auto_start_client.call_async(Trigger.Request())
        future.add_done_callback(self._auto_start_response_cb)

    def _auto_start_response_cb(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Auto cleaning service call failed: {e}')
            return
        if resp.success:
            self.get_logger().info(f'Auto cleaning started: {resp.message}')
        else:
            self.get_logger().warn(f'Auto cleaning rejected: {resp.message}')

    def joy_callback(self, msg: Joy):
        now = self._now()
        self.last_joy_time = now
        axes = msg.axes
        buttons = msg.buttons

        def get_axis(idx: int, default: float = 0.0) -> float:
            return axes[idx] if 0 <= idx < len(axes) else default

        def get_button(idx: int) -> bool:
            return bool(buttons[idx]) if 0 <= idx < len(buttons) else False

        # -------- Read axes --------
        raw_lin = get_axis(self.axis_linear, 0.0)
        raw_ang = get_axis(self.axis_angular, 0.0)

        # invert if needed
        if self.invert_linear:
            raw_lin = -raw_lin
        if self.invert_angular:
            raw_ang = -raw_ang

        # deadzone + clamp
        raw_lin = self._apply_deadzone(raw_lin, self.deadzone_linear)
        raw_ang = self._apply_deadzone(raw_ang, self.deadzone_angular)
        raw_lin = self._clamp(raw_lin, -1.0, 1.0)
        raw_ang = self._clamp(raw_ang, -1.0, 1.0)
        raw_lin = self._shape_axis(raw_lin, self.expo_linear)
        raw_ang = self._shape_axis(raw_ang, self.expo_angular)
        if (
            self.spin_snap_enabled and
            abs(raw_ang) >= self.spin_snap_angular_min and
            abs(raw_lin) <= abs(raw_ang) * self.spin_snap_linear_ratio
        ):
            raw_lin = 0.0

        # -------- Read buttons --------
        enable_pressed = get_button(self.enable_button)
        self.deadman_active = enable_pressed
        a_pressed = get_button(self.button_a)
        b_pressed = get_button(self.button_b)
        x_pressed = get_button(self.button_x)
        y_pressed = get_button(self.button_y)

        # ---------------- A hold: toggle manual/auto ----------------
        if a_pressed and not self.prev_a:
            self.a_press_start = now
            self.a_hold_toggled = False
        elif a_pressed and self.prev_a:
            if (self.a_press_start is not None
                    and not self.a_hold_toggled
                    and (now - self.a_press_start) >= self.mode_hold_sec):
                self.manual_mode = not self.manual_mode
                self.a_hold_toggled = True

                mode_str = 'manual' if self.manual_mode else 'auto'
                self.get_logger().info(f'Control mode switched to: {mode_str}')
                m = String()
                m.data = mode_str
                self.pub_mode.publish(m)
        else:
            self.a_press_start = None
            self.a_hold_toggled = False

        # ---------------- Y: E-STOP ----------------
        if y_pressed and not self.prev_y:
            if not self.estop_active:
                self.estop_active = True
                self.get_logger().warn('E-STOP activated by Y button')
                # Safety: force the vibration sieve off on E-STOP so the motor
                # never keeps running while the robot is halted.
                if self.vibration_on:
                    self.vibration_on = False
                    vmsg = Bool()
                    vmsg.data = False
                    self.pub_vibration.publish(vmsg)
                    self.get_logger().warn('Vibration motor forced OFF by E-STOP')
            self.y_press_start = now
        elif y_pressed and self.prev_y:
            if (self.estop_active
                    and self.y_press_start is not None
                    and (now - self.y_press_start) >= self.estop_hold_sec):
                self.estop_active = False
                self.get_logger().info('E-STOP released after holding Y')
                self.y_press_start = None
        else:
            self.y_press_start = None

        # ---------------- X: vibration toggle (manual only) ----------------
        if self.manual_mode and x_pressed and not self.prev_x:
            self.vibration_on = not self.vibration_on
            vmsg = Bool()
            vmsg.data = self.vibration_on
            self.pub_vibration.publish(vmsg)

        # ---------------- B: start auto clean (rising edge) ----------------
        if b_pressed and not self.prev_b:
            self.start_auto_clean()

        # ==============================================================
        # Update target cmd_vel.
        # - manual mode + deadman + not estop => send motion
        # ==============================================================
        if self.estop_active:
            self._set_target_zero()
        elif self.manual_mode and enable_pressed:
            self.target_v = raw_lin * self.scale_linear * self.max_linear
            self.target_w = raw_ang * self.scale_angular * self.max_angular
        elif self.manual_mode and not enable_pressed:
            self._set_target_zero()

        # publish mode + estop (optional: keep as-is)
        mode_msg = String()
        mode_msg.data = 'manual' if self.manual_mode else 'auto'
        self.pub_mode.publish(mode_msg)

        estop_msg = Bool()
        estop_msg.data = self.estop_active
        self.pub_estop.publish(estop_msg)

        self.prev_a = a_pressed
        self.prev_b = b_pressed
        self.prev_x = x_pressed
        self.prev_y = y_pressed

    def publish_timer_cb(self):
        now = self._now()
        dt = max(0.0, now - self.last_publish_time)
        self.last_publish_time = now

        joy_stale = (
            self.last_joy_time is None or
            (now - self.last_joy_time) > self.joy_timeout_sec
        )
        # Publish /cmd_vel ONLY while actively hand-driving: manual mode + deadman held + not
        # e-stop + fresh joystick. In EVERY other case (auto mode, deadman released, e-stop,
        # stale joy) stay SILENT — otherwise teleop spams zero /cmd_vel and fights Nav2 during
        # coverage (controller reports "Failed to make progress" and the robot never moves).
        # Stopping is handled elsewhere: the ESP32 bridge E-STOP force-zeros motors on /e_stop,
        # and the mixer's ~300 ms cmd_vel watchdog zeros the wheels when nobody is publishing.
        if (not self.manual_mode or not self.deadman_active
                or self.estop_active or joy_stale):
            self._set_target_zero()
            self.current_v = 0.0
            self.current_w = 0.0
            return

        self.current_v = self._slew(
            self.current_v,
            self.target_v,
            self._slew_rate_for(
                self.current_v,
                self.target_v,
                self.linear_slew_rate,
                self.linear_decel_rate,
            ),
            dt,
        )
        self.current_w = self._slew(
            self.current_w,
            self.target_w,
            self._slew_rate_for(
                self.current_w,
                self.target_w,
                self.angular_slew_rate,
                self.angular_decel_rate,
            ),
            dt,
        )

        if abs(self.current_v) < 1e-4:
            self.current_v = 0.0
        if abs(self.current_w) < 1e-4:
            self.current_w = 0.0

        self._publish_cmd(self.current_v, self.current_w)


def main(args=None):
    rclpy.init(args=args)
    node = Teleop4WDSkid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
