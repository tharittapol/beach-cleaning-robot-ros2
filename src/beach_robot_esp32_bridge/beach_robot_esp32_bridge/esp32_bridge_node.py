import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from sensor_msgs.msg import Imu, Range

import serial


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('timeout', 0.05)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('enc_vel_max_abs_mps', 3.0)
        self.declare_parameter('enc_vel_max_step_mps', 1.0)
        self.declare_parameter('wheel_cmd_send_rate_hz', 30.0)
        self.declare_parameter('wheel_cmd_stale_timeout_sec', 0.5)
        self.declare_parameter('safety_estop_topic', '/safety/e_stop')
        self.declare_parameter('esp32_debug_enabled', False)
        self.declare_parameter('publish_raw_json', False)

        # Read parameters and store in self.*
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.imu_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value
        self.enc_vel_max_abs_mps = float(self.get_parameter('enc_vel_max_abs_mps').value)
        self.enc_vel_max_step_mps = float(self.get_parameter('enc_vel_max_step_mps').value)
        self.wheel_cmd_send_rate_hz = float(self.get_parameter('wheel_cmd_send_rate_hz').value)
        self.wheel_cmd_stale_timeout_sec = float(self.get_parameter('wheel_cmd_stale_timeout_sec').value)
        self.safety_estop_topic = str(self.get_parameter('safety_estop_topic').value)
        self.esp32_debug_enabled = bool(self.get_parameter('esp32_debug_enabled').value)
        self.publish_raw_json_enabled = bool(self.get_parameter('publish_raw_json').value)
        self.last_enc_vel = None
        self.last_enc_reject_warn_time = 0.0
        self.latest_wheel_cmd = None
        self.latest_wheel_cmd_time = 0.0
        self.wheel_cmd_seq = 0
        self.wheel_cmd_period = 1.0 / max(self.wheel_cmd_send_rate_hz, 1.0)
        self.last_wheel_cmd_write_time = 0.0
        self.manual_estop_active = False
        self.safety_estop_active = False
        self.estop_active = False

        # Ultrasonic params
        self.declare_parameter('ultra_min_m', 0.02)
        self.declare_parameter('ultra_max_m', 2.50)
        self.declare_parameter('ultra_fov_rad', 0.52)  # ~30 deg
        self.declare_parameter('ultra_frame_left', 'ultra_left_link')
        self.declare_parameter('ultra_frame_middle', 'ultra_middle_link')
        self.declare_parameter('ultra_frame_right', 'ultra_right_link')

        self.ultra_min_m = self.get_parameter('ultra_min_m').value
        self.ultra_max_m = self.get_parameter('ultra_max_m').value
        self.ultra_fov_rad = self.get_parameter('ultra_fov_rad').value
        self.ultra_frame_left = self.get_parameter('ultra_frame_left').value
        self.ultra_frame_middle = self.get_parameter('ultra_frame_middle').value
        self.ultra_frame_right = self.get_parameter('ultra_frame_right').value

        # Ultrasonic publishers (sensor_msgs/Range)
        self.pub_ultra_left = self.create_publisher(Range, '/ultrasonic/left', 10)
        self.pub_ultra_middle = self.create_publisher(Range, '/ultrasonic/middle', 10)
        self.pub_ultra_right = self.create_publisher(Range, '/ultrasonic/right', 10)

        self.ser = None
        self.serial_lock = threading.RLock()

        # Publishers
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.pub_enc_vel = self.create_publisher(
            Float32MultiArray,
            'enc_vel',
            10
        )
        self.pub_imu = self.create_publisher(
            Imu,
            'imu/data',
            10
        )
        self.pub_esp32_debug = self.create_publisher(
            String,
            'esp32/debug',
            debug_qos
        )
        self.pub_esp32_raw_json = self.create_publisher(
            String,
            'esp32/raw_json',
            debug_qos
        )

        # Subscribers
        self.sub_wheel_cmd = self.create_subscription(
            Float32MultiArray,
            'wheel_cmd',
            self.wheel_cmd_callback,
            10
        )
        self.sub_buzzer = self.create_subscription(
            Float32,
            'buzzer_duration',
            self.buzzer_callback,
            10
        )
        self.sub_vibration = self.create_subscription(
            Bool,
            'vibration_enable',
            self.vibration_callback,
            10
        )
        self.sub_json_cmd = self.create_subscription(
            String,
            'esp32/json_cmd',
            self.json_cmd_callback,
            10
        )
        # Hardware-level E-STOP: manual /e_stop and /safety/e_stop are ORed. While either is
        # true, the timer forces zero wheel_cmd regardless of Nav2 / teleop / mixer output.
        self.sub_estop = self.create_subscription(
            Bool,
            'e_stop',
            self.estop_callback,
            10
        )
        self.sub_safety_estop = self.create_subscription(
            Bool,
            self.safety_estop_topic,
            self.safety_estop_callback,
            10
        )

        self.wheel_cmd_timer = self.create_timer(
            self.wheel_cmd_period,
            self.send_latest_wheel_cmd,
        )

        # Try to open serial (with retry)
        self.open_serial_with_retry()

        # Start read thread
        self.read_thread = threading.Thread(
            target=self.read_loop,
            daemon=True
        )
        self.read_thread.start()

        self.get_logger().info(
            f'ESP32Bridge node started. wheel_cmd_send_rate_hz={self.wheel_cmd_send_rate_hz:.1f}'
        )

    # ------------------------------------------------------------------
    # Serial helpers
    # ------------------------------------------------------------------
    def open_serial_with_retry(self):
        """Try to open the serial port, retrying until success or node shutdown."""
        while rclpy.ok():
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )
                self.get_logger().info(
                    f'Opened serial port {self.port} at {self.baudrate} baud'
                )
                configured = self.write_json_line(
                    {'dbg_enable': self.esp32_debug_enabled},
                    'debug mode',
                    reconnect_on_error=False,
                )
                if configured:
                    self.get_logger().info(
                        f'ESP32 debug mode default: '
                        f'{"enabled" if self.esp32_debug_enabled else "disabled"}'
                    )
                return
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Failed to open serial port {self.port}: {e}. Retrying in 2s...'
                )
                time.sleep(2.0)

    def close_serial(self):
        with self.serial_lock:
            if self.ser is not None and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None

    # ------------------------------------------------------------------
    # Callbacks to send commands to ESP32
    # ------------------------------------------------------------------
    def write_json_line(self, data: dict, label: str, reconnect_on_error: bool = True):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn(f'Serial not open, cannot send {label}')
            return False

        line = json.dumps(data, separators=(',', ':')) + '\n'
        encoded = line.encode('utf-8')

        with self.serial_lock:
            try:
                self.ser.write(encoded)
                return True
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error ({label}): {e}')
                self.close_serial()
                if reconnect_on_error:
                    self.open_serial_with_retry()
                return False

    def wheel_cmd_callback(self, msg: Float32MultiArray):
        """Store latest wheel_cmd; the timer owns serial writes for stable pacing."""
        self.latest_wheel_cmd = [float(x) for x in msg.data[:4]]
        self.latest_wheel_cmd_time = time.monotonic()

    def estop_callback(self, msg: Bool):
        self.manual_estop_active = bool(msg.data)
        self._update_estop_state()

    def safety_estop_callback(self, msg: Bool):
        self.safety_estop_active = bool(msg.data)
        self._update_estop_state()

    def _update_estop_state(self):
        active = self.manual_estop_active or self.safety_estop_active
        if active != self.estop_active:
            sources = []
            if self.manual_estop_active:
                sources.append('manual')
            if self.safety_estop_active:
                sources.append('safety')
            source_text = '+'.join(sources) if sources else 'none'
            self.get_logger().warn(
                f'E-STOP {"ENGAGED" if active else "released"} '
                f'(source={source_text}) → '
                f'{"forcing zero motor output" if active else "resuming"}')
        self.estop_active = active

    def send_latest_wheel_cmd(self):
        now = time.monotonic()
        if self.estop_active:
            # E-STOP: keep streaming zeros to the ESP32 (overrides Nav2/teleop/mixer entirely).
            self.wheel_cmd_seq = (self.wheel_cmd_seq + 1) & 0x7fffffff
            if self.write_json_line({'wheel_cmd': [0.0, 0.0, 0.0, 0.0],
                                     'cmd_seq': self.wheel_cmd_seq}, 'wheel_cmd'):
                self.last_wheel_cmd_write_time = now
            return
        if self.latest_wheel_cmd is None:
            return
        wheel_cmd = self.latest_wheel_cmd
        if (
            self.wheel_cmd_stale_timeout_sec > 0.0 and
            (now - self.latest_wheel_cmd_time) > self.wheel_cmd_stale_timeout_sec
        ):
            wheel_cmd = [0.0, 0.0, 0.0, 0.0]
        self.wheel_cmd_seq = (self.wheel_cmd_seq + 1) & 0x7fffffff
        if self.write_json_line(
            {
                'wheel_cmd': wheel_cmd,
                'cmd_seq': self.wheel_cmd_seq,
            },
            'wheel_cmd',
        ):
            self.last_wheel_cmd_write_time = now

    def buzzer_callback(self, msg: Float32):
        """Send buzzer_duration to ESP32 as JSON: {"buzzer_duration": sec}"""
        self.write_json_line({'buzzer_duration': float(msg.data)}, 'buzzer')

    def vibration_callback(self, msg: Bool):
        """
        Send vibration_enable to ESP32 as JSON:
        {"vibration_enable": true/false}
        """
        self.write_json_line({'vibration_enable': bool(msg.data)}, 'vibration')

    def json_cmd_callback(self, msg: String):
        """Forward a validated JSON object to ESP32 for debug/tuning commands."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid esp32/json_cmd JSON: {e}')
            return

        if not isinstance(data, dict):
            self.get_logger().warn('esp32/json_cmd must be a JSON object')
            return

        self.write_json_line(data, 'json_cmd')

    # ------------------------------------------------------------------
    # Read loop from ESP32
    # ------------------------------------------------------------------
    def read_loop(self):
        """
        Run in separate thread, read JSON line by line from ESP32.

        Expected JSON from ESP32:
        {
          "enc_vel":[v_fl,v_fr,v_rl,v_rr],
          "imu_quat":[x,y,z,w],
          "imu_gyro":[gx,gy,gz],
          "imu_lin_acc":[ax,ay,az]
        }
        plus sometimes {"info":"..."} lines.
        """
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                self.open_serial_with_retry()

            try:
                line = self.ser.readline()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}. Reconnecting...')
                self.close_serial()
                time.sleep(1.0)
                continue

            if not line:
                continue

            try:
                text = line.decode('utf-8', errors='ignore').strip()
                if not text:
                    continue

                # Skip non-JSON garbage (boot logs, etc.)
                if not text.startswith('{'):
                    continue

                data = json.loads(text)
            except Exception as e:
                self.get_logger().warn(f'Invalid JSON from ESP32: {e}')
                continue

            if self.publish_raw_json_enabled:
                self.publish_raw_json(text)

            # Info messages
            if 'info' in data:
                self.get_logger().info(f"ESP32 info: {data['info']}")

            # Low-level motor debug from esp32_main.
            if 'debug' in data:
                self.publish_esp32_debug(data['debug'])

            # Encoders
            if 'enc_vel' in data:
                self.publish_enc_vel(data['enc_vel'])

            # IMU (only if fields present)
            if any(k in data for k in ('imu_quat', 'imu_gyro', 'imu_lin_acc')):
                self.publish_imu(data)

            # Ultrasonic (accept several formats)
            # Format A: {"ultra":[left,middle,right]}
            if 'ultra' in data:
                self.publish_ultrasonic_triplet(data['ultra'])

    def publish_raw_json(self, text: str):
        msg = String()
        msg.data = text
        self.pub_esp32_raw_json.publish(msg)

    def publish_esp32_debug(self, debug_data):
        msg = String()
        try:
            msg.data = json.dumps(debug_data, separators=(',', ':'))
        except (TypeError, ValueError) as e:
            self.get_logger().warn(f'Invalid ESP32 debug data: {e}')
            return
        self.pub_esp32_debug.publish(msg)

    def publish_enc_vel(self, enc_list):
        if not isinstance(enc_list, list):
            self.get_logger().warn('enc_vel is not a list from ESP32')
            return

        try:
            values = [float(x) for x in enc_list]
        except (TypeError, ValueError) as e:
            self.get_logger().warn(f'Invalid enc_vel format from ESP32: {e}')
            return

        values = values[:4]
        if not self.accept_enc_vel(values):
            return

        self.last_enc_vel = values
        msg = Float32MultiArray()
        msg.data = values
        self.pub_enc_vel.publish(msg)

    def accept_enc_vel(self, values):
        if len(values) < 4:
            self.get_logger().warn('enc_vel must contain at least 4 wheel values')
            return False

        reason = None
        for idx, value in enumerate(values):
            if not math.isfinite(value):
                reason = f'wheel {idx + 1} is not finite: {value}'
                break
            if self.enc_vel_max_abs_mps > 0.0 and abs(value) > self.enc_vel_max_abs_mps:
                reason = (
                    f'wheel {idx + 1} abs velocity {value:.3f} m/s exceeds '
                    f'{self.enc_vel_max_abs_mps:.3f} m/s'
                )
                break
            if (
                self.last_enc_vel is not None and
                self.enc_vel_max_step_mps > 0.0 and
                abs(value - self.last_enc_vel[idx]) > self.enc_vel_max_step_mps
            ):
                reason = (
                    f'wheel {idx + 1} velocity step '
                    f'{self.last_enc_vel[idx]:.3f}->{value:.3f} m/s exceeds '
                    f'{self.enc_vel_max_step_mps:.3f} m/s'
                )
                break

        if reason is None:
            return True

        now = time.monotonic()
        if now - self.last_enc_reject_warn_time >= 2.0:
            self.last_enc_reject_warn_time = now
            self.get_logger().warn(f'Dropped invalid enc_vel sample: {reason}')
        return False

    def publish_imu(self, data: dict):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        try:
            # Orientation
            if 'imu_quat' in data:
                q = data['imu_quat']
                if isinstance(q, list) and len(q) == 4:
                    imu_msg.orientation.x = float(q[0])
                    imu_msg.orientation.y = float(q[1])
                    imu_msg.orientation.z = float(q[2])
                    imu_msg.orientation.w = float(q[3])

            # Angular velocity
            if 'imu_gyro' in data:
                g = data['imu_gyro']
                if isinstance(g, list) and len(g) == 3:
                    imu_msg.angular_velocity.x = float(g[0])
                    imu_msg.angular_velocity.y = float(g[1])
                    imu_msg.angular_velocity.z = float(g[2])

            # Linear acceleration
            if 'imu_lin_acc' in data:
                a = data['imu_lin_acc']
                if isinstance(a, list) and len(a) == 3:
                    imu_msg.linear_acceleration.x = float(a[0])
                    imu_msg.linear_acceleration.y = float(a[1])
                    imu_msg.linear_acceleration.z = float(a[2])

        except (TypeError, ValueError) as e:
            self.get_logger().warn(f'Invalid IMU data from ESP32: {e}')
            return

        self.pub_imu.publish(imu_msg)

    def publish_ultrasonic_triplet(self, arr):
        if not isinstance(arr, list) or len(arr) != 3:
            self.get_logger().warn('Ultrasonic data is not [L,M,R]')
            return

        try:
            l = float(arr[0])
            m = float(arr[1])
            r = float(arr[2])
        except (TypeError, ValueError):
            self.get_logger().warn('Invalid ultrasonic values')
            return

        now = self.get_clock().now().to_msg()

        def mk_range(frame_id, val_m):
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = float(self.ultra_fov_rad)
            msg.min_range = float(self.ultra_min_m)
            msg.max_range = float(self.ultra_max_m)
            # clamp
            if val_m < msg.min_range:
                val_m = msg.min_range
            if val_m > msg.max_range:
                val_m = msg.max_range
            msg.range = float(val_m)
            return msg

        self.pub_ultra_left.publish(mk_range(self.ultra_frame_left, l))
        self.pub_ultra_middle.publish(mk_range(self.ultra_frame_middle, m))
        self.pub_ultra_right.publish(mk_range(self.ultra_frame_right, r))



def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
