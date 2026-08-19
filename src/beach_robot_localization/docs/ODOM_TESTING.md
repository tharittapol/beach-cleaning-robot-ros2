# Odometry Localization Tests

This package has a side-by-side odometry test launch for comparing:

- Raw wheel encoder odometry: `/wheel/odom`
- Wheel-only EKF: `/odometry/wheel_only`
- IMU-only EKF: `/odometry/imu_only`
- Wheel + IMU EKF: `/odometry/fusion`

The test EKFs publish no TF, so all modes can run together without fighting over
`odom -> base_link`. Use the normal `localization.launch.py` for the production
TF used by Nav2.

There is also a three-sensor IMU comparison launch for choosing between the
ESP32/BNO055 IMU and the ZED Mini IMU:

- Raw wheel encoder odometry: `/wheel/odom`
- Wheel-only EKF: `/odometry/wheel_only`
- BNO055 IMU-only EKF: `/odometry/bno_imu_only`
- ZED Mini IMU-only EKF: `/odometry/zed_imu_only`
- Wheel + BNO055 EKF: `/odometry/fusion_bno`
- Wheel + ZED Mini EKF: `/odometry/fusion_zed`

All comparison EKFs publish no TF, so they can run together safely.

## Build

```bash
cd ~/beach_robot_ws
colcon build --symlink-install --packages-select beach_robot_localization
source install/setup.bash
```

## Hardware inputs

Start the ESP32 bridge in one terminal:

```bash
ros2 launch beach_robot_esp32_bridge esp32_bridge.launch.py
```

Check that the two required sensor topics are alive:

```bash
ros2 topic hz /enc_vel
ros2 topic hz /imu/data
```

## Run all odometry comparisons

```bash
ros2 launch beach_robot_localization localization_odom_test.launch.py mode:=all
```

Run one mode at a time if you want cleaner logs:

```bash
ros2 launch beach_robot_localization localization_odom_test.launch.py mode:=wheel
ros2 launch beach_robot_localization localization_odom_test.launch.py mode:=imu
ros2 launch beach_robot_localization localization_odom_test.launch.py mode:=fusion
```

Pass calibration scales while testing without editing code:

```bash
ros2 launch beach_robot_localization localization_odom_test.launch.py mode:=all \
  linear_scale:=1.0 \
  angular_scale:=1.0 \
  wheel_scale_fl:=1.0 wheel_scale_fr:=1.0 \
  wheel_scale_rl:=1.0 wheel_scale_rr:=1.0
```

## Record a bag

```bash
ros2 bag record \
  /enc_vel /imu/data /wheel/odom \
  /odometry/wheel_only /odometry/imu_only /odometry/fusion \
  /tf_static
```

## Compare BNO055 vs ZED Mini IMU

Start the ESP32 bridge first:

```bash
ros2 launch beach_robot_esp32_bridge esp32_bridge.launch.py port:=/dev/ttyESP32
```

Start the ZED Mini wrapper. The `zed_nav2_cloud_filter` launch opens the ZED
camera through `zed_wrapper`, publishes `/zed/zed_node/imu/data`, and also
starts the filtered point cloud pipeline used by Nav2:

```bash
ros2 launch zed_nav2_cloud_filter zedm_nav2_filtered.launch.py
```

Check the required inputs:

```bash
ros2 topic hz /enc_vel
ros2 topic hz /imu/data
ros2 topic hz /zed/zed_node/imu/data
ros2 topic echo /imu/data --once | grep frame_id
ros2 topic echo /zed/zed_node/imu/data --once | grep frame_id
```

Run all encoder/BNO/ZED localization candidates at the same time:

```bash
ros2 launch beach_robot_localization localization_imu_compare.launch.py
```

If the ZED IMU topic is different on the robot, override it:

```bash
ros2 launch beach_robot_localization localization_imu_compare.launch.py \
  zed_imu_topic:=/zed/zed_node/imu/data
```

Record the comparison:

```bash
mkdir -p ~/beach_robot_logs/odom_compare

ros2 bag record -o ~/beach_robot_logs/odom_compare/imu_compare_$(date +%Y%m%d_%H%M%S) \
  /enc_vel \
  /imu/data \
  /zed/zed_node/imu/data \
  /wheel/odom \
  /odometry/wheel_only \
  /odometry/bno_imu_only \
  /odometry/zed_imu_only \
  /odometry/fusion_bno \
  /odometry/fusion_zed \
  /tf_static
```

Suggested pass/fail rule: prefer the fusion output whose yaw follows real
turns without jumps, keeps sign consistent with wheel odometry, and returns
closest to the starting heading after a square or 360-degree test.

## Suggested checks

1. Static test, 30 seconds:
   `/wheel/odom`, `/odometry/wheel_only`, and `/odometry/fusion` should stay
   close to zero. `/odometry/imu_only` may drift.

2. Straight test, measured distance:
   Drive forward 1-2 m and compare x distance. Wheel-only and fusion should
   roughly match tape distance.

3. In-place yaw test:
   Rotate 90 or 360 degrees. Confirm wheel yaw sign matches IMU yaw sign.

4. Square path:
   Fusion should usually close better than wheel-only if the IMU yaw is stable.

## Calibration loop

Tune one thing at a time:

1. Straight distance:
   Drive a measured 1-2 m line. If `/wheel/odom` reports too far, reduce
   `linear_scale`; if it reports too short, increase `linear_scale`.
   Suggested formula:
   `new_linear_scale = old_linear_scale * measured_distance / odom_distance`.

2. In-place yaw:
   Rotate a measured 360 degrees. If `/wheel/odom` reports too much yaw,
   reduce `angular_scale`; if it reports too little, increase `angular_scale`.
   Suggested formula:
   `new_angular_scale = old_angular_scale * measured_yaw / odom_yaw`.

3. IMU sign:
   Rotate left by hand or with teleop. `/imu/data` yaw and `/wheel/odom` yaw
   must change in the same direction. If they disagree, fix IMU mounting/TF or
   IMU driver sign before trusting fusion.

4. Fusion check:
   Repeat a square path. `/odometry/fusion` should preserve yaw better than
   wheel-only while keeping x/y close to wheel odometry over short distances.

IMU-only x/y is diagnostic only. A low-cost IMU will drift quickly because
position comes from double-integrating acceleration.
