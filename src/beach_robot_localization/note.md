ros2 launch beach_robot_localization localization_full_test.launch.py \
  use_teleop:=false \
  use_zed:=true \
  use_gnss:=true \
  esp32_port:=/dev/ttyESP32 \
  wheel_cmd_send_rate_hz:=20.0 \
  publish_raw_json:=false \
  linear_scale:=1.45 \
  angular_scale:=1.0 \
  mixer_params_file:=$HOME/beach_robot_ws/src/beach_wheel_mixer/config/mixer.yaml
-------------------------
STAMP=$(date +%Y%m%d_%H%M%S)

ros2 bag record -o ~/beach_robot_logs/report_pose/straight_10m_${STAMP} \
  /cmd_vel /wheel_cmd \
  /wheel/odom \
  /odometry/bno_imu_only /odometry/zed_imu_only \
  /odometry/fusion_bno /odometry/fusion_zed \
  /gps/fix /gps/fix_quality /odometry/gps \
  /imu/data /zed/zed_node/imu/data /tf_static
---------------------------------
ros2 run beach_robot_bringup drive_straight_odom \
  --odom-topic /odometry/fusion_bno \
  --cmd-topic /cmd_vel \
  --distance-m 10.0 \
  --speed-mps 0.15 \
  --timeout-sec 120
---------------------------
BAG=~/beach_robot_logs/report_pose/straight_10m_${STAMP}

ros2 run beach_robot_bringup localization_pose_report "$BAG" \
  --trial-id straight_10m_001 \
  --straight-distance-m 10.0 \
  --meter-step-m 1.0 \
  --out ~/beach_robot_logs/report_pose/straight_10m_summary.csv \
  --meters-out ~/beach_robot_logs/report_pose/straight_10m_every_1m.csv \
  --plot-out ~/beach_robot_logs/report_pose/straight_10m_pose.svg
