#!/usr/bin/env bash
set -e
TS=$(date +"%Y%m%d_%H%M%S")
OUT=~/bags/beach_run_${TS}
mkdir -p ~/bags

ros2 bag record -o "${OUT}" \
  /cmd_vel /wheel_cmd /enc_vel /imu/data \
  /gps/fix /odometry/local /odometry/global \
  /tf /tf_static \
  /zed/filtered_cloud \
  /local_costmap/costmap /global_costmap/costmap
echo "Saved bag: ${OUT}"