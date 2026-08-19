# Calibration

## 1) Motor direction
- Lift wheels
- Publish /cmd_vel forward
- All wheels must spin forward
- Fix using wheel_mixer params: dir_fl/dir_fr/dir_rl/dir_rr

## 2) Wheel radius
Drive 10m straight (slow).
Compare odom distance vs tape measure.
Adjust wheel_radius until error < 3%.

## 3) Front / Rear track widths
Spin in place 360 degrees.
Compare odom yaw vs real rotation.
Adjust `front_track_width` and `rear_track_width` together until yaw error < 5 degrees per 360.
For this robot, start with a narrower front track and a wider rear track.
If the rear wheels chatter or hop during spin-in-place, reduce `turn_gain_rear_in_place`
in `beach_wheel_mixer/config/mixer.yaml` before changing the geometry values.

## 4) IMU orientation
Ensure IMU axes match ROS convention.
If yaw drifts badly at standstill, check IMU mounting and biases.

## 5) GNSS usage
Use GNSS outdoors only.
Check /gps/fix covariance and update rate.
