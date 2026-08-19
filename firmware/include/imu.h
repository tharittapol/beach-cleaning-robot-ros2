#ifndef IMU_H
#define IMU_H

struct ImuData {
  float qx, qy, qz, qw;   // quaternion (unitless)
  float gx, gy, gz;       // rad/s
  float ax, ay, az;       // m/s^2
  bool valid;
};

void imu_begin();
void imu_poll();
bool imu_is_ready();
ImuData get_imu_data();

#endif