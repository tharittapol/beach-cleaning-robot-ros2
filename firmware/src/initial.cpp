#include "initial.h"

#include "imu.h"
#include "ultrasonic.h"
#include "motor.h"
#include "vibration.h"

void system_begin() {
  motor_begin();
  imu_begin();
  ultrasonic_begin();
  vibration_begin();
}