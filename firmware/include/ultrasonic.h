#ifndef ULTRASONIC_H
#define ULTRASONIC_H

struct UltrasonicData {
  float left;   // meters, -1.0 = invalid / out of range
  float middle; // meters, -1.0 = invalid / out of range
  float right;  // meters, -1.0 = invalid / out of range
};

void ultrasonic_begin();
void ultrasonic_poll();
UltrasonicData get_ultrasonic_data();

#endif