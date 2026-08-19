// * ref: https://www.instructables.com/Tutorial-How-to-Build-Range-Detector-Using-Arduino/

#include "ultrasonic.h"
#include <Arduino.h>

static constexpr int TRIG_LEFT   = 18;
static constexpr int ECHO_LEFT   = 34;
static constexpr int TRIG_MID    = 19;
static constexpr int ECHO_MID    = 39;
static constexpr int TRIG_RIGHT  = 23;
static constexpr int ECHO_RIGHT  = 36;

static constexpr uint32_t SENSOR_GAP_MS = 25;         // rotate one sensor per poll slot
static constexpr uint32_t ECHO_TIMEOUT_US = 12000;    // ~2 m max, limits blocking
static constexpr float MAX_DISTANCE_M = 2.0f;

static UltrasonicData s_data = { -1.0f, -1.0f, -1.0f };
static uint32_t s_last_poll_ms = 0;
static int s_next_sensor = 0;

static float read_distance_m(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0f;

  const float distance_m = ((float)duration * 0.000001f * 343.0f) * 0.5f;
  if (distance_m <= 0.0f || distance_m > MAX_DISTANCE_M) return -1.0f;

  return distance_m;
}

void ultrasonic_begin() {
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_MID, OUTPUT);
  pinMode(ECHO_MID, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  digitalWrite(TRIG_LEFT, LOW);
  digitalWrite(TRIG_MID, LOW);
  digitalWrite(TRIG_RIGHT, LOW);

  s_last_poll_ms = millis();
}

void ultrasonic_poll() {
  const uint32_t now = millis();
  if (now - s_last_poll_ms < SENSOR_GAP_MS) return;
  s_last_poll_ms = now;

  if (s_next_sensor == 0) {
    s_data.left = read_distance_m(TRIG_LEFT, ECHO_LEFT);
  } else if (s_next_sensor == 1) {
    s_data.middle = read_distance_m(TRIG_MID, ECHO_MID);
  } else {
    s_data.right = read_distance_m(TRIG_RIGHT, ECHO_RIGHT);
  }

  s_next_sensor = (s_next_sensor + 1) % 3;
}

UltrasonicData get_ultrasonic_data() {
  return s_data;
}