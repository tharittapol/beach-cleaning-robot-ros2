#include <Arduino.h>
#include <ArduinoJson.h>
#include <algorithm>

#include "initial.h"
#include "imu.h"
#include "ultrasonic.h"
#include "motor.h"
#include "vibration.h"
#include "pid.h"

static constexpr int WHEEL_COUNT = 4;

static constexpr uint32_t JETSON_BAUD = 230400;
static constexpr uint32_t ENC_BAUD = 115200;
static constexpr int ENC_UART_RX_PIN = 16;
static constexpr int ENC_UART_TX_PIN = 17;

static constexpr float CONTROL_HZ = 100.0f;
static constexpr uint32_t CMD_TIMEOUT_MS = 1000;
static constexpr uint32_t ENC_TIMEOUT_MS = 300;
static constexpr uint32_t WHEEL_TEST_ENC_TIMEOUT_MS = 1000;

static constexpr bool ENABLE_WHEEL[WHEEL_COUNT] = { true, true, true, true };
static bool USE_CLOSED_LOOP[WHEEL_COUNT] = { true, true, true, true };

static constexpr float V_CMD_DEADBAND = 0.005f;
static constexpr float U_MIN = -1.0f;
static constexpr float U_MAX = 1.0f;
static float LOW_SPEED_TARGET_MAX_MPS[WHEEL_COUNT] = { 0.60f, 0.60f, 0.60f, 0.60f };
static float ACTIVE_U_FLOOR[WHEEL_COUNT] = { 0.12f, 0.12f, 0.06f, 0.06f }; // front 0.22->0.12: sand low-speed stick-slip fix, 2026-06-09
static constexpr float ACTIVE_FLOOR_DISABLE_ABOVE_TARGET_RATIO = 0.75f;
static float SPIN_U_FLOOR[WHEEL_COUNT] = { 0.40f, 0.40f, 0.14f, 0.15f };
static float SPIN_HOLD_U_FLOOR_POS[WHEEL_COUNT] = { 0.36f, 0.36f, 0.10f, 0.12f };
static float SPIN_HOLD_U_FLOOR_NEG[WHEEL_COUNT] = { 0.34f, 0.34f, 0.09f, 0.11f };
static float TURN_U_FLOOR_POS[WHEEL_COUNT] = { 0.38f, 0.38f, 0.22f, 0.22f };
static float TURN_U_FLOOR_NEG[WHEEL_COUNT] = { 0.16f, 0.16f, 0.18f, 0.18f };
static constexpr uint32_t SPIN_START_BOOST_MS = 650;
static constexpr float SPIN_FLOOR_DISABLE_ABOVE_TARGET_RATIO = 1.75f;
static constexpr float IN_PLACE_LINEAR_MEAN_MAX_MPS = 0.120f;
static constexpr float MOVING_TURN_DIFF_MIN_MPS = 0.06f;
static constexpr float TURN_FLOOR_DISABLE_ABOVE_TARGET_RATIO = 1.25f;
static constexpr float CONTROL_ENCODER_MAX_MPS[WHEEL_COUNT] = { 2.0f, 2.0f, 2.0f, 2.0f };
static constexpr float CONTROL_ENCODER_OVERSPEED_GAIN = 4.0f;
static constexpr float CONTROL_ENCODER_OVERSPEED_MARGIN_MPS = 0.35f;

static float V_MAX_MPS[WHEEL_COUNT] = { 1.25f, 1.10f, 9.70f, 8.60f }; // lifted 0.17 m/s response tune, 2026-04-20

static int ENC_SIGN[WHEEL_COUNT] = { +1, +1, +1, +1 };
static int MOTOR_SIGN[WHEEL_COUNT] = { +1, +1, +1, +1 };
static constexpr float ENC_VEL_LPF_ALPHA = 0.35f;

// wheels on air at battery 25.9 V, tune each wheel step, 2026-03-21
static float PID_KP[WHEEL_COUNT] = { 0.15f, 0.18f, 0.04f, 0.03f };
static float PID_KI[WHEEL_COUNT] = { 0.08f, 0.08f, 0.05f, 0.05f }; // raised for sand hill/sustained-load climb (rears were 0), 2026-06-09
static float PID_KD[WHEEL_COUNT] = { 0.00f, 0.00f, 0.00f, 0.00f };

static uint32_t TELEMETRY_PERIOD_MS = 100;
static bool DEBUG_TO_USB = false;
static uint32_t DEBUG_PERIOD_MS = 200;

static PID pid[WHEEL_COUNT];

static float v_cmd[WHEEL_COUNT] = {0, 0, 0, 0};
static float enc_vel_raw[WHEEL_COUNT] = {0, 0, 0, 0};
static float enc_vel_corr[WHEEL_COUNT] = {0, 0, 0, 0};
static int64_t wheel_cnt[WHEEL_COUNT] = {0, 0, 0, 0};

static float u_ff[WHEEL_COUNT] = {0, 0, 0, 0};
static float u_pid[WHEEL_COUNT] = {0, 0, 0, 0};
static float u_out[WHEEL_COUNT] = {0, 0, 0, 0};
static float u_send[WHEEL_COUNT] = {0, 0, 0, 0};

static uint32_t last_cmd_ms = 0;
static uint32_t last_enc_ms = 0;
static uint32_t last_ctrl_us = 0;
static uint32_t last_telemetry_ms = 0;
static uint32_t last_debug_ms = 0;
static bool last_in_place_turn = false;
static uint32_t in_place_turn_started_ms = 0;
static uint32_t usb_json_parse_error_count = 0;
static uint32_t wheel_cmd_rx_count = 0;
static uint32_t wheel_cmd_rx_interval_ms = 0;
static int32_t wheel_cmd_rx_seq = -1;
static int32_t wheel_cmd_rx_seq_gap = 0;
static uint32_t last_wheel_cmd_rx_ms = 0;

struct WheelTestState {
  bool active = false;
  bool direct_u = false;
  bool closed_loop = true;
  int wheel = -1;
  float target = 0.0f;
  uint32_t until_ms = 0;
};

static WheelTestState wheel_test;

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float safeFloat(float x) {
  if (!isfinite(x)) return 0.0f;
  if (fabsf(x) < 0.0001f) return 0.0f;
  return x;
}

static inline bool validWheelIndex(int idx) {
  return idx >= 0 && idx < WHEEL_COUNT;
}

static int signFromCommand(float value) {
  if (fabsf(value) < V_CMD_DEADBAND) return 0;
  return (value > 0.0f) ? +1 : -1;
}

static bool isInPlaceTurnCommand() {
  if (wheel_test.active) return false;

  const float left_cmd = 0.5f * (v_cmd[0] + v_cmd[2]);
  const float right_cmd = 0.5f * (v_cmd[1] + v_cmd[3]);
  const int s_left = signFromCommand(left_cmd);
  const int s_right = signFromCommand(right_cmd);
  if (s_left == 0 || s_right == 0) return false;

  const bool sides_opposite = (s_left == -s_right);
  const float linear_mean = 0.25f * (v_cmd[0] + v_cmd[1] + v_cmd[2] + v_cmd[3]);

  // Creep-pivot intentionally keeps the inside rear wheel near zero to reduce
  // scrub. Detect spin from side averages instead of requiring every wheel to
  // have a strong sign, otherwise the ESP32 falls back to moving-turn floors.
  return sides_opposite &&
         fabsf(linear_mean) <= IN_PLACE_LINEAR_MEAN_MAX_MPS;
}

static bool isMovingTurnCommand() {
  if (wheel_test.active || isInPlaceTurnCommand()) return false;

  const float left_cmd = 0.5f * (v_cmd[0] + v_cmd[2]);
  const float right_cmd = 0.5f * (v_cmd[1] + v_cmd[3]);
  const float mean_cmd_mag =
    0.25f * (fabsf(v_cmd[0]) + fabsf(v_cmd[1]) + fabsf(v_cmd[2]) + fabsf(v_cmd[3]));

  if (mean_cmd_mag < V_CMD_DEADBAND) return false;
  return fabsf(right_cmd - left_cmd) >= MOVING_TURN_DIFF_MIN_MPS;
}

static float spinHoldFloorForCommand(int idx, float cmd_target) {
  if (!validWheelIndex(idx)) return 0.0f;
  return (cmd_target >= 0.0f) ? SPIN_HOLD_U_FLOOR_POS[idx] : SPIN_HOLD_U_FLOOR_NEG[idx];
}

static float turnFloorForCommand(int idx, float cmd_target) {
  if (!validWheelIndex(idx)) return 0.0f;
  return (cmd_target >= 0.0f) ? TURN_U_FLOOR_POS[idx] : TURN_U_FLOOR_NEG[idx];
}

static bool shouldApplyMovingTurnFloor(int idx) {
  if (!validWheelIndex(idx)) return false;
  if (!isMovingTurnCommand()) return false;

  const float left_cmd = 0.5f * (v_cmd[0] + v_cmd[2]);
  const float right_cmd = 0.5f * (v_cmd[1] + v_cmd[3]);
  const float mean_cmd = 0.25f * (v_cmd[0] + v_cmd[1] + v_cmd[2] + v_cmd[3]);
  const bool left_side = (idx == 0 || idx == 2);

  // Assist the higher-demand/outside side. Do not boost the slow inside side,
  // otherwise a curve command turns into nearly-straight drive.
  const bool left_is_outer = fabsf(left_cmd) > fabsf(right_cmd);
  if (left_side == left_is_outer) return true;

  // If the mixer explicitly asks an inside wheel to reverse/brake against the
  // main travel direction, give that small corrective command enough authority.
  return signFromCommand(v_cmd[idx]) != 0 &&
         signFromCommand(mean_cmd) != 0 &&
         signFromCommand(v_cmd[idx]) != signFromCommand(mean_cmd);
}

static float applyDriveFloor(
    int idx,
    bool in_place_turn,
    bool moving_turn,
    bool moving_turn_floor,
    bool spin_start_boost,
    float cmd_target,
    float measured,
    float u_cmd) {
  if (!validWheelIndex(idx)) return u_cmd;

  const float target_mag = fabsf(cmd_target);
  // Straight joystick driving needs a small crawl floor to break static
  // friction. During moving turns, only apply floors to the assisted side so
  // the slow inside side does not get boosted into a straight-drive command.
  float floor_u = (!moving_turn || moving_turn_floor) ? ACTIVE_U_FLOOR[idx] : 0.0f;
  float disable_ratio = ACTIVE_FLOOR_DISABLE_ABOVE_TARGET_RATIO;
  if (in_place_turn) {
    const float spin_floor =
      spin_start_boost ? SPIN_U_FLOOR[idx] : spinHoldFloorForCommand(idx, cmd_target);
    if (spin_floor > floor_u) floor_u = spin_floor;
    disable_ratio = SPIN_FLOOR_DISABLE_ABOVE_TARGET_RATIO;
  } else if (moving_turn && moving_turn_floor) {
    const float turn_floor = turnFloorForCommand(idx, cmd_target);
    if (turn_floor > floor_u) floor_u = turn_floor;
    disable_ratio = TURN_FLOOR_DISABLE_ABOVE_TARGET_RATIO;
  }

  if (floor_u <= 0.0f) return u_cmd;
  if (target_mag < V_CMD_DEADBAND) return u_cmd;
  if (!in_place_turn && !moving_turn && target_mag > LOW_SPEED_TARGET_MAX_MPS[idx]) return u_cmd;

  // Keep the high spin breakaway floor active only briefly. After the wheels
  // have started moving, allow PID to reduce power when a wheel is overspeeding.
  if (!spin_start_boost &&
      (measured * cmd_target) > 0.0f &&
      fabsf(measured) >= (target_mag * disable_ratio)) {
    return u_cmd;
  }

  // Do not override a braking/corrective output from the PID.
  if ((u_cmd * cmd_target) < 0.0f) {
    return u_cmd;
  }

  if (fabsf(u_cmd) < floor_u) {
    return copysignf(floor_u, cmd_target);
  }

  return u_cmd;
}

static float encoderFeedbackForControl(int idx, float cmd_target) {
  if (!validWheelIndex(idx)) return 0.0f;

  float measured = enc_vel_corr[idx];
  if (!isfinite(measured)) return 0.0f;

  float cap = CONTROL_ENCODER_MAX_MPS[idx];
  if (fabsf(cmd_target) >= V_CMD_DEADBAND) {
    const float target_cap =
      fabsf(cmd_target) * CONTROL_ENCODER_OVERSPEED_GAIN +
      CONTROL_ENCODER_OVERSPEED_MARGIN_MPS;
    cap = std::min(cap, target_cap);
  }

  return clampf(measured, -cap, cap);
}

struct LineReader {
  size_t n = 0;
  uint32_t overflow_count = 0;

  bool read(Stream &s, char *buf, size_t cap) {
    while (s.available()) {
      char c = (char)s.read();
      if (c == '\r') continue;
      if (c == '\n') {
        buf[n] = '\0';
        n = 0;
        return true;
      }
      if (n + 1 < cap) {
        buf[n++] = c;
      } else {
        n = 0;
        ++overflow_count;
      }
    }
    return false;
  }
};

static LineReader lr_usb;
static LineReader lr_enc;

static void publishDebug(bool full = false);

static void setPidGains(int idx, float kp, float ki, float kd) {
  if (!validWheelIndex(idx)) return;
  PID_KP[idx] = kp;
  PID_KI[idx] = ki;
  PID_KD[idx] = kd;
  pid[idx].setGains(PID_KP[idx], PID_KI[idx], PID_KD[idx]);
  pid[idx].reset();
}

static void setClosedLoop(int idx, bool enable) {
  if (!validWheelIndex(idx)) return;
  USE_CLOSED_LOOP[idx] = enable;
  pid[idx].reset();
}

static void setVmax(int idx, float vmax_mps) {
  if (!validWheelIndex(idx)) return;
  if (!isfinite(vmax_mps) || vmax_mps <= 0.01f) return;
  V_MAX_MPS[idx] = vmax_mps;
}

static void setActiveUFloor(int idx, float floor_u) {
  if (!validWheelIndex(idx)) return;
  if (!isfinite(floor_u)) return;
  ACTIVE_U_FLOOR[idx] = clampf(floor_u, 0.0f, 1.0f);
}

static void setTelemetryPeriodMs(uint32_t ms) {
  if (ms < 50) ms = 50;
  if (ms > 1000) ms = 1000;
  TELEMETRY_PERIOD_MS = ms;
}

static void setLowSpeedTargetMax(int idx, float target_mps) {
  if (!validWheelIndex(idx)) return;
  if (!isfinite(target_mps)) return;
  LOW_SPEED_TARGET_MAX_MPS[idx] = clampf(target_mps, 0.0f, 5.0f);
}

static void setSpinUFloor(int idx, float floor_u) {
  if (!validWheelIndex(idx)) return;
  if (!isfinite(floor_u)) return;
  SPIN_U_FLOOR[idx] = clampf(floor_u, 0.0f, 1.0f);
}

static void setSpinHoldUFloor(int idx, float floor_u) {
  if (!validWheelIndex(idx)) return;
  if (!isfinite(floor_u)) return;
  const float clamped = clampf(floor_u, 0.0f, 1.0f);
  SPIN_HOLD_U_FLOOR_POS[idx] = clamped;
  SPIN_HOLD_U_FLOOR_NEG[idx] = clamped;
}

static void setSpinHoldUFloorPos(int idx, float floor_u) {
  if (!validWheelIndex(idx)) return;
  if (!isfinite(floor_u)) return;
  SPIN_HOLD_U_FLOOR_POS[idx] = clampf(floor_u, 0.0f, 1.0f);
}

static void setSpinHoldUFloorNeg(int idx, float floor_u) {
  if (!validWheelIndex(idx)) return;
  if (!isfinite(floor_u)) return;
  SPIN_HOLD_U_FLOOR_NEG[idx] = clampf(floor_u, 0.0f, 1.0f);
}

static void setTurnUFloor(int idx, float floor_u) {
  if (!validWheelIndex(idx)) return;
  if (!isfinite(floor_u)) return;
  const float clamped = clampf(floor_u, 0.0f, 1.0f);
  TURN_U_FLOOR_POS[idx] = clamped;
  TURN_U_FLOOR_NEG[idx] = clamped;
}

static void setTurnUFloorPos(int idx, float floor_u) {
  if (!validWheelIndex(idx)) return;
  if (!isfinite(floor_u)) return;
  TURN_U_FLOOR_POS[idx] = clampf(floor_u, 0.0f, 1.0f);
}

static void setTurnUFloorNeg(int idx, float floor_u) {
  if (!validWheelIndex(idx)) return;
  if (!isfinite(floor_u)) return;
  TURN_U_FLOOR_NEG[idx] = clampf(floor_u, 0.0f, 1.0f);
}

static void setSignedConfig(int idx, int *dst, int sign_value) {
  if (!validWheelIndex(idx)) return;
  dst[idx] = (sign_value >= 0) ? +1 : -1;
}

static bool parseWheelSelector(const char *token, int &wheel_idx, bool &all) {
  if (strcmp(token, "ALL") == 0 || strcmp(token, "all") == 0) {
    all = true;
    wheel_idx = -1;
    return true;
  }

  const int wheel = atoi(token);
  if (wheel < 1 || wheel > WHEEL_COUNT) return false;

  all = false;
  wheel_idx = wheel - 1;
  return true;
}

static void printHelp() {
  Serial.println("Wheel-control text commands:");
  Serial.println("  DBG 1");
  Serial.println("  DBGRATE 100");
  Serial.println("  STATUS");
  Serial.println("  STOP");
  Serial.println("  PID <wheel|ALL> <kp> <ki> <kd>");
  Serial.println("  VMAX <wheel|ALL> <mps>");
  Serial.println("  CLOSED <wheel|ALL> <0|1>");
  Serial.println("  ENC_SIGN <wheel|ALL> <-1|1>");
  Serial.println("  MOTOR_SIGN <wheel|ALL> <-1|1>");
  Serial.println("  PWM_START <wheel|ALL> <0..255>");
  Serial.println("  PWM_HOLD <wheel|ALL> <0..255>");
  Serial.println("  STEP <wheel> <target_mps> <duration_ms> [closed_loop]");
  Serial.println("  OPEN <wheel> <motor_u> <duration_ms>");
}

static void sendBuzzerDurationToEnc(float duration_sec) {
  if (!isfinite(duration_sec)) return;
  if (duration_sec < 0.0f) duration_sec = 0.0f;
  Serial2.printf("B:%.3f\n", duration_sec);
}

static void stopAll() {
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    v_cmd[i] = 0.0f;
    u_ff[i] = 0.0f;
    u_pid[i] = 0.0f;
    u_out[i] = 0.0f;
    u_send[i] = 0.0f;
    pid[i].reset();
  }
  set_motor_speeds(0.0f, 0.0f, 0.0f, 0.0f);
}

static void clearWheelTest(bool stop_motors) {
  wheel_test = WheelTestState{};
  if (stop_motors) stopAll();
}

static bool startWheelTest(int wheel_number, float target, uint32_t duration_ms, bool direct_u, bool closed_loop) {
  const int idx = wheel_number - 1;
  if (!validWheelIndex(idx)) return false;
  if (!ENABLE_WHEEL[idx]) return false;
  if (duration_ms == 0 || !isfinite(target)) return false;

  clearWheelTest(true);
  wheel_test.active = true;
  wheel_test.direct_u = direct_u;
  wheel_test.closed_loop = closed_loop;
  wheel_test.wheel = idx;
  wheel_test.target = direct_u ? clampf(target, U_MIN, U_MAX) : target;
  wheel_test.until_ms = millis() + duration_ms;
  last_cmd_ms = millis();
  return true;
}

static bool controlNeedsEncoderFeedback() {
  if (wheel_test.active) {
    return (!wheel_test.direct_u) && wheel_test.closed_loop && (fabsf(wheel_test.target) >= V_CMD_DEADBAND);
  }

  for (int i = 0; i < WHEEL_COUNT; ++i) {
    if (!ENABLE_WHEEL[i]) continue;
    if (!USE_CLOSED_LOOP[i]) continue;
    if (fabsf(v_cmd[i]) < V_CMD_DEADBAND) continue;
    return true;
  }

  return false;
}

static bool handleTextCmd(const char *line) {
  if (strncmp(line, "DBGRATE", 7) == 0) {
    const char *p = line + 7;
    while (*p == ' ') ++p;
    int ms = atoi(p);
    if (ms < 50) ms = 50;
    if (ms > 5000) ms = 5000;
    DEBUG_PERIOD_MS = (uint32_t)ms;
    return true;
  }

  if (strncmp(line, "DBG", 3) == 0) {
    const char *p = line + 3;
    while (*p == ' ') ++p;
    DEBUG_TO_USB = (atoi(p) != 0);
    return true;
  }

  if (strcmp(line, "HELP") == 0) {
    printHelp();
    return true;
  }

  if (strcmp(line, "STATUS") == 0) {
    publishDebug(true);
    return true;
  }

  if (strcmp(line, "STOP") == 0) {
    clearWheelTest(true);
    return true;
  }

  char selector[16] = {0};
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;
  int enable = 0;
  int sign_value = 1;
  int pwm_value = 0;
  int wheel = 0;
  unsigned long duration_ms = 0;

  if (sscanf(line, "PID %15s %f %f %f", selector, &a, &b, &c) == 4) {
    int idx = -1;
    bool all = false;
    if (!parseWheelSelector(selector, idx, all)) return true;
    if (all) {
      for (int i = 0; i < WHEEL_COUNT; ++i) setPidGains(i, a, b, c);
    } else {
      setPidGains(idx, a, b, c);
    }
    return true;
  }

  if (sscanf(line, "VMAX %15s %f", selector, &a) == 2) {
    int idx = -1;
    bool all = false;
    if (!parseWheelSelector(selector, idx, all)) return true;
    if (all) {
      for (int i = 0; i < WHEEL_COUNT; ++i) setVmax(i, a);
    } else {
      setVmax(idx, a);
    }
    return true;
  }

  if (sscanf(line, "CLOSED %15s %d", selector, &enable) == 2) {
    int idx = -1;
    bool all = false;
    if (!parseWheelSelector(selector, idx, all)) return true;
    if (all) {
      for (int i = 0; i < WHEEL_COUNT; ++i) setClosedLoop(i, enable != 0);
    } else {
      setClosedLoop(idx, enable != 0);
    }
    return true;
  }

  if (sscanf(line, "ENC_SIGN %15s %d", selector, &sign_value) == 2) {
    int idx = -1;
    bool all = false;
    if (!parseWheelSelector(selector, idx, all)) return true;
    if (all) {
      for (int i = 0; i < WHEEL_COUNT; ++i) setSignedConfig(i, ENC_SIGN, sign_value);
    } else {
      setSignedConfig(idx, ENC_SIGN, sign_value);
    }
    return true;
  }

  if (sscanf(line, "MOTOR_SIGN %15s %d", selector, &sign_value) == 2) {
    int idx = -1;
    bool all = false;
    if (!parseWheelSelector(selector, idx, all)) return true;
    if (all) {
      for (int i = 0; i < WHEEL_COUNT; ++i) setSignedConfig(i, MOTOR_SIGN, sign_value);
    } else {
      setSignedConfig(idx, MOTOR_SIGN, sign_value);
    }
    return true;
  }

  if (sscanf(line, "PWM_START %15s %d", selector, &pwm_value) == 2) {
    int idx = -1;
    bool all = false;
    if (!parseWheelSelector(selector, idx, all)) return true;
    if (all) {
      for (int i = 0; i < WHEEL_COUNT; ++i) set_motor_pwm_start(i, pwm_value);
    } else {
      set_motor_pwm_start(idx, pwm_value);
    }
    return true;
  }

  if (sscanf(line, "PWM_HOLD %15s %d", selector, &pwm_value) == 2) {
    int idx = -1;
    bool all = false;
    if (!parseWheelSelector(selector, idx, all)) return true;
    if (all) {
      for (int i = 0; i < WHEEL_COUNT; ++i) set_motor_pwm_hold(i, pwm_value);
    } else {
      set_motor_pwm_hold(idx, pwm_value);
    }
    return true;
  }

  enable = 1;
  const int step_args = sscanf(line, "STEP %d %f %lu %d", &wheel, &a, &duration_ms, &enable);
  if (step_args >= 3) {
    startWheelTest(wheel, a, (uint32_t)duration_ms, false, (step_args >= 4) ? (enable != 0) : true);
    return true;
  }

  if (sscanf(line, "OPEN %d %f %lu", &wheel, &a, &duration_ms) == 3) {
    startWheelTest(wheel, a, (uint32_t)duration_ms, true, false);
    return true;
  }

  return false;
}

static void handleJetsonLine(const char *line) {
  if (handleTextCmd(line)) return;

  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    ++usb_json_parse_error_count;
    return;
  }

  if (doc.containsKey("stop") && doc["stop"].as<bool>()) {
    clearWheelTest(true);
  }

  if (doc.containsKey("wheel_cmd")) {
    if (!wheel_test.active) {
      const uint32_t now_ms = millis();
      wheel_cmd_rx_interval_ms =
        (last_wheel_cmd_rx_ms == 0) ? 0 : (now_ms - last_wheel_cmd_rx_ms);
      last_wheel_cmd_rx_ms = now_ms;
      ++wheel_cmd_rx_count;
      if (doc.containsKey("cmd_seq")) {
        const int32_t seq = doc["cmd_seq"] | -1;
        if (seq >= 0) {
          if (wheel_cmd_rx_seq >= 0) {
            const int32_t expected = wheel_cmd_rx_seq + 1;
            if (seq != expected) {
              wheel_cmd_rx_seq_gap += (seq > expected) ? (seq - expected) : 1;
            }
          }
          wheel_cmd_rx_seq = seq;
        }
      }
      JsonArray arr = doc["wheel_cmd"];
      clearWheelTest(false);
      for (int i = 0; i < WHEEL_COUNT; ++i) {
        const float val = arr[i] | 0.0f;
        v_cmd[i] = (fabsf(val) < V_CMD_DEADBAND) ? 0.0f : val;
      }
      last_cmd_ms = now_ms;
    }
  }

  if (doc.containsKey("pid_kp")) {
    JsonArray arr = doc["pid_kp"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setPidGains(i, arr[i] | PID_KP[i], PID_KI[i], PID_KD[i]);
    }
  }

  if (doc.containsKey("pid_ki")) {
    JsonArray arr = doc["pid_ki"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setPidGains(i, PID_KP[i], arr[i] | PID_KI[i], PID_KD[i]);
    }
  }

  if (doc.containsKey("pid_kd")) {
    JsonArray arr = doc["pid_kd"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setPidGains(i, PID_KP[i], PID_KI[i], arr[i] | PID_KD[i]);
    }
  }

  if (doc.containsKey("vmax_mps")) {
    JsonArray arr = doc["vmax_mps"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setVmax(i, arr[i] | V_MAX_MPS[i]);
    }
  }

  if (doc.containsKey("use_closed_loop")) {
    JsonVariant variant = doc["use_closed_loop"];
    if (variant.is<JsonArray>()) {
      JsonArray arr = variant.as<JsonArray>();
      for (int i = 0; i < WHEEL_COUNT; ++i) {
        setClosedLoop(i, arr[i] | USE_CLOSED_LOOP[i]);
      }
    } else {
      const bool enable_all = variant.as<bool>();
      for (int i = 0; i < WHEEL_COUNT; ++i) setClosedLoop(i, enable_all);
    }
  }

  if (doc.containsKey("enc_sign")) {
    JsonArray arr = doc["enc_sign"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setSignedConfig(i, ENC_SIGN, arr[i] | ENC_SIGN[i]);
    }
  }

  if (doc.containsKey("motor_sign")) {
    JsonArray arr = doc["motor_sign"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setSignedConfig(i, MOTOR_SIGN, arr[i] | MOTOR_SIGN[i]);
    }
  }

  if (doc.containsKey("pwm_start")) {
    JsonArray arr = doc["pwm_start"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      set_motor_pwm_start(i, arr[i] | get_motor_pwm_start(i));
    }
  }

  if (doc.containsKey("pwm_hold")) {
    JsonArray arr = doc["pwm_hold"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      set_motor_pwm_hold(i, arr[i] | get_motor_pwm_hold(i));
    }
  }

  if (doc.containsKey("motor_u_deadband")) {
    JsonArray arr = doc["motor_u_deadband"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      set_motor_u_deadband(i, arr[i] | get_motor_u_deadband(i));
    }
  }

  if (doc.containsKey("active_u_floor")) {
    JsonArray arr = doc["active_u_floor"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setActiveUFloor(i, arr[i] | ACTIVE_U_FLOOR[i]);
    }
  }

  if (doc.containsKey("low_speed_target_max_mps")) {
    JsonArray arr = doc["low_speed_target_max_mps"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setLowSpeedTargetMax(i, arr[i] | LOW_SPEED_TARGET_MAX_MPS[i]);
    }
  }

  if (doc.containsKey("spin_u_floor")) {
    JsonArray arr = doc["spin_u_floor"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setSpinUFloor(i, arr[i] | SPIN_U_FLOOR[i]);
    }
  }

  if (doc.containsKey("spin_hold_u_floor")) {
    JsonArray arr = doc["spin_hold_u_floor"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setSpinHoldUFloor(i, arr[i] | spinHoldFloorForCommand(i, 1.0f));
    }
  }

  if (doc.containsKey("spin_hold_u_floor_pos")) {
    JsonArray arr = doc["spin_hold_u_floor_pos"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setSpinHoldUFloorPos(i, arr[i] | SPIN_HOLD_U_FLOOR_POS[i]);
    }
  }

  if (doc.containsKey("spin_hold_u_floor_neg")) {
    JsonArray arr = doc["spin_hold_u_floor_neg"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setSpinHoldUFloorNeg(i, arr[i] | SPIN_HOLD_U_FLOOR_NEG[i]);
    }
  }

  if (doc.containsKey("turn_u_floor")) {
    JsonArray arr = doc["turn_u_floor"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setTurnUFloor(i, arr[i] | turnFloorForCommand(i, 1.0f));
    }
  }

  if (doc.containsKey("turn_u_floor_pos")) {
    JsonArray arr = doc["turn_u_floor_pos"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setTurnUFloorPos(i, arr[i] | TURN_U_FLOOR_POS[i]);
    }
  }

  if (doc.containsKey("turn_u_floor_neg")) {
    JsonArray arr = doc["turn_u_floor_neg"];
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setTurnUFloorNeg(i, arr[i] | TURN_U_FLOOR_NEG[i]);
    }
  }

  if (doc.containsKey("wheel_test")) {
    JsonObject wt = doc["wheel_test"];
    const int wheel = wt["wheel"] | 0;
    const uint32_t duration_ms = wt["duration_ms"] | 0;
    if (wt.containsKey("motor_u")) {
      startWheelTest(wheel, wt["motor_u"] | 0.0f, duration_ms, true, false);
    } else if (wt.containsKey("target_mps")) {
      startWheelTest(wheel, wt["target_mps"] | 0.0f, duration_ms, false, wt["closed_loop"] | true);
    }
  }

  if (doc.containsKey("buzzer_duration")) {
    sendBuzzerDurationToEnc(doc["buzzer_duration"]);
  }

  if (doc.containsKey("vibration_enable")) {
    set_vibration(doc["vibration_enable"]);
  }

  if (doc.containsKey("dbg_enable")) {
    DEBUG_TO_USB = doc["dbg_enable"];
  }

  if (doc.containsKey("dbg_rate_ms")) {
    uint32_t ms = doc["dbg_rate_ms"];
    if (ms < 50) ms = 50;
    if (ms > 5000) ms = 5000;
    DEBUG_PERIOD_MS = ms;
  }

  if (doc.containsKey("telemetry_period_ms")) {
    setTelemetryPeriodMs(doc["telemetry_period_ms"] | TELEMETRY_PERIOD_MS);
  }

  if (doc.containsKey("status") && doc["status"].as<bool>()) {
    publishDebug(true);
  }
}

static void handleEncLine(const char *line) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) return;

  bool ok_any = false;

  if (doc.containsKey("enc_vel") || doc.containsKey("enc_vel_mps")) {
    JsonArray arr = doc.containsKey("enc_vel") ? doc["enc_vel"] : doc["enc_vel_mps"];
    for (int i = 0; i < WHEEL_COUNT; ++i) enc_vel_raw[i] = arr[i] | 0.0f;
    ok_any = true;
  }

  if (doc.containsKey("wheel_cnt") || doc.containsKey("enc_counts")) {
    JsonArray arr = doc.containsKey("wheel_cnt") ? doc["wheel_cnt"] : doc["enc_counts"];
    for (int i = 0; i < WHEEL_COUNT; ++i) wheel_cnt[i] = arr[i] | 0;
    ok_any = true;
  }

  if (ok_any) last_enc_ms = millis();
}

static void publishTelemetry() {
  const ImuData imu_vals = get_imu_data();
  const UltrasonicData us = get_ultrasonic_data();

  StaticJsonDocument<512> doc;

  JsonArray enc_arr = doc.createNestedArray("enc_vel");
  for (int i = 0; i < WHEEL_COUNT; ++i) enc_arr.add(safeFloat(enc_vel_corr[i]));

  JsonArray quat_arr = doc.createNestedArray("imu_quat");
  quat_arr.add(safeFloat(imu_vals.qx));
  quat_arr.add(safeFloat(imu_vals.qy));
  quat_arr.add(safeFloat(imu_vals.qz));
  quat_arr.add(safeFloat(imu_vals.qw));

  JsonArray gyro_arr = doc.createNestedArray("imu_gyro");
  gyro_arr.add(safeFloat(imu_vals.gx));
  gyro_arr.add(safeFloat(imu_vals.gy));
  gyro_arr.add(safeFloat(imu_vals.gz));

  JsonArray acc_arr = doc.createNestedArray("imu_lin_acc");
  acc_arr.add(safeFloat(imu_vals.ax));
  acc_arr.add(safeFloat(imu_vals.ay));
  acc_arr.add(safeFloat(imu_vals.az));

  JsonArray us_arr = doc.createNestedArray("ultrasonic");
  us_arr.add(safeFloat(us.left));
  us_arr.add(safeFloat(us.middle));
  us_arr.add(safeFloat(us.right));

  serializeJson(doc, Serial);
  Serial.println();
}

static void publishDebug(bool full) {
  const uint32_t now_ms = millis();

  StaticJsonDocument<4096> doc;
  JsonObject dbg = doc.createNestedObject("debug");
  dbg["full"] = full ? 1 : 0;

  JsonArray cmd_arr = dbg.createNestedArray("wheel_cmd");
  for (int i = 0; i < WHEEL_COUNT; ++i) cmd_arr.add(safeFloat(v_cmd[i]));

  JsonArray corr_arr = dbg.createNestedArray("enc_vel_corr");
  for (int i = 0; i < WHEEL_COUNT; ++i) corr_arr.add(safeFloat(enc_vel_corr[i]));

  JsonArray out_arr = dbg.createNestedArray("motor_u");
  for (int i = 0; i < WHEEL_COUNT; ++i) out_arr.add(safeFloat(u_send[i]));

  if (full) {
    JsonArray raw_arr = dbg.createNestedArray("enc_vel_raw");
    for (int i = 0; i < WHEEL_COUNT; ++i) raw_arr.add(safeFloat(enc_vel_raw[i]));

    JsonArray cnt_arr = dbg.createNestedArray("wheel_cnt");
    for (int i = 0; i < WHEEL_COUNT; ++i) cnt_arr.add((long long)wheel_cnt[i]);

    JsonArray ff_arr = dbg.createNestedArray("u_ff");
    for (int i = 0; i < WHEEL_COUNT; ++i) ff_arr.add(safeFloat(u_ff[i]));

    JsonArray pid_arr = dbg.createNestedArray("u_pid");
    for (int i = 0; i < WHEEL_COUNT; ++i) pid_arr.add(safeFloat(u_pid[i]));

    JsonArray kp_arr = dbg.createNestedArray("pid_kp");
    for (int i = 0; i < WHEEL_COUNT; ++i) kp_arr.add(safeFloat(PID_KP[i]));

    JsonArray ki_arr = dbg.createNestedArray("pid_ki");
    for (int i = 0; i < WHEEL_COUNT; ++i) ki_arr.add(safeFloat(PID_KI[i]));

    JsonArray kd_arr = dbg.createNestedArray("pid_kd");
    for (int i = 0; i < WHEEL_COUNT; ++i) kd_arr.add(safeFloat(PID_KD[i]));

    JsonArray vmax_arr = dbg.createNestedArray("vmax_mps");
    for (int i = 0; i < WHEEL_COUNT; ++i) vmax_arr.add(safeFloat(V_MAX_MPS[i]));

    JsonArray loop_arr = dbg.createNestedArray("use_closed_loop");
    for (int i = 0; i < WHEEL_COUNT; ++i) loop_arr.add(USE_CLOSED_LOOP[i] ? 1 : 0);

    JsonArray enc_sign_arr = dbg.createNestedArray("enc_sign");
    for (int i = 0; i < WHEEL_COUNT; ++i) enc_sign_arr.add(ENC_SIGN[i]);

    JsonArray motor_sign_arr = dbg.createNestedArray("motor_sign");
    for (int i = 0; i < WHEEL_COUNT; ++i) motor_sign_arr.add(MOTOR_SIGN[i]);

    JsonArray pwm_start_arr = dbg.createNestedArray("pwm_start");
    for (int i = 0; i < WHEEL_COUNT; ++i) pwm_start_arr.add(get_motor_pwm_start(i));

    JsonArray pwm_hold_arr = dbg.createNestedArray("pwm_hold");
    for (int i = 0; i < WHEEL_COUNT; ++i) pwm_hold_arr.add(get_motor_pwm_hold(i));

    JsonArray motor_deadband_arr = dbg.createNestedArray("motor_u_deadband");
    for (int i = 0; i < WHEEL_COUNT; ++i) motor_deadband_arr.add(safeFloat(get_motor_u_deadband(i)));

    JsonArray active_floor_arr = dbg.createNestedArray("active_u_floor");
    for (int i = 0; i < WHEEL_COUNT; ++i) active_floor_arr.add(safeFloat(ACTIVE_U_FLOOR[i]));

    JsonArray low_speed_max_arr = dbg.createNestedArray("low_speed_target_max_mps");
    for (int i = 0; i < WHEEL_COUNT; ++i) low_speed_max_arr.add(safeFloat(LOW_SPEED_TARGET_MAX_MPS[i]));

    JsonArray spin_floor_arr = dbg.createNestedArray("spin_u_floor");
    for (int i = 0; i < WHEEL_COUNT; ++i) spin_floor_arr.add(safeFloat(SPIN_U_FLOOR[i]));

    JsonArray spin_hold_floor_arr = dbg.createNestedArray("spin_hold_u_floor");
    for (int i = 0; i < WHEEL_COUNT; ++i) spin_hold_floor_arr.add(safeFloat(SPIN_HOLD_U_FLOOR_POS[i]));

    JsonArray spin_hold_floor_pos_arr = dbg.createNestedArray("spin_hold_u_floor_pos");
    for (int i = 0; i < WHEEL_COUNT; ++i) spin_hold_floor_pos_arr.add(safeFloat(SPIN_HOLD_U_FLOOR_POS[i]));

    JsonArray spin_hold_floor_neg_arr = dbg.createNestedArray("spin_hold_u_floor_neg");
    for (int i = 0; i < WHEEL_COUNT; ++i) spin_hold_floor_neg_arr.add(safeFloat(SPIN_HOLD_U_FLOOR_NEG[i]));

    JsonArray turn_floor_arr = dbg.createNestedArray("turn_u_floor");
    for (int i = 0; i < WHEEL_COUNT; ++i) turn_floor_arr.add(safeFloat(TURN_U_FLOOR_POS[i]));

    JsonArray turn_floor_pos_arr = dbg.createNestedArray("turn_u_floor_pos");
    for (int i = 0; i < WHEEL_COUNT; ++i) turn_floor_pos_arr.add(safeFloat(TURN_U_FLOOR_POS[i]));

    JsonArray turn_floor_neg_arr = dbg.createNestedArray("turn_u_floor_neg");
    for (int i = 0; i < WHEEL_COUNT; ++i) turn_floor_neg_arr.add(safeFloat(TURN_U_FLOOR_NEG[i]));
  }

  dbg["cmd_age_ms"] = wheel_test.active ? 0 : (now_ms - last_cmd_ms);
  dbg["enc_age_ms"] = now_ms - last_enc_ms;
  dbg["cmd_rx_count"] = wheel_cmd_rx_count;
  dbg["cmd_rx_interval_ms"] = wheel_cmd_rx_interval_ms;
  dbg["cmd_seq"] = wheel_cmd_rx_seq;
  dbg["cmd_seq_gap"] = wheel_cmd_rx_seq_gap;
  dbg["usb_parse_errors"] = usb_json_parse_error_count;
  dbg["usb_line_overflows"] = lr_usb.overflow_count;
  dbg["telemetry_period_ms"] = TELEMETRY_PERIOD_MS;
  dbg["in_place_turn"] = isInPlaceTurnCommand() ? 1 : 0;
  dbg["moving_turn"] = isMovingTurnCommand() ? 1 : 0;
  dbg["spin_start_ms"] = in_place_turn_started_ms;
  dbg["spin_start_boost"] = (isInPlaceTurnCommand() &&
                            in_place_turn_started_ms != 0 &&
                            (now_ms - in_place_turn_started_ms) <= SPIN_START_BOOST_MS) ? 1 : 0;
  dbg["imu_ok"] = imu_is_ready() ? 1 : 0;
  dbg["vibration_enable"] = get_vibration_state() ? 1 : 0;

  if (full) {
    const ImuData imu_vals = get_imu_data();
    JsonArray q_arr = dbg.createNestedArray("imu_quat");
    q_arr.add(safeFloat(imu_vals.qx));
    q_arr.add(safeFloat(imu_vals.qy));
    q_arr.add(safeFloat(imu_vals.qz));
    q_arr.add(safeFloat(imu_vals.qw));
  }

  JsonObject test_obj = dbg.createNestedObject("wheel_test");
  test_obj["active"] = wheel_test.active ? 1 : 0;
  test_obj["wheel"] = wheel_test.active ? (wheel_test.wheel + 1) : 0;
  test_obj["direct_u"] = wheel_test.direct_u ? 1 : 0;
  test_obj["closed_loop"] = wheel_test.closed_loop ? 1 : 0;
  test_obj["target"] = safeFloat(wheel_test.target);
  test_obj["remaining_ms"] = wheel_test.active && (wheel_test.until_ms > now_ms) ? (wheel_test.until_ms - now_ms) : 0;

  serializeJson(doc, Serial);
  Serial.println();
}

void setup() {
  Serial.begin(JETSON_BAUD);
  Serial2.begin(ENC_BAUD, SERIAL_8N1, ENC_UART_RX_PIN, ENC_UART_TX_PIN);

  Serial.setRxBufferSize(4096);
  Serial2.setRxBufferSize(4096);

  system_begin();

  for (int i = 0; i < WHEEL_COUNT; ++i) {
    pid[i].setGains(PID_KP[i], PID_KI[i], PID_KD[i]);
    pid[i].setLimits(U_MIN, U_MAX, -0.5f, 0.5f);
    pid[i].reset();
  }

  last_cmd_ms = millis();
  last_enc_ms = millis();
  last_ctrl_us = micros();
  last_telemetry_ms = millis();
  last_debug_ms = millis();
}

void loop() {
  static char line_cmd[256];
  while (lr_usb.read(Serial, line_cmd, sizeof(line_cmd))) {
    handleJetsonLine(line_cmd);
  }

  static char line_enc[256];
  while (lr_enc.read(Serial2, line_enc, sizeof(line_enc))) {
    handleEncLine(line_enc);
  }

  imu_poll();
  ultrasonic_poll();

  for (int i = 0; i < WHEEL_COUNT; ++i) {
    const float v = (float)ENC_SIGN[i] * enc_vel_raw[i];
    enc_vel_corr[i] = ENC_VEL_LPF_ALPHA * v + (1.0f - ENC_VEL_LPF_ALPHA) * enc_vel_corr[i];
  }

  const uint32_t now_ms = millis();
  if (wheel_test.active) {
    if (now_ms >= wheel_test.until_ms) {
      clearWheelTest(true);
    } else {
      last_cmd_ms = now_ms;
    }
  }

  const uint32_t now_us = micros();
  const float dt = (now_us - last_ctrl_us) / 1000000.0f;

  if (dt >= (1.0f / CONTROL_HZ)) {
    last_ctrl_us = now_us;

    const bool cmd_ok = wheel_test.active || ((now_ms - last_cmd_ms) <= CMD_TIMEOUT_MS);
    const uint32_t enc_timeout_ms = wheel_test.active ? WHEEL_TEST_ENC_TIMEOUT_MS : ENC_TIMEOUT_MS;
    const bool enc_ok = !controlNeedsEncoderFeedback() || ((now_ms - last_enc_ms) <= enc_timeout_ms);

    if (!cmd_ok || !enc_ok) {
      stopAll();
    } else {
      const bool in_place_turn = isInPlaceTurnCommand();
      const bool moving_turn = isMovingTurnCommand();
      if (in_place_turn && !last_in_place_turn) {
        in_place_turn_started_ms = now_ms;
      } else if (!in_place_turn) {
        in_place_turn_started_ms = 0;
      }
      last_in_place_turn = in_place_turn;
      const bool spin_start_boost =
        in_place_turn &&
        in_place_turn_started_ms != 0 &&
        (now_ms - in_place_turn_started_ms) <= SPIN_START_BOOST_MS;

      for (int i = 0; i < WHEEL_COUNT; ++i) {
        if (!ENABLE_WHEEL[i]) {
          u_ff[i] = 0.0f;
          u_pid[i] = 0.0f;
          u_out[i] = 0.0f;
          u_send[i] = 0.0f;
          pid[i].reset();
          continue;
        }

        float cmd_target = v_cmd[i];
        bool use_closed_loop = USE_CLOSED_LOOP[i];
        bool direct_u = false;
        float direct_u_cmd = 0.0f;

        if (wheel_test.active) {
          if (i == wheel_test.wheel) {
            if (wheel_test.direct_u) {
              direct_u = true;
              direct_u_cmd = wheel_test.target;
            } else {
              cmd_target = wheel_test.target;
              use_closed_loop = wheel_test.closed_loop;
            }
          } else {
            cmd_target = 0.0f;
            use_closed_loop = false;
          }
        }

        if (direct_u) {
          u_ff[i] = clampf(direct_u_cmd, U_MIN, U_MAX);
          u_pid[i] = 0.0f;
          u_out[i] = u_ff[i];
          pid[i].reset();
        } else {
          const float vmax = (V_MAX_MPS[i] > 1e-3f) ? V_MAX_MPS[i] : 1.0f;
          u_ff[i] = clampf(cmd_target / vmax, U_MIN, U_MAX);
          const float measured_for_control = encoderFeedbackForControl(i, cmd_target);

          if (fabsf(cmd_target) < V_CMD_DEADBAND) {
            u_ff[i] = 0.0f;
            u_pid[i] = 0.0f;
            u_out[i] = 0.0f;
            pid[i].reset();
          } else if (use_closed_loop) {
            u_pid[i] = pid[i].update(cmd_target, measured_for_control, dt);
            u_out[i] = clampf(u_ff[i] + u_pid[i], U_MIN, U_MAX);
          } else {
            u_pid[i] = 0.0f;
            u_out[i] = u_ff[i];
            pid[i].reset();
          }

          u_out[i] = applyDriveFloor(
            i,
            in_place_turn,
            moving_turn,
            shouldApplyMovingTurnFloor(i),
            spin_start_boost,
            cmd_target,
            measured_for_control,
            u_out[i]);
        }

        u_send[i] = (float)MOTOR_SIGN[i] * u_out[i];
      }

      set_motor_speeds(u_send[0], u_send[1], u_send[2], u_send[3]);
    }
  }

  if (now_ms - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = now_ms;
    publishTelemetry();
  }

  if (DEBUG_TO_USB && (now_ms - last_debug_ms >= DEBUG_PERIOD_MS)) {
    last_debug_ms = now_ms;
    publishDebug();
  }
}
