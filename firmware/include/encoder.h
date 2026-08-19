#pragma once
#include <Arduino.h>

// ===================== Data Structures (ดั้งเดิม) =====================
struct EncoderWheel {
  int64_t count;
  float vel_mps;
};

struct EncoderData {
  EncoderWheel w[4];
};

void encoder_begin();
void encoder_reset();
void encoder_update(float dt_s);
EncoderData encoder_get();

// ===================== เลือกโมเดลการแปลง =====================
enum WheelModel : uint8_t {
  MODEL_ROUND = 0,  // linear per rev = 2*pi*R
};

struct WheelConfig {
  int pinA;
  int pinB;
  int32_t encoder_ppr;   // RT3806 = 360
  int32_t decode_x;      // 4 recommended (x4)
  int8_t  sign;          // +1 / -1 ถ้าทิศกลับ
  float driveTeeth;
  float encoderTeeth;
  float driveRevPerOutputRev;
  WheelModel model;
  float radius_m;
  float vel_lpf_alpha;   // 0..1 (สูง=ตอบสนองไว)
};

// PCNT glitch reject (ns) เพิ่มถ้ามี noise count (เพิ่มได้ถึง 10000)
static constexpr uint32_t PCNT_GLITCH_NS = 5000;

// ===================== Helper =====================
static constexpr float PI_F = 3.14159265358979323846f;

// Track sprocket -> treat as round: Reff = (pitch * teeth) / (2*pi)
static inline float sprocket_pitch_radius(float pitch_m, int teeth) {
  return (pitch_m * (float)teeth) / (2.0f * PI_F);
}

// ===================== Encoder Config =====================

// ----- Round wheel -----
static constexpr float ROUND_WHEEL_DIAM_M = 0.3202f; // 33.02 cm
static constexpr float ROUND_WHEEL_R_M    = ROUND_WHEEL_DIAM_M * 0.5f;

// Motor MY1016Z3 gearbox ratio (ใช้เมื่อ encoder อยู่ "ก่อนเกียร์")
static constexpr float GEAR_RATIO = 9.78f; // 9.78:1

// ----- Track wheel -----
static constexpr float TRACK_PITCH_M = 0.035f; // 3.5 cm
static constexpr int   SPROCKET_OUT_TEETH = 15; // wheel sprocket teeth (output)
static constexpr int   SPROCKET_DRIVE_TEETH = 9; // motor sprocket teeth (drive)
static constexpr int   SPROCKET_ENCODER_TEETH = 9; // encoder sprocket teeth

// track effective radius (treat as round drive wheel)
static constexpr float TRACK_R_EFF_M = 0.115f; // 11.5 cm (real measure)

// ================== CONFIG 4 WHEELS ==================
static WheelConfig WCFG[4] = {
  // -------- Wheel 1 (FL) : TRACK (treat as ROUND drive sprocket) --------
  {
    .pinA=32, .pinB=33,
    .encoder_ppr=360, .decode_x=4, .sign=+1,
    .driveTeeth=(float)SPROCKET_DRIVE_TEETH, .encoderTeeth=(float)SPROCKET_ENCODER_TEETH, // 9:9 = 1:1
    .driveRevPerOutputRev=2.92708333f,      // counts_per_output_rev / counts_per_encoder_rev 4215/1440
    .model=MODEL_ROUND,
    .radius_m=TRACK_R_EFF_M,
    .vel_lpf_alpha=0.25f
  },

  // -------- Wheel 2 (FR) : TRACK (treat as ROUND drive sprocket) --------
  {
    .pinA=25, .pinB=26,
    .encoder_ppr=360, .decode_x=4, .sign=-1,
    .driveTeeth=(float)SPROCKET_DRIVE_TEETH, .encoderTeeth=(float)SPROCKET_ENCODER_TEETH,
    .driveRevPerOutputRev=2.97708333f, // 4287/1440
    .model=MODEL_ROUND,
    .radius_m=TRACK_R_EFF_M,
    .vel_lpf_alpha=0.25f
  },

  // -------- Wheel 3 (RL) : ROUND --------
  {
    .pinA=27, .pinB=14,
    .encoder_ppr=360, .decode_x=4, .sign=+1,
    .driveTeeth=1, .encoderTeeth=1,
    .driveRevPerOutputRev=1.0f, //1445/1440
    .model=MODEL_ROUND,
    .radius_m=ROUND_WHEEL_R_M,
    .vel_lpf_alpha=0.25f
  },

  // -------- Wheel 4 (RR) : ROUND --------
  {
    .pinA=39, .pinB=35,
    .encoder_ppr=360, .decode_x=4, .sign=-1,
    .driveTeeth=1, .encoderTeeth=1,
    .driveRevPerOutputRev=1.0f,
    .model=MODEL_ROUND,
    .radius_m=ROUND_WHEEL_R_M,
    .vel_lpf_alpha=0.25f
  },
};