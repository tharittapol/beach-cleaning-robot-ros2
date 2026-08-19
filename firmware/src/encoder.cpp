#include "encoder.h"
#include "driver/pcnt.h"

// ===== enable wheels (FL, FR, RL, RR) =====
static constexpr bool ENABLE_WHEEL[4] = { true, true, true, true };

// PCNT unit map (ใช้ UNIT0-UNIT3)
static const pcnt_unit_t UNIT_MAP[4] = {
  PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3
};

struct EncState {
  int64_t total = 0;
  float vel_f = 0.0f;
  bool inited = false;
  pcnt_unit_t unit;
};

static EncState S[4];

static inline float counts_per_encoder_rev(const WheelConfig& c) {
  return (float)c.encoder_ppr * (float)c.decode_x; // 360*4 = 1440
}

static inline float encoder_rev_per_drive_rev(const WheelConfig& c) {
  if (c.encoderTeeth <= 0.0f) return 1.0f;
  return c.driveTeeth / c.encoderTeeth; // 9/9 = 1
}

static inline float counts_per_output_rev(const WheelConfig& c) {
  // counts/output = counts/enc_rev * enc_rev/drive_rev * drive_rev/output_rev
  return counts_per_encoder_rev(c) * encoder_rev_per_drive_rev(c) * c.driveRevPerOutputRev;
}

static inline float meters_per_output_rev(const WheelConfig& c) {
  // MODEL_ROUND only
  return 2.0f * PI_F * c.radius_m;
}

static inline float meters_per_count(const WheelConfig& c) {
  const float cpor = counts_per_output_rev(c);
  return (cpor > 1e-6f) ? (meters_per_output_rev(c) / cpor) : 0.0f;
}

// IDF4 PCNT filter value is in APB clock cycles (0..1023), APB=80MHz => 12.5ns/cycle
static inline uint16_t glitch_cycles_from_ns(uint32_t ns) {
  uint32_t cycles = (ns + 12) / 12; // approx
  if (cycles > 1023) cycles = 1023;
  return (uint16_t)cycles;
}

static bool pcnt_begin_one(int idx) {
  S[idx].unit = UNIT_MAP[idx];

  // Channel 0: pulse=A, ctrl=B
  pcnt_config_t c0 = {};
  c0.pulse_gpio_num = WCFG[idx].pinA;
  c0.ctrl_gpio_num  = WCFG[idx].pinB;
  c0.unit           = S[idx].unit;
  c0.channel        = PCNT_CHANNEL_0;

  // Quadrature x4 (common recipe)
  c0.pos_mode   = PCNT_COUNT_DEC;      // rising
  c0.neg_mode   = PCNT_COUNT_INC;      // falling
  c0.lctrl_mode = PCNT_MODE_KEEP;      // ctrl low
  c0.hctrl_mode = PCNT_MODE_REVERSE;   // ctrl high -> reverse
  c0.counter_h_lim = 30000;
  c0.counter_l_lim = -30000;

  // Channel 1: pulse=B, ctrl=A
  pcnt_config_t c1 = c0;
  c1.pulse_gpio_num = WCFG[idx].pinB;
  c1.ctrl_gpio_num  = WCFG[idx].pinA;
  c1.channel        = PCNT_CHANNEL_1;

  c1.pos_mode = PCNT_COUNT_INC;
  c1.neg_mode = PCNT_COUNT_DEC;

  if (pcnt_unit_config(&c0) != ESP_OK) return false;
  if (pcnt_unit_config(&c1) != ESP_OK) return false;

  // Glitch filter
  pcnt_set_filter_value(S[idx].unit, glitch_cycles_from_ns(PCNT_GLITCH_NS));
  pcnt_filter_enable(S[idx].unit);

  // Start
  pcnt_counter_pause(S[idx].unit);
  pcnt_counter_clear(S[idx].unit);
  pcnt_counter_resume(S[idx].unit);

  S[idx].inited = true;
  return true;
}

static inline int32_t pcnt_read_delta_clear(int idx) {
  int16_t v = 0;
  pcnt_counter_pause(S[idx].unit);
  pcnt_get_counter_value(S[idx].unit, &v);
  pcnt_counter_clear(S[idx].unit);
  pcnt_counter_resume(S[idx].unit);
  return (int32_t)v;
}

void encoder_begin() {
  for (int i = 0; i < 4; i++) {
    if (!ENABLE_WHEEL[i]) continue;
    pinMode(WCFG[i].pinA, INPUT);
    pinMode(WCFG[i].pinB, INPUT);
  }

  for (int i = 0; i < 4; i++) {
    if (!ENABLE_WHEEL[i]) continue;

    if (!pcnt_begin_one(i)) {
      Serial.printf("PCNT init failed ENC%d\n", i + 1);
      while (true) delay(1000);
    }
  }

  encoder_reset();
}

void encoder_reset() {
  for (int i = 0; i < 4; i++) {
    S[i].total = 0;
    S[i].vel_f = 0.0f;

    if (S[i].inited) {
      pcnt_counter_clear(S[i].unit);
    }
  }
}

void encoder_update(float dt_s) {
  if (dt_s <= 0.0f) return;

  for (int i = 0; i < 4; i++) {
    if (!ENABLE_WHEEL[i] || !S[i].inited) {
      S[i].vel_f = 0.0f;
      continue;
    }

    int32_t delta = pcnt_read_delta_clear(i);
    delta *= WCFG[i].sign;
    S[i].total += (int64_t)delta;

    const float v = ((float)delta / dt_s) * meters_per_count(WCFG[i]);
    const float a = WCFG[i].vel_lpf_alpha;
    S[i].vel_f = a * v + (1.0f - a) * S[i].vel_f;
  }
}

EncoderData encoder_get() {
  EncoderData d{};
  for (int i = 0; i < 4; i++) {
    d.w[i].count = S[i].total;
    d.w[i].vel_mps = S[i].vel_f;
  }
  return d;
}