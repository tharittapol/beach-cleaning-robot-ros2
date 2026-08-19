#include "pid.h"

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void PID::setGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}
void PID::setLimits(float out_min, float out_max, float i_min, float i_max) {
    _out_min = out_min; _out_max = out_max; _i_min = i_min; _i_max = i_max;
}
void PID::reset() {
    _integral = 0.0f; _prev_error = 0.0f;
}
float PID::update(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    const float error = setpoint - measured;
    const float derivative = (error - _prev_error) / dt;

    const float p_term = _kp * error;
    const float d_term = _kd * derivative;

    const float integral_candidate = clampf(_integral + (_ki * error * dt), _i_min, _i_max);
    const float output_candidate = p_term + integral_candidate + d_term;

    const bool saturating_high = (output_candidate > _out_max) && (error > 0.0f);
    const bool saturating_low = (output_candidate < _out_min) && (error < 0.0f);

    if (!saturating_high && !saturating_low) {
        _integral = integral_candidate;
    }

    _prev_error = error;

    const float output = p_term + _integral + d_term;
    return clampf(output, _out_min, _out_max);
}
