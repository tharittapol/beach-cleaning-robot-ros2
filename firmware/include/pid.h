#pragma once

class PID {
public:
    void setGains(float kp, float ki, float kd);
    void setLimits(float out_min, float out_max, float i_min, float i_max);
    void reset();
    float update(float setpoint, float measured, float dt);
private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;
    float _out_min = -1.0f;
    float _out_max = 1.0f;
    float _i_min = -0.5f;
    float _i_max = 0.5f;
    float _integral = 0.0f;
    float _prev_error = 0.0f;
};
