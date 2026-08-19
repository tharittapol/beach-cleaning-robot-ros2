#ifndef MOTOR_H
#define MOTOR_H

void motor_begin();

// input = normalized control u in [-1.0, +1.0], not m/s
void set_motor_speeds(float u_fl, float u_fr, float u_rl, float u_rr);

bool set_motor_pwm_start(int wheel_index, int pwm);
bool set_motor_pwm_hold(int wheel_index, int pwm);
bool set_motor_u_deadband(int wheel_index, float deadband);
int get_motor_pwm_start(int wheel_index);
int get_motor_pwm_hold(int wheel_index);
float get_motor_u_deadband(int wheel_index);

#endif
