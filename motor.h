#ifndef MOTOR_H
#define MOTOR_H
void motor_init(void);
void motor_on(void);
void motor_off(void);
void motor_run(int l, int r);
void motor_stop(void);
#endif