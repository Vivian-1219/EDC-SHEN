#ifndef MOTOR_H
#define MOTOR_H

void motor_init(void);
void motor_set_speed(int left, int right);
void motor_stop(void);
void motor_lock(void);
void motor_unlock(void);

#endif