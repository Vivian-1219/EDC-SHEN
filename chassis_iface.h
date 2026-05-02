#ifndef CHASSIS_IFACE_H
#define CHASSIS_IFACE_H

#define A 0
#define B 1
#define C 2
#define D 3

void chassis_unlock(void);
void chassis_lock(void);
void chassis_find_line(void);
void chassis_follow_target(int target);
void chassis_run_line(void);

#endif