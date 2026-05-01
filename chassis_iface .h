#ifndef CHASSIS_IFACE_H
#define CHASSIS_IFACE_H
typedef enum { IDLE, FOLLOW, ARRIVE }CarState;
typedef enum { A, B, C, D }Point;

void car_lock(void);
void car_unlock(void);
void car_goto(Point p);
CarState car_get_state(void);
#endif