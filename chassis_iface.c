#include "chassis_iface.h"
#include "motor.h"

static CarState s = IDLE;

void car_lock(void)
{
    motor_stop();
    motor_off();
    s = IDLE;
}

void car_unlock(void)
{
    motor_on();
}

void car_goto(Point p)
{
    s = FOLLOW;
}

CarState car_get_state(void)
{
    return s;
}