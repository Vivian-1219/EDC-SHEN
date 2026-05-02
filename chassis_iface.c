#include "chassis_iface.h"
#include "motor.h"
#include "track.h"
#include "pid.h"

void chassis_unlock(void) { motor_unlock(); }
void chassis_lock(void) { motor_lock(); }
void chassis_find_line(void) { motor_set_speed(-25, 25); }
void chassis_follow_target(int target) { (void)target; chassis_run_line(); }

void chassis_run_line(void)
{
    if(track_is_lost()) { chassis_find_line(); return; }

    int err = track_get_error();
    int out = pid_calc(err);

    int l = 50 + out;
    int r = 50 - out;

    if(l > 80) l = 80;
    if(l < -80) l = -80;
    if(r > 80) r = 80;
    if(r < -80) r = -80;

    motor_set_speed(l, r);
}