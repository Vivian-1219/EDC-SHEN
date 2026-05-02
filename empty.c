#include "ti_msp_dl_config.h"
#include "motor.h"
#include "track.h"
#include "chassis_iface.h"

int main(void)
{
    SYSCFG_DL_init();
    motor_init();
    track_init();
    chassis_unlock();

    while(1)
    {
        if(track_is_cross())
        {
            chassis_lock();
            while(1);
        }
        else
        {
            chassis_run_line();
        }
    }
}