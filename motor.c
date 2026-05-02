#include "motor.h"
#include "ti_msp_dl_config.h"

#define AIN1        DL_GPIO_PIN_8
#define AIN2        DL_GPIO_PIN_26
#define BIN1        DL_GPIO_PIN_9
#define BIN2        DL_GPIO_PIN_19

void motor_init(void)
{
    // 初始化只传引脚
    DL_GPIO_initDigitalOutput(AIN1);
    DL_GPIO_initDigitalOutput(AIN2);
    DL_GPIO_initDigitalOutput(BIN1);
    DL_GPIO_initDigitalOutput(BIN2);

    motor_stop();
}

void motor_set_speed(int left, int right)
{
    if (left > 0)
    {
        DL_GPIO_setPins(GPIOA, AIN1);
        DL_GPIO_clearPins(GPIOA, AIN2);
    }
    else if (left < 0)
    {
        DL_GPIO_clearPins(GPIOA, AIN1);
        DL_GPIO_setPins(GPIOA, AIN2);
    }
    else
    {
        DL_GPIO_clearPins(GPIOA, AIN1);
        DL_GPIO_clearPins(GPIOA, AIN2);
    }

    if (right > 0)
    {
        DL_GPIO_setPins(GPIOA, BIN1);
        DL_GPIO_clearPins(GPIOA, BIN2);
    }
    else if (right < 0)
    {
        DL_GPIO_clearPins(GPIOA, BIN1);
        DL_GPIO_setPins(GPIOA, BIN2);
    }
    else
    {
        DL_GPIO_clearPins(GPIOA, BIN1);
        DL_GPIO_clearPins(GPIOA, BIN2);
    }
}

void motor_stop(void)
{
    motor_set_speed(0, 0);
}

void motor_lock(void)
{
    motor_stop();
}

void motor_unlock(void)
{

}
