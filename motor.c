#include "motor.h"
#include "ti_msp_dl_config.h"

void motor_init(void)
{
    DL_GPIO_initDigitalOutput(STBY_PIN);
    DL_GPIO_setPins(STBY_PORT, STBY_PIN);

    DL_GPIO_initDigitalOutput(AIN1_PIN);
    DL_GPIO_initDigitalOutput(AIN2_PIN);
    DL_GPIO_initDigitalOutput(BIN1_PIN);
    DL_GPIO_initDigitalOutput(BIN2_PIN);

    DL_GPIO_initDigitalOutput(PWMA_PIN);
    DL_GPIO_initDigitalOutput(PWMB_PIN);

    motor_stop();
}

void motor_set_speed(int left, int right)
{
    // 左电机方向控制
    if (left > 0)
    {
        DL_GPIO_setPins(AIN1_PORT, AIN1_PIN);
        DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN);
    }
    else if (left < 0)
    {
        DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN);
        DL_GPIO_setPins(AIN2_PORT, AIN2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN);
        DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN);
    }

    // 右电机方向控制
    if (right > 0)
    {
        DL_GPIO_setPins(BIN1_PORT, BIN1_PIN);
        DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN);
    }
    else if (right < 0)
    {
        DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN);
        DL_GPIO_setPins(BIN2_PORT, BIN2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN);
        DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN);
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
    // 空函数 满足接口声明 无报错
}
