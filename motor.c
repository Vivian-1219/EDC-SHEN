#include "motor.h"
#include "ti_msp_dl_config.h"

void motor_init(void)
{
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_0);
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_2);
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_3);
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_5);
    DL_GPIO_initDigitalOutput(GPIOA, DL_GPIO_PIN_6);
}

void motor_on(void) { DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_0); }
void motor_off(void) { DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_0); }
void motor_run(int l, int r) {}
void motor_stop(void) { motor_run(0, 0); }