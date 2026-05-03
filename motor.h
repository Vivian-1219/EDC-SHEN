#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"

// 方向引脚
#define AIN1_PORT   GPIOA
#define AIN1_PIN    DL_GPIO_PIN_8

#define AIN2_PORT   GPIOA
#define AIN2_PIN    DL_GPIO_PIN_26

#define BIN1_PORT   GPIOB
#define BIN1_PIN    DL_GPIO_PIN_9

#define BIN2_PORT   GPIOB
#define BIN2_PIN    DL_GPIO_PIN_19

// PWM 你要的 PB8 PB16
#define PWMA_PORT   GPIOB
#define PWMA_PIN    DL_GPIO_PIN_8

#define PWMB_PORT   GPIOB
#define PWMB_PIN    DL_GPIO_PIN_16

#define STBY_PORT   GPIOA
#define STBY_PIN    DL_GPIO_PIN_15

// 函数声明
void motor_init(void);
void motor_set_speed(int left, int right);
void motor_stop(void);
void motor_lock(void);
void motor_unlock(void);

#endif
