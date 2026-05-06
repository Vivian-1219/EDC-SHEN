#include "motor.h"
#include "ti_msp_dl_config.h"

#define AIN1_PORT   GPIO_MOTOR_AIN1_PORT
#define AIN1_PIN    GPIO_MOTOR_AIN1_PIN

#define AIN2_PORT   GPIO_MOTOR_AIN2_PORT
#define AIN2_PIN    GPIO_MOTOR_AIN2_PIN

#define BIN1_PORT   GPIO_MOTOR_BIN1_PORT
#define BIN1_PIN    GPIO_MOTOR_BIN1_PIN

#define BIN2_PORT   GPIO_MOTOR_BIN2_PORT
#define BIN2_PIN    GPIO_MOTOR_BIN2_PIN

#define PWMA_PORT   GPIO_MOTOR_PWMA_PORT
#define PWMA_PIN    GPIO_MOTOR_PWMA_PIN

#define PWMB_PORT   GPIO_MOTOR_PWMB_PORT
#define PWMB_PIN    GPIO_MOTOR_PWMB_PIN

#define CIN1_PORT   GPIO_MOTOR_CIN1_PORT
#define CIN1_PIN    GPIO_MOTOR_CIN1_PIN

#define CIN2_PORT   GPIO_MOTOR_CIN2_PORT
#define CIN2_PIN    GPIO_MOTOR_CIN2_PIN

#define DIN1_PORT   GPIO_MOTOR_DIN1_PORT
#define DIN1_PIN    GPIO_MOTOR_DIN1_PIN

#define DIN2_PORT   GPIO_MOTOR_DIN2_PORT
#define DIN2_PIN    GPIO_MOTOR_DIN2_PIN

#define PWMC_PORT   GPIO_MOTOR_PWMC_PORT
#define PWMC_PIN    GPIO_MOTOR_PWMC_PIN

#define PWMD_PORT   GPIO_MOTOR_PWMD_PORT
#define PWMD_PIN    GPIO_MOTOR_PWMD_PIN

#define STBY_PORT   GPIO_MOTOR_STBY_PORT
#define STBY_PIN    GPIO_MOTOR_STBY_PIN

#define MOTOR_PWM_STEPS 10
#define MOTOR_A_POLARITY 1
#define MOTOR_B_POLARITY -1
#define MOTOR_C_POLARITY -1
#define MOTOR_D_POLARITY 1

static int motor_locked = 1;
static int duty_a = 0;
static int duty_b = 0;
static int duty_c = 0;
static int duty_d = 0;
static int pwm_phase = 0;

static int clamp_speed(int speed)
{
    if (speed > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) return -MOTOR_MAX_SPEED;
    return speed;
}

static void set_channel(GPIO_Regs *in1_port, uint32_t in1_pin,
                        GPIO_Regs *in2_port, uint32_t in2_pin,
                        int speed)
{
    if (speed > 0)
    {
        DL_GPIO_setPins(in1_port, in1_pin);
        DL_GPIO_clearPins(in2_port, in2_pin);
    }
    else if (speed < 0)
    {
        DL_GPIO_clearPins(in1_port, in1_pin);
        DL_GPIO_setPins(in2_port, in2_pin);
    }
    else
    {
        DL_GPIO_clearPins(in1_port, in1_pin);
        DL_GPIO_clearPins(in2_port, in2_pin);
    }
}

static void brake_channel(GPIO_Regs *in1_port, uint32_t in1_pin,
                          GPIO_Regs *in2_port, uint32_t in2_pin)
{
    DL_GPIO_setPins(in1_port, in1_pin);
    DL_GPIO_setPins(in2_port, in2_pin);
}

static void enable_driver(void)
{
    DL_GPIO_setPins(STBY_PORT, STBY_PIN);
    DL_GPIO_setPins(PWMA_PORT, PWMA_PIN);
    DL_GPIO_setPins(PWMB_PORT, PWMB_PIN);
    DL_GPIO_setPins(PWMC_PORT, PWMC_PIN);
    DL_GPIO_setPins(PWMD_PORT, PWMD_PIN);
}

static void disable_driver(void)
{
    DL_GPIO_clearPins(PWMA_PORT, PWMA_PIN);
    DL_GPIO_clearPins(PWMB_PORT, PWMB_PIN);
    DL_GPIO_clearPins(PWMC_PORT, PWMC_PIN);
    DL_GPIO_clearPins(PWMD_PORT, PWMD_PIN);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN);
}

static int speed_to_duty(int speed)
{
    int duty = clamp_speed(speed);

    if (duty < 0) duty = -duty;
    return duty;
}

static void write_pwm(GPIO_Regs *port, uint32_t pin, int duty, int phase)
{
    int high_slots = (duty + MOTOR_PWM_STEPS - 1) / MOTOR_PWM_STEPS;

    if (high_slots > phase)
    {
        DL_GPIO_setPins(port, pin);
    }
    else
    {
        DL_GPIO_clearPins(port, pin);
    }
}

void motor_init(void)
{
    enable_driver();
    motor_locked = 0;
    motor_stop();
}

void motor_set_raw(int left, int right)
{
    motor_set_wheels(left, left, right, right);
}

void motor_set_speed(int left, int right)
{
    motor_set_raw(left, right);
}

void motor_set_wheels(int left_front, int left_rear, int right_rear, int right_front)
{
    if (motor_locked)
    {
        return;
    }

    DL_GPIO_setPins(STBY_PORT, STBY_PIN);

    set_channel(AIN1_PORT, AIN1_PIN, AIN2_PORT, AIN2_PIN,
        clamp_speed(left_front) * MOTOR_A_POLARITY);
    set_channel(BIN1_PORT, BIN1_PIN, BIN2_PORT, BIN2_PIN,
        clamp_speed(left_rear) * MOTOR_B_POLARITY);
    set_channel(CIN1_PORT, CIN1_PIN, CIN2_PORT, CIN2_PIN,
        clamp_speed(right_rear) * MOTOR_C_POLARITY);
    set_channel(DIN1_PORT, DIN1_PIN, DIN2_PORT, DIN2_PIN,
        clamp_speed(right_front) * MOTOR_D_POLARITY);

    duty_a = speed_to_duty(left_front);
    duty_b = speed_to_duty(left_rear);
    duty_c = speed_to_duty(right_rear);
    duty_d = speed_to_duty(right_front);
}

void motor_update_pwm(void)
{
    if (motor_locked)
    {
        disable_driver();
        return;
    }

    DL_GPIO_setPins(STBY_PORT, STBY_PIN);

    write_pwm(PWMA_PORT, PWMA_PIN, duty_a, pwm_phase);
    write_pwm(PWMB_PORT, PWMB_PIN, duty_b, pwm_phase);
    write_pwm(PWMC_PORT, PWMC_PIN, duty_c, pwm_phase);
    write_pwm(PWMD_PORT, PWMD_PIN, duty_d, pwm_phase);

    pwm_phase++;
    if (pwm_phase >= MOTOR_PWM_STEPS)
    {
        pwm_phase = 0;
    }
}

void motor_force_forward_full(void)
{
    motor_unlock();
    motor_set_wheels(MOTOR_MAX_SPEED, MOTOR_MAX_SPEED,
        MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    motor_update_pwm();
}

void motor_coast(void)
{
    duty_a = 0;
    duty_b = 0;
    duty_c = 0;
    duty_d = 0;

    set_channel(AIN1_PORT, AIN1_PIN, AIN2_PORT, AIN2_PIN, 0);
    set_channel(BIN1_PORT, BIN1_PIN, BIN2_PORT, BIN2_PIN, 0);
    set_channel(CIN1_PORT, CIN1_PIN, CIN2_PORT, CIN2_PIN, 0);
    set_channel(DIN1_PORT, DIN1_PIN, DIN2_PORT, DIN2_PIN, 0);

    DL_GPIO_clearPins(PWMA_PORT, PWMA_PIN);
    DL_GPIO_clearPins(PWMB_PORT, PWMB_PIN);
    DL_GPIO_clearPins(PWMC_PORT, PWMC_PIN);
    DL_GPIO_clearPins(PWMD_PORT, PWMD_PIN);
}

void motor_brake(void)
{
    if (motor_locked)
    {
        return;
    }

    duty_a = MOTOR_MAX_SPEED;
    duty_b = MOTOR_MAX_SPEED;
    duty_c = MOTOR_MAX_SPEED;
    duty_d = MOTOR_MAX_SPEED;

    DL_GPIO_setPins(STBY_PORT, STBY_PIN);
    DL_GPIO_setPins(PWMA_PORT, PWMA_PIN);
    DL_GPIO_setPins(PWMB_PORT, PWMB_PIN);
    DL_GPIO_setPins(PWMC_PORT, PWMC_PIN);
    DL_GPIO_setPins(PWMD_PORT, PWMD_PIN);

    brake_channel(AIN1_PORT, AIN1_PIN, AIN2_PORT, AIN2_PIN);
    brake_channel(BIN1_PORT, BIN1_PIN, BIN2_PORT, BIN2_PIN);
    brake_channel(CIN1_PORT, CIN1_PIN, CIN2_PORT, CIN2_PIN);
    brake_channel(DIN1_PORT, DIN1_PIN, DIN2_PORT, DIN2_PIN);
}

void motor_stop(void)
{
    motor_coast();
}

void motor_lock(void)
{
    motor_brake();
    disable_driver();
    motor_locked = 1;
}

void motor_unlock(void)
{
    enable_driver();
    motor_locked = 0;
}

int motor_is_locked(void)
{
    return motor_locked;
}
