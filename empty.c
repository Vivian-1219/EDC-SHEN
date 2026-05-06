#include "ti_msp_dl_config.h"
#include "chassis_iface.h"
#include "motor.h"

#define RUN_MOTOR_SELF_TEST 0
#define RUN_SQUARE_TEST 0
#define RUN_ENCODER_SQUARE_TEST 0
#define MOTOR_TEST_SPEED 100
#define MOTOR_TEST_RUN_MS 3000
#define MOTOR_TEST_STOP_MS 1500
#define SQUARE_DRIVE_SPEED 45
#define SQUARE_TURN_SPEED 45
#define SQUARE_FORWARD_MS 4500
#define SQUARE_TURN_RIGHT_MS 850
#define SQUARE_FORWARD_ENCODER_TICKS 1200
#define SQUARE_TURN_90_ENCODER_TICKS 420
#define SQUARE_SIDE_COUNT 4

static void delay_ms(uint32_t ms)
{
    while (ms > 0)
    {
        delay_cycles(CPUCLK_FREQ / 1000);
        ms--;
    }
}

#if RUN_MOTOR_SELF_TEST
static void run_motor_pwm_for(uint32_t ms)
{
    while (ms > 0)
    {
        motor_update_pwm();
        delay_ms(1);
        ms--;
    }
}

static void stop_between_tests(void)
{
    motor_coast();
    run_motor_pwm_for(MOTOR_TEST_STOP_MS);
}

#endif

#if RUN_SQUARE_TEST || RUN_ENCODER_SQUARE_TEST
static void run_chassis_pwm_for(uint32_t ms)
{
    while (ms > 0)
    {
        motor_update_pwm();
        delay_ms(1);
        ms--;
    }
}
#endif

#if RUN_SQUARE_TEST
static void run_square_1p5m(void)
{
    int side;

    chassis_init();
    chassis_unlock();

    for (side = 0; side < SQUARE_SIDE_COUNT; side++)
    {
        chassis_forward(SQUARE_DRIVE_SPEED);
        run_chassis_pwm_for(SQUARE_FORWARD_MS);

        chassis_brake();
        run_chassis_pwm_for(250);

        if (side < SQUARE_SIDE_COUNT - 1)
        {
            chassis_rotate_right(SQUARE_TURN_SPEED);
            run_chassis_pwm_for(SQUARE_TURN_RIGHT_MS);

            chassis_brake();
            run_chassis_pwm_for(250);
        }
    }

    chassis_brake();
}
#endif

#if RUN_ENCODER_SQUARE_TEST
static void run_encoder_square_until_forward(int target_ticks)
{
    chassis_encoder_reset();

    while (((chassis_encoder_get_left_count() +
             chassis_encoder_get_right_count()) / 2) < target_ticks)
    {
        chassis_forward(SQUARE_DRIVE_SPEED);
        motor_update_pwm();
        delay_ms(1);
    }

    chassis_brake();
}

static void run_encoder_square_until_turn_right(int target_ticks)
{
    chassis_encoder_reset();

    while (((chassis_encoder_get_left_count() +
             chassis_encoder_get_right_count()) / 2) < target_ticks)
    {
        chassis_rotate_right(SQUARE_TURN_SPEED);
        motor_update_pwm();
        delay_ms(1);
    }

    chassis_brake();
}

static void run_encoder_square_1p5m(void)
{
    int side;

    chassis_init();
    chassis_unlock();

    for (side = 0; side < SQUARE_SIDE_COUNT; side++)
    {
        run_encoder_square_until_forward(SQUARE_FORWARD_ENCODER_TICKS);
        run_chassis_pwm_for(250);

        if (side < SQUARE_SIDE_COUNT - 1)
        {
            run_encoder_square_until_turn_right(SQUARE_TURN_90_ENCODER_TICKS);
            run_chassis_pwm_for(250);
        }
    }

    chassis_brake();
}
#endif

int main(void)
{
    SYSCFG_DL_init();

#if RUN_MOTOR_SELF_TEST
    motor_init();
    motor_unlock();
    stop_between_tests();

    while (1)
    {
        motor_set_wheels(MOTOR_TEST_SPEED, MOTOR_TEST_SPEED,
                         MOTOR_TEST_SPEED, MOTOR_TEST_SPEED);
        motor_update_pwm();
        delay_ms(1);
    }
#elif RUN_SQUARE_TEST
    run_square_1p5m();

    while (1)
    {
        motor_update_pwm();
        delay_ms(1);
    }
#elif RUN_ENCODER_SQUARE_TEST
    run_encoder_square_1p5m();

    while (1)
    {
        motor_update_pwm();
        delay_ms(1);
    }
#else
    uint32_t chassis_tick = 0;

    chassis_init();
    chassis_set_target(CHASSIS_TARGET_C);
    chassis_unlock();

    while (1)
    {
        motor_update_pwm();

        chassis_tick++;
        if (chassis_tick >= 10)
        {
            chassis_tick = 0;
            chassis_run_line();
        }

        delay_ms(1);
    }
#endif
}
