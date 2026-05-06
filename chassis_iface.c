#include "chassis_iface.h"
#include "motor.h"
#include "track.h"
#include "pid.h"
#include "ti_msp_dl_config.h"

#if defined(__has_include)
#if __has_include("track_bridge.h")
#include "track_bridge.h"
#define CHASSIS_HAS_TRACK_BRIDGE 1
#endif
#endif

#ifndef CHASSIS_HAS_TRACK_BRIDGE
#define CHASSIS_HAS_TRACK_BRIDGE 0
#endif

#define CHASSIS_BASE_SPEED          22
#define CHASSIS_MAX_SPEED           45
#define CHASSIS_CURVE_SPEED         18
#define CHASSIS_FIND_SPEED          16
#define CHASSIS_CROSS_CONFIRM       4
#define CHASSIS_LOST_LINE_SHORT_TICKS 6
#define CHASSIS_LOST_LINE_LONG_TICKS  150
#define CHASSIS_CURVE_ERROR         420
#define CHASSIS_TRACK_ERROR_SCALE   7
#define CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD 6
#define CHASSIS_USE_ENCODER         1

#define ENCODER_SPEED_DIV           5
#define SPEED_PID_OUTPUT_LIMIT      18
#define ENCODER_E1_SIGN             1
#define ENCODER_E2_SIGN             1
#define ENCODER_E3_SIGN             1
#define ENCODER_E4_SIGN             1

static pid_t line_pid;
#if CHASSIS_USE_ENCODER
static pid_t left_speed_pid;
static pid_t right_speed_pid;
#endif
static chassis_state_t chassis_state = CHASSIS_LOCKED;
static chassis_target_t chassis_target = CHASSIS_TARGET_C;
static int last_line_error = 0;
static int lost_count = 0;
static int cross_count = 0;
static int cross_latched = 0;
static int node_count = 0;
#if CHASSIS_USE_ENCODER
static volatile int encoder_count[4] = {0, 0, 0, 0};
static int encoder_last_count[4] = {0, 0, 0, 0};
#endif

static const int target_finish_nodes[] = {
    1,  /* Target A: calibrate on real field. */
    1,  /* Target B: calibrate on real field. */
    2,  /* Target C: basic A-to-C path through center square. */
    1   /* Target D: calibrate on real field. */
};

typedef struct {
    int error;
    int line_detected;
    int all_white;
    int active_count;
    int stale;
} chassis_line_input_t;

static int abs_int(int value)
{
    return value < 0 ? -value : value;
}

static int clamp_speed(int speed)
{
    if (speed > CHASSIS_MAX_SPEED) return CHASSIS_MAX_SPEED;
    if (speed < -CHASSIS_MAX_SPEED) return -CHASSIS_MAX_SPEED;
    return speed;
}

static chassis_line_input_t chassis_read_line_input(void)
{
    chassis_line_input_t line = {0};

#if CHASSIS_HAS_TRACK_BRIDGE
    const TrackBridgeData_t *bridge = track_bridge_get();

    if (bridge == 0)
    {
        line.stale = 1;
        line.all_white = 1;
        return line;
    }

    line.error = bridge->error * CHASSIS_TRACK_ERROR_SCALE;
    line.line_detected = bridge->line_detected ? 1 : 0;
    line.all_white = bridge->all_white ? 1 : 0;
    line.active_count = bridge->active_count;
    line.stale = bridge->stale ? 1 : 0;
#else
    track_sample_t track = track_read();

    line.error = track.error;
    line.line_detected = track.black_count > 0;
    line.all_white = track.black_count == 0;
    line.active_count = track.black_count;
    line.stale = 0;
#endif

    return line;
}

#if CHASSIS_USE_ENCODER
static void encoder_add_edge(int index, GPIO_Regs *a_port, uint32_t a_pin,
                             GPIO_Regs *b_port, uint32_t b_pin, int sign)
{
    int a = DL_GPIO_readPins(a_port, a_pin) ? 1 : 0;
    int b = DL_GPIO_readPins(b_port, b_pin) ? 1 : 0;
    int step = (a == b) ? 1 : -1;

    encoder_count[index] += step * sign;
}

static void encoder_init(void)
{
    encoder_count[0] = 0;
    encoder_count[1] = 0;
    encoder_count[2] = 0;
    encoder_count[3] = 0;
    encoder_last_count[0] = 0;
    encoder_last_count[1] = 0;
    encoder_last_count[2] = 0;
    encoder_last_count[3] = 0;

    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);
}

static void encoder_reset_counts(void)
{
    __disable_irq();
    encoder_count[0] = 0;
    encoder_count[1] = 0;
    encoder_count[2] = 0;
    encoder_count[3] = 0;
    encoder_last_count[0] = 0;
    encoder_last_count[1] = 0;
    encoder_last_count[2] = 0;
    encoder_last_count[3] = 0;
    __enable_irq();
}

static void encoder_sample_side_speed(int *left_speed, int *right_speed)
{
    int count_now[4];
    int delta[4];
    int i;

    __disable_irq();
    count_now[0] = encoder_count[0];
    count_now[1] = encoder_count[1];
    count_now[2] = encoder_count[2];
    count_now[3] = encoder_count[3];
    __enable_irq();

    for (i = 0; i < 4; i++)
    {
        delta[i] = count_now[i] - encoder_last_count[i];
        encoder_last_count[i] = count_now[i];
    }

    *left_speed = (abs_int(delta[0]) + abs_int(delta[1])) / 2;
    *right_speed = (abs_int(delta[2]) + abs_int(delta[3])) / 2;
}

static int speed_pid_apply(pid_t *pid, int command, int measured_speed)
{
    int sign;
    int target_speed;
    int duty;
    int correction;

    if (command == 0)
    {
        pid_reset(pid);
        return 0;
    }

    sign = command > 0 ? 1 : -1;
    target_speed = abs_int(command) / ENCODER_SPEED_DIV;
    if (target_speed < 1)
    {
        target_speed = 1;
    }

    correction = pid_update(pid, target_speed - measured_speed);
    duty = abs_int(command) + correction;

    if (duty > CHASSIS_MAX_SPEED) duty = CHASSIS_MAX_SPEED;
    if (duty < 0) duty = 0;

    return duty * sign;
}

static void chassis_apply_speed(int left, int right)
{
    int left_measured;
    int right_measured;

    left = clamp_speed(left);
    right = clamp_speed(right);

    encoder_sample_side_speed(&left_measured, &right_measured);
    left = speed_pid_apply(&left_speed_pid, left, left_measured);
    right = speed_pid_apply(&right_speed_pid, right, right_measured);

    motor_set_speed(left, right);
}
#else
static void encoder_reset_counts(void)
{
}

static void chassis_apply_speed(int left, int right)
{
    motor_set_speed(clamp_speed(left), clamp_speed(right));
}
#endif

void chassis_init(void)
{
    motor_init();
    track_init();
    pid_init(&line_pid, 18, 0, 10, 38, 1600);
#if CHASSIS_USE_ENCODER
    pid_init(&left_speed_pid, 100, 0, 20, SPEED_PID_OUTPUT_LIMIT, 200);
    pid_init(&right_speed_pid, 100, 0, 20, SPEED_PID_OUTPUT_LIMIT, 200);
    encoder_init();
#endif
    chassis_state = CHASSIS_IDLE;
    lost_count = 0;
    cross_count = 0;
    cross_latched = 0;
    node_count = 0;
    last_line_error = 0;
}

void chassis_set_target(chassis_target_t target)
{
    if (target < CHASSIS_TARGET_A || target > CHASSIS_TARGET_D)
    {
        target = CHASSIS_TARGET_C;
    }

    chassis_target = target;
    node_count = 0;
    cross_count = 0;
    cross_latched = 0;
    lost_count = 0;
    pid_reset(&line_pid);
#if CHASSIS_USE_ENCODER
    pid_reset(&left_speed_pid);
    pid_reset(&right_speed_pid);
    encoder_init();
#endif
}

void chassis_unlock(void)
{
    motor_unlock();
    if (chassis_state == CHASSIS_LOCKED)
    {
        chassis_state = CHASSIS_IDLE;
    }
}

void chassis_lock(void)
{
    motor_lock();
    chassis_state = CHASSIS_LOCKED;
}

void chassis_emergency_stop(void)
{
    motor_lock();
    pid_reset(&line_pid);
#if CHASSIS_USE_ENCODER
    pid_reset(&left_speed_pid);
    pid_reset(&right_speed_pid);
#endif
    chassis_state = CHASSIS_LOCKED;
}

void chassis_brake(void)
{
    motor_brake();
    pid_reset(&line_pid);
#if CHASSIS_USE_ENCODER
    pid_reset(&left_speed_pid);
    pid_reset(&right_speed_pid);
#endif
    chassis_state = CHASSIS_IDLE;
}

void chassis_stop(void)
{
    motor_coast();
    pid_reset(&line_pid);
#if CHASSIS_USE_ENCODER
    pid_reset(&left_speed_pid);
    pid_reset(&right_speed_pid);
#endif
    chassis_state = CHASSIS_IDLE;
}

void chassis_drive(int forward, int turn)
{
    int left;
    int right;

    if (motor_is_locked())
    {
        return;
    }

    left = clamp_speed(forward + turn);
    right = clamp_speed(forward - turn);
    chassis_apply_speed(left, right);
    chassis_state = CHASSIS_IDLE;
}

void chassis_forward(int speed)
{
    speed = clamp_speed(speed);
    chassis_drive(speed, 0);
}

void chassis_backward(int speed)
{
    speed = clamp_speed(speed);
    if (speed < 0) speed = -speed;
    chassis_drive(-speed, 0);
}

void chassis_turn_left(int speed)
{
    speed = clamp_speed(speed);
    if (speed < 0) speed = -speed;
    chassis_drive(speed / 2, -speed / 2);
}

void chassis_turn_right(int speed)
{
    speed = clamp_speed(speed);
    if (speed < 0) speed = -speed;
    chassis_drive(speed / 2, speed / 2);
}

void chassis_rotate_left(int speed)
{
    speed = clamp_speed(speed);
    if (speed < 0) speed = -speed;
    chassis_drive(0, -speed);
}

void chassis_rotate_right(int speed)
{
    speed = clamp_speed(speed);
    if (speed < 0) speed = -speed;
    chassis_drive(0, speed);
}

void chassis_find_line(void)
{
    chassis_state = CHASSIS_FIND_LINE;

    if (last_line_error >= 0)
    {
        chassis_rotate_right(CHASSIS_FIND_SPEED);
    }
    else
    {
        chassis_rotate_left(CHASSIS_FIND_SPEED);
    }
}

void chassis_follow_target(int target)
{
    chassis_set_target((chassis_target_t)target);
    chassis_run_line();
}

void chassis_run_line(void)
{
    chassis_line_input_t line = chassis_read_line_input();
    int correction;
    int left;
    int right;

    if (motor_is_locked() || chassis_state == CHASSIS_FINISHED)
    {
        return;
    }

    if (line.stale)
    {
        chassis_brake();
        return;
    }

    if (!line.line_detected || line.all_white)
    {
        lost_count++;
        if (lost_count >= CHASSIS_LOST_LINE_LONG_TICKS)
        {
            motor_brake();
            chassis_state = CHASSIS_IDLE;
            pid_reset(&line_pid);
            return;
        }

        if (lost_count >= CHASSIS_LOST_LINE_SHORT_TICKS)
        {
            chassis_find_line();
            return;
        }
    }
    else
    {
        lost_count = 0;
    }

    if (line.active_count >= CHASSIS_NODE_ACTIVE_COUNT_THRESHOLD)
    {
        cross_count++;
        if (!cross_latched && cross_count >= CHASSIS_CROSS_CONFIRM)
        {
            cross_latched = 1;
            node_count++;
            chassis_state = CHASSIS_CROSS;

            if (node_count >= target_finish_nodes[chassis_target])
            {
                motor_brake();
                chassis_state = CHASSIS_FINISHED;
                return;
            }
        }
    }
    else
    {
        cross_count = 0;
        cross_latched = 0;
    }

    last_line_error = line.error;
    correction = pid_update(&line_pid, line.error);

    if (line.error > CHASSIS_CURVE_ERROR || line.error < -CHASSIS_CURVE_ERROR)
    {
        left = CHASSIS_CURVE_SPEED + correction;
        right = CHASSIS_CURVE_SPEED - correction;
    }
    else
    {
        left = CHASSIS_BASE_SPEED + correction;
        right = CHASSIS_BASE_SPEED - correction;
    }

    left = clamp_speed(left);
    right = clamp_speed(right);

    chassis_apply_speed(left, right);
    chassis_state = CHASSIS_LINE;
}

void chassis_encoder_reset(void)
{
    encoder_reset_counts();
}

void chassis_encoder_get_counts(int *e1, int *e2, int *e3, int *e4)
{
#if CHASSIS_USE_ENCODER
    int count_now[4];

    __disable_irq();
    count_now[0] = encoder_count[0];
    count_now[1] = encoder_count[1];
    count_now[2] = encoder_count[2];
    count_now[3] = encoder_count[3];
    __enable_irq();

    if (e1 != 0) *e1 = count_now[0];
    if (e2 != 0) *e2 = count_now[1];
    if (e3 != 0) *e3 = count_now[2];
    if (e4 != 0) *e4 = count_now[3];
#else
    if (e1 != 0) *e1 = 0;
    if (e2 != 0) *e2 = 0;
    if (e3 != 0) *e3 = 0;
    if (e4 != 0) *e4 = 0;
#endif
}

int chassis_encoder_get_left_count(void)
{
    int e1;
    int e2;

    chassis_encoder_get_counts(&e1, &e2, 0, 0);
    return (abs_int(e1) + abs_int(e2)) / 2;
}

int chassis_encoder_get_right_count(void)
{
    int e3;
    int e4;

    chassis_encoder_get_counts(0, 0, &e3, &e4);
    return (abs_int(e3) + abs_int(e4)) / 2;
}

chassis_state_t chassis_get_state(void)
{
    return chassis_state;
}

int chassis_get_node_count(void)
{
    return node_count;
}

int chassis_is_finished(void)
{
    return chassis_state == CHASSIS_FINISHED;
}

void GROUP1_IRQHandler(void)
{
#if CHASSIS_USE_ENCODER
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        GPIO_ENCODER_E1A_PIN | GPIO_ENCODER_E2A_PIN |
        GPIO_ENCODER_E3A_PIN | GPIO_ENCODER_E4A_PIN);

    if ((gpioA & GPIO_ENCODER_E1A_PIN) == GPIO_ENCODER_E1A_PIN)
    {
        encoder_add_edge(0, GPIO_ENCODER_E1A_PORT, GPIO_ENCODER_E1A_PIN,
            GPIO_ENCODER_E1B_PORT, GPIO_ENCODER_E1B_PIN, ENCODER_E1_SIGN);
        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_ENCODER_E1A_PIN);
    }

    if ((gpioA & GPIO_ENCODER_E2A_PIN) == GPIO_ENCODER_E2A_PIN)
    {
        encoder_add_edge(1, GPIO_ENCODER_E2A_PORT, GPIO_ENCODER_E2A_PIN,
            GPIO_ENCODER_E2B_PORT, GPIO_ENCODER_E2B_PIN, ENCODER_E2_SIGN);
        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_ENCODER_E2A_PIN);
    }

    if ((gpioA & GPIO_ENCODER_E3A_PIN) == GPIO_ENCODER_E3A_PIN)
    {
        encoder_add_edge(2, GPIO_ENCODER_E3A_PORT, GPIO_ENCODER_E3A_PIN,
            GPIO_ENCODER_E3B_PORT, GPIO_ENCODER_E3B_PIN, ENCODER_E3_SIGN);
        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_ENCODER_E3A_PIN);
    }

    if ((gpioA & GPIO_ENCODER_E4A_PIN) == GPIO_ENCODER_E4A_PIN)
    {
        encoder_add_edge(3, GPIO_ENCODER_E4A_PORT, GPIO_ENCODER_E4A_PIN,
            GPIO_ENCODER_E4B_PORT, GPIO_ENCODER_E4B_PIN, ENCODER_E4_SIGN);
        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_ENCODER_E4A_PIN);
    }
#endif
}
