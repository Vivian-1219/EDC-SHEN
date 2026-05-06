#ifndef CHASSIS_IFACE_H
#define CHASSIS_IFACE_H

typedef enum {
    CHASSIS_LOCKED = 0,
    CHASSIS_IDLE,
    CHASSIS_LINE,
    CHASSIS_FIND_LINE,
    CHASSIS_CROSS,
    CHASSIS_FINISHED
} chassis_state_t;

typedef enum {
    CHASSIS_TARGET_A = 0,
    CHASSIS_TARGET_B,
    CHASSIS_TARGET_C,
    CHASSIS_TARGET_D
} chassis_target_t;

void chassis_init(void);
void chassis_set_target(chassis_target_t target);
void chassis_unlock(void);
void chassis_lock(void);
void chassis_emergency_stop(void);
void chassis_brake(void);
void chassis_stop(void);
void chassis_drive(int forward, int turn);
void chassis_forward(int speed);
void chassis_backward(int speed);
void chassis_turn_left(int speed);
void chassis_turn_right(int speed);
void chassis_rotate_left(int speed);
void chassis_rotate_right(int speed);
void chassis_find_line(void);
void chassis_follow_target(int target);
void chassis_run_line(void);
void chassis_encoder_reset(void);
void chassis_encoder_get_counts(int *e1, int *e2, int *e3, int *e4);
int chassis_encoder_get_left_count(void);
int chassis_encoder_get_right_count(void);
chassis_state_t chassis_get_state(void);
int chassis_get_node_count(void);
int chassis_is_finished(void);

#endif
