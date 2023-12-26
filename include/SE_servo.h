#ifndef SE_SERVO_H
#define SE_SERVO_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "SE_enum.h"
#include "stdint.h"

typedef struct _se_servo_data SE_servo_data_t;

typedef struct _servo_args {
    int controller_id;
    uint32_t period_us;
    SE_easing_mov_t move_type;
    SE_easing_t easing_type;
    uint8_t servo_id;
    uint8_t speed;
    uint8_t init_angle;
    uint8_t reverse;
} SE_argument_t;

typedef struct _se_servo
{
    int id;
    SE_easing_mov_t mov_type;
    SE_easing_t easing_type;
    struct SE_controller *controller;
    SE_servo_data_t *servo_data;
    uint8_t period_ms;
} SE_servo_t;

typedef void (*SE_servo_dest_reach_cb_t)(SE_servo_t *);

SE_ret_t SE_servo_init(SE_servo_t *servo, SE_argument_t *args); 
void SE_servo_deinit(SE_servo_t *servo);
SE_ret_t SE_servo_set_angle(SE_servo_t *servo, int angle);
int SE_servo_get_angle(SE_servo_t *servo);
SE_ret_t SE_servo_update(SE_servo_t *servo);
uint8_t SE_servo_is_moving(SE_servo_t *servo);
uint8_t SE_servo_is_stop(SE_servo_t *servo);
SE_ret_t SE_servo_start(SE_servo_t *servo);
SE_ret_t SE_servo_stop(SE_servo_t *servo);
SE_ret_t SE_servo_resume(SE_servo_t *servo);
SE_ret_t SE_servo_set_speed(SE_servo_t *servo, uint8_t deg_per_sec);
uint8_t SE_servo_get_speed(SE_servo_t *servo);
uint32_t SE_servo_get_milis_to_complete_move(SE_servo_t *servo);
SE_ret_t SE_servo_set_milis_to_complete_move(SE_servo_t *servo, uint32_t milis);
uint32_t SE_servo_get_delta_unit_to_move(SE_servo_t *servo);
uint32_t SE_servo_get_start_move_milis(SE_servo_t *servo);
SE_ret_t SE_servo_on_destination_reach(SE_servo_t *servo, SE_servo_dest_reach_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /*SE_SERVO_H*/