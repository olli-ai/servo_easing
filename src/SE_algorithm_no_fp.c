#include "SE_algorithm.h"

#include "SE_logging.h"
#include "SE_errors.h"
#include "SE_ticks.h"

static uint32_t SE_move_in_update(SE_servo_t *servo);
static uint32_t SE_move_out_update(SE_servo_t *servo);
static uint32_t SE_move_in_out_update(SE_servo_t *servo);
static uint32_t SE_move_bouncing_out_in_update(SE_servo_t *servo);
static inline uint32_t SE_easing_function(SE_servo_t *servo, uint32_t completed_percent);

uint32_t SE_algorithm_update(SE_servo_t *servo)
{
    uint32_t servo_value = 0;
    switch (servo->mov_type)
    {
    case eSE_MOV_IN:
        servo_value = SE_move_in_update(servo);
        break;
    case eSE_MOV_OUT:
        servo_value = SE_move_out_update(servo);
        break;
    case eSE_MOV_IN_OUT:
        servo_value = SE_move_in_out_update(servo);
        break;
    case eSE_MOV_BOUNCING_OUT_IN:
        servo_value = SE_move_bouncing_out_in_update(servo);
        break;
    default:
        break;
    }
    return servo_value;
}

static uint32_t _SE_get_elapse_ticks(SE_servo_t *servo)
{
    uint32_t milis_since_start = 0;
    if (SE_tick_get_current_tick() < SE_servo_get_start_move_milis(servo))
    {
        milis_since_start = (0xffffffff - SE_servo_get_start_move_milis(servo)) + SE_tick_get_current_tick();
    }
    else
    {
        milis_since_start = SE_tick_get_current_tick() - SE_servo_get_start_move_milis(servo);
    }

    return milis_since_start;
}

static uint32_t SE_move_in_update(SE_servo_t *servo)
{
    uint32_t milis_since_start = _SE_get_elapse_ticks(servo);
    uint32_t milis_to_move = SE_servo_get_milis_to_complete_move(servo);

    if (milis_since_start > SE_servo_get_milis_to_complete_move(servo))
    {
        return 100;
    }

    uint32_t completed_percent = milis_since_start * 100 / milis_to_move;
    uint32_t movement_completed = SE_easing_function(servo, completed_percent);
    return movement_completed;
}

static uint32_t SE_move_out_update(SE_servo_t *servo)
{
    uint32_t milis_since_start = _SE_get_elapse_ticks(servo);
    if (milis_since_start > SE_servo_get_milis_to_complete_move(servo))
    {
        return 100;
    }

    uint32_t milis_to_move = SE_servo_get_milis_to_complete_move(servo);
    uint32_t completed_percent = milis_since_start * 100 / milis_to_move;
    uint32_t movement_completed = 100 - SE_easing_function(servo, (100 - completed_percent));
    return movement_completed;
}

static uint32_t SE_move_in_out_update(SE_servo_t *servo)
{
    uint32_t milis_since_start = _SE_get_elapse_ticks(servo);
    if (milis_since_start > SE_servo_get_milis_to_complete_move(servo))
    {
        return 100;
    }

    uint32_t milis_to_move = SE_servo_get_milis_to_complete_move(servo);
    uint32_t completed_percent = milis_since_start * 100 / milis_to_move;
    uint32_t movement_completed = 0;
    if (completed_percent <= 50)
    {
        movement_completed = 0.5 * SE_easing_function(servo, 2 * completed_percent);
    }
    else
    {
        movement_completed = 100 - (0.5 * SE_easing_function(servo, 200 - (2 * completed_percent)));
    }

    return movement_completed;
}

static uint32_t SE_move_bouncing_out_in_update(SE_servo_t *servo)
{
    uint32_t milis_since_start = _SE_get_elapse_ticks(servo);
    if (milis_since_start > SE_servo_get_milis_to_complete_move(servo))
    {
        return 100;
    }

    uint32_t milis_to_move = SE_servo_get_milis_to_complete_move(servo);
    uint32_t completed_percent = milis_since_start * 100 / milis_to_move;
    uint32_t movement_completed = 0;
    if (completed_percent <= 50)
    {
        movement_completed = 100 - SE_easing_function(servo, (100 - 2 * completed_percent));
    }
    else
    {
        movement_completed = 100 - SE_easing_function(servo, (2 * completed_percent) - 100);
    }
    return movement_completed;
}

static inline uint32_t SE_quaractic_in(uint32_t completed_percent)
{
    return (completed_percent * completed_percent) / (100);
}

static inline uint32_t SE_quartic_in(uint32_t completed_percent)
{
    return SE_quaractic_in(SE_quaractic_in(completed_percent));
}

static inline uint32_t SE_easing_function(SE_servo_t *servo, uint32_t completed_percent)
{
    uint32_t percent = 0;
    switch (servo->easing_type)
    {
    case eSE_EASE_QUARACTIC:
        percent = SE_quaractic_in(completed_percent);
        break;
    case eSE_EASE_QUARTIC:
        percent = SE_quartic_in(completed_percent);
        break;
    default:
        SE_WARNING("Easing type %d is not supported or disable due to no FP", servo->easing_type);
        SE_set_error("Easing method not support");
        break;
    }
    return percent;
}