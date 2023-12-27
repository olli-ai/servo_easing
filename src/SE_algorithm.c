#include "SE_algorithm.h"

#include <math.h>

#include "SE_logging.h"
#include "SE_errors.h"
#include "SE_ticks.h"

static float SE_move_in_update(SE_servo_t *servo);
static float SE_move_out_update(SE_servo_t *servo);
static float SE_move_in_out_update(SE_servo_t *servo);
static float SE_move_bouncing_out_in_update(SE_servo_t *servo);
static inline float SE_easing_function(SE_servo_t *servo, float time_factor);

uint32_t SE_algorithm_update(SE_servo_t *servo)
{
    float servo_value = 0;
    switch (servo->mov_type)
    {
    case eSE_MOV_IN:
        servo_value = SE_move_in_update(servo) * 100;
        break;
    case eSE_MOV_OUT:
        servo_value = SE_move_out_update(servo) * 100;
        break;
    case eSE_MOV_IN_OUT:
        servo_value = SE_move_in_out_update(servo) * 100;
        break;
    case eSE_MOV_BOUNCING_OUT_IN:
        servo_value = SE_move_bouncing_out_in_update(servo) * 100;
        break;
    default:
        break;
    }
    return (uint32_t)servo_value;
}

static uint32_t _SE_get_elapse_ticks(SE_servo_t *servo)
{
    uint32_t milis_since_start = 0;
    if (SE_tick_get_current_tick() < SE_servo_get_start_move_milis(servo))
    {
        milis_since_start = (0xffffffff - SE_servo_get_start_move_milis(servo)) + SE_tick_get_current_tick(); 
    } else
    {
        milis_since_start = SE_tick_get_current_tick() - SE_servo_get_start_move_milis(servo);
    }

    return milis_since_start;
}

static float SE_move_in_update(SE_servo_t *servo)
{
    uint32_t milis_since_start = _SE_get_elapse_ticks(servo);
    SE_DEBUG("Milis since start %ld", milis_since_start);
    if (milis_since_start > SE_servo_get_milis_to_complete_move(servo))
    {
        return 1;
    }

    float time_factor = (float)milis_since_start / SE_servo_get_milis_to_complete_move(servo);
    float movement_completed = SE_easing_function(servo, time_factor);
    SE_DEBUG("Easing movement completed value %0.2f", movement_completed);
    return movement_completed;
}

static float SE_move_out_update(SE_servo_t *servo)
{
    uint32_t milis_since_start = _SE_get_elapse_ticks(servo);

    if (milis_since_start > SE_servo_get_milis_to_complete_move(servo))
    {
        return 1;
    }
    float time_factor = (float)milis_since_start / SE_servo_get_milis_to_complete_move(servo);
    float movement_completed = 1.0f - (float)SE_easing_function(servo, (1.0f - time_factor));
    return movement_completed;
}

static float SE_move_in_out_update(SE_servo_t *servo)
{
    uint32_t milis_since_start = _SE_get_elapse_ticks(servo);

    if (milis_since_start > SE_servo_get_milis_to_complete_move(servo))
    {
        return 1;
    }

    float time_factor = (float)milis_since_start / SE_servo_get_milis_to_complete_move(servo);
    float movement_completed = 0.0f;
    if (time_factor <= 0.5f)
    {
        movement_completed = 0.5f * SE_easing_function(servo, 2.0f * time_factor);
    }
    else
    {
        movement_completed = 1.0f - (0.5f * SE_easing_function(servo, 2.0f - (2.0f * time_factor)));
    }
    SE_DEBUG("Easing movement completed value %0.2f", movement_completed);

    return movement_completed;
}

static float SE_move_bouncing_out_in_update(SE_servo_t *servo)
{
    uint32_t milis_since_start = _SE_get_elapse_ticks(servo);
    SE_DEBUG("Milis since start %ld", milis_since_start);
    if (milis_since_start > SE_servo_get_milis_to_complete_move(servo))
    {
        return 1;
    }
    float time_factor = (float)milis_since_start / SE_servo_get_milis_to_complete_move(servo);
    float movement_completed = 0.0f;
    if (time_factor <= 0.5f)
    {
        movement_completed = 1.0f - SE_easing_function(servo, (1.0f - 2.0f * time_factor));
    }
    else
    {
        movement_completed = 1.0f - SE_easing_function(servo, (2.0f * time_factor) - 1.0f);
    }
    return movement_completed;
}

static inline float SE_linear_easing(float time_factor)
{
    return 0.0f;
}

static inline float SE_quaractic_in(float time_factor)
{
    return time_factor * time_factor;
}

static inline float SE_quartic_in(float time_factor)
{
    return SE_quaractic_in(SE_quaractic_in(time_factor));
}

static inline float SE_sine_in(float time_factor)
{
    return sin((time_factor - 1) * M_PI_2) + 1;
}

static inline float SE_cicular_in(float time_factor)
{
    return 1 - sqrt(1 - (time_factor * time_factor));
}

static inline float SE_cubic_in(float time_factor)
{
    return SE_quaractic_in(time_factor) * time_factor;
}

static inline float SE_precision_in(float time_factor)
{
    SE_set_error("Method not implemented");
    return time_factor;
}

static inline float SE_elastic_in(float time_factor)
{
    return sin(13 * M_PI_2 * time_factor) * pow(2, 10 * (time_factor - 1));
}

static inline float SE_back_in(float time_factor)
{
    return (time_factor * time_factor * time_factor) - (time_factor * sin(time_factor * M_PI));
}

static inline float SE_easing_function(SE_servo_t *servo, float time_factor)
{
    float percent = 0.0f;
    switch (servo->easing_type)
    {
    case eSE_EASE_LINEAR:
        percent = SE_linear_easing(time_factor);
        break;
    case eSE_EASE_QUARACTIC:
        percent = SE_quaractic_in(time_factor);
        break;
    case eSE_EASE_QUARTIC:
        percent = SE_quartic_in(time_factor);
        break;
    case eSE_EASE_SINE:
        percent = SE_sine_in(time_factor);
        break;
    case eSE_EASE_CIRCULAR:
        percent = SE_cicular_in(time_factor);
        break;
    case eSE_EASE_BACK:
        percent = SE_back_in(time_factor);
        break;
    case eSE_EASE_ELASTIC:
        percent = SE_elastic_in(time_factor);
        break;
    case eSE_EASE_PRECISION:
        percent = SE_precision_in(time_factor);
        break;
    default:
        SE_WARNING("Easing type %d is not supported", servo->easing_type);
        SE_set_error("Easing method not implement");
        break;
    }

    return percent;
}