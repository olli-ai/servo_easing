#include "SE_servo.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "SE_def.h"
#include "servo_easing.h"
#include "SE_ticks.h"
#include "SE_algorithm.h"
#include "SE_errors.h"
#include "SE_logging.h"

#define SERVO_VALIDATE(servo, invalid)       \
    if (servo == NULL)                       \
    {                                        \
        SE_set_error("Servo input is NULL"); \
        return invalid;                      \
    }

#define SERVO_DATA_VALIDATE(servo, invalid)       \
    if (servo->servo_data == NULL)                \
    {                                             \
        SE_set_error("Servo is not initialized"); \
        return invalid;                           \
    }

enum servo_async_action
{
    eSERVO_ASYNC_NONE = 0,
    eSERVO_ASYNC_MOVE = 1,
    eSERVO_ASYNC_RESUME,
    eSERVO_ASYNC_STOP
};

enum servo_direction
{
    eSERVO_DIRECT_CLOCK_WISE = 0,
    eSERVO_DIRECT_COUNTER_CLOCKWISE,
};

struct _se_servo_data
{
    uint32_t milis_start;
    uint32_t milis_stop;
    uint32_t start_units;
    uint32_t current_units;
    uint32_t delta_units;
    uint32_t end_units;
    uint32_t milis_to_complete_move;
    uint16_t current_angle;
    uint16_t expect_angle;
    SE_servo_dest_reach_cb_t reach_cb;
    uint8_t speed;
    uint8_t await_action : 2;
    uint8_t is_moving : 1;
    uint8_t is_stop : 1;
    uint8_t is_inuse : 1;
    uint8_t direction : 1;
    uint8_t reverse : 2;
};

static struct _se_servo_data servo_data_instances[MAX_SERVO_INSTANCES] = {0};

static void _SE_servo_default_reach_callback(SE_servo_t *servo)
{
    SE_DEBUG("Servo %d destination reach %d deg", servo->id, servo->servo_data->current_angle);
}

static struct _se_servo_data *_SE_servo_get_new_data_instance()
{
    struct _se_servo_data *data = NULL;
    for (int i = 0; i < MAX_SERVO_INSTANCES; i++)
    {
        if (servo_data_instances[i].is_inuse == false)
        {
            data = &servo_data_instances[i];
            break;
        }
    }
    if (data != NULL)
    {
        data->is_inuse = true;
    }
    return data;
}

SE_ret_t SE_servo_init(SE_servo_t *servo, SE_argument_t *args)
{
    SERVO_VALIDATE(servo, kSE_NULL);

    SE_servo_data_t *servo_data = _SE_servo_get_new_data_instance();
    if (servo_data == NULL)
    {
        SE_set_error("Out of instance allow for servo");
        SE_ERROR("Total instances is %d, all in use", MAX_SERVO_INSTANCES);
        return kSE_NO_MEM;
    }
    servo->servo_data = servo_data;
    servo->mov_type = args->move_type;
    servo->easing_type = args->easing_type;
    servo->controller = NULL;
    servo->servo_data->milis_start = 0;
    servo->servo_data->milis_stop = 0;
    servo->servo_data->current_angle = args->init_angle;
    servo->servo_data->expect_angle = args->init_angle;
    servo->servo_data->is_moving = 0;
    servo->servo_data->is_stop = 1;
    servo->servo_data->milis_to_complete_move = 0;
    servo->servo_data->await_action = eSERVO_ASYNC_NONE;
    servo->servo_data->reach_cb = _SE_servo_default_reach_callback;
    servo->servo_data->current_units = 0;
    servo->servo_data->start_units = 0;
    servo->servo_data->end_units = 0;
    servo->servo_data->delta_units = 0;
    servo->servo_data->speed = args->speed;
    return kSE_SUCCESS;
}

void SE_servo_deinit(SE_servo_t *servo)
{
    if (servo == NULL)
    {
        return;
    }

    if (servo->servo_data == NULL)
    {
        return;
    }

    memset(servo->servo_data, '\0', sizeof(struct _se_servo_data));
    servo->servo_data = NULL;
    servo->controller = NULL;
}

SE_ret_t SE_servo_set_angle(SE_servo_t *servo, int angle)
{
    SERVO_VALIDATE(servo, kSE_NULL);
    SERVO_DATA_VALIDATE(servo, kSE_NULL);

    SE_DEBUG("Expect angle %d, current angle %d", servo->servo_data->expect_angle, servo->servo_data->current_angle);
    SE_DEBUG("Servo current unit %ld end units %ld", servo->servo_data->current_units, servo->servo_data->end_units);

    if (servo->servo_data->is_moving)
    {
        SE_set_error("Servo is moving, stop it first");
        return kSE_BUSY;
    }

    servo->servo_data->expect_angle = angle;
    return kSE_SUCCESS;
}

int SE_servo_get_angle(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, -1);
    SERVO_DATA_VALIDATE(servo, -1);

    return servo->servo_data->current_angle;
}

static bool _SE_servo_is_destination_reach(struct _se_servo_data *data)
{
    if (data->direction == eSERVO_DIRECT_CLOCK_WISE)
    {
        if (data->current_angle >= data->expect_angle)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    if (data->direction == eSERVO_DIRECT_COUNTER_CLOCKWISE)
    {
        if (data->current_angle <= data->expect_angle)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    SE_WARNING("Function reach undesire code, due to unhandle direction");
    return false;
}

static void _SE_servo_moving_update(SE_servo_t *servo)
{
    struct _se_servo_data *data = (struct _se_servo_data *)servo->servo_data;
    if (_SE_servo_is_destination_reach(data))
    {
        uint32_t unit_per_us = servo->controller->get_pulse_resolution(servo->controller, servo->id);
        uint32_t duty = data->end_units * unit_per_us /100;
        servo->controller->set_duty(servo->controller, servo->id, duty);
        data->await_action = eSERVO_ASYNC_STOP;
        data->reach_cb(servo);
        return;
    }

    const struct SE_controller_info *info_ref = servo->controller->get_info_ref(servo->controller);
    uint32_t easing_value = SE_algorithm_update(servo);
    SE_DEBUG("Easing value %d", easing_value);
    switch (data->direction)
    {
    case eSERVO_DIRECT_CLOCK_WISE:
        data->current_units = (easing_value * data->delta_units / 100) + data->start_units;
        break;

    case eSERVO_DIRECT_COUNTER_CLOCKWISE:
        data->current_units = data->start_units - (easing_value * data->delta_units / 100);
        break;

    default:
        SE_WARNING("Unhandle direction of server in start function value is %d", data->direction);
        break;
    }
    
    uint32_t unit_per_deg = (info_ref->units_for_180_degree - info_ref->units_for_0_degree) / 180;
    uint32_t unit_per_us = servo->controller->get_pulse_resolution(servo->controller, servo->id);
    uint32_t duty = data->current_units * unit_per_us / 100;
    data->current_angle = (data->current_units - info_ref->units_for_0_degree) / unit_per_deg;
    servo->controller->set_duty(servo->controller, servo->id, duty);
}

static void _SE_servo_await_action_update(SE_servo_t *servo)
{
    struct _se_servo_data *data = (struct _se_servo_data *)servo->servo_data;

    switch (data->await_action)
    {
    case eSERVO_ASYNC_MOVE:
        servo->servo_data->is_moving = true;
        servo->servo_data->is_stop = false;
        servo->servo_data->milis_start = SE_tick_get_current_tick();
        break;
    case eSERVO_ASYNC_STOP:
        servo->servo_data->milis_stop = SE_tick_get_current_tick();
        servo->servo_data->is_moving = false;
        servo->servo_data->is_stop = true;
        break;
    case eSERVO_ASYNC_RESUME:
        data->milis_start = data->milis_stop + SE_tick_get_current_tick();
        data->is_moving = true;
        data->is_stop = false;
        break;
    default:
        break;
    }

    data->await_action = eSERVO_ASYNC_NONE;
}

SE_ret_t SE_servo_update(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, kSE_NULL);
    SERVO_DATA_VALIDATE(servo, kSE_NULL);

    _SE_servo_await_action_update(servo);
    if (servo->servo_data->is_moving)
    {
        _SE_servo_moving_update(servo);
    }
    return kSE_SUCCESS;
}

uint8_t SE_servo_is_moving(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, false);
    SERVO_DATA_VALIDATE(servo, false);

    return servo->servo_data->is_moving;
}

uint8_t SE_servo_is_stop(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, false);
    SERVO_DATA_VALIDATE(servo, false);

    return servo->servo_data->is_stop;
}

SE_ret_t SE_servo_stop(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, kSE_NULL);
    SERVO_DATA_VALIDATE(servo, kSE_NULL);

    servo->servo_data->is_moving = false;
    servo->servo_data->is_stop = true;
    servo->servo_data->milis_stop = SE_tick_get_current_tick();
    return kSE_SUCCESS;
}

SE_ret_t SE_servo_resume(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, kSE_NULL);
    SERVO_DATA_VALIDATE(servo, kSE_NULL);

    servo->servo_data->await_action = eSERVO_ASYNC_RESUME;
    return kSE_SUCCESS;
}

SE_ret_t SE_servo_set_speed(SE_servo_t *servo, uint8_t speed)
{
    SERVO_VALIDATE(servo, kSE_NULL);
    SERVO_DATA_VALIDATE(servo, kSE_NULL);

    servo->servo_data->speed = speed;
    return kSE_SUCCESS;
}

uint8_t SE_servo_get_speed(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, 0);
    SERVO_DATA_VALIDATE(servo, 0);

    return servo->servo_data->speed;
}

uint32_t SE_servo_get_milis_to_complete_move(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, 0);
    SERVO_DATA_VALIDATE(servo, 0);

    return servo->servo_data->milis_to_complete_move;
}

SE_ret_t SE_servo_set_milis_to_complete_move(SE_servo_t *servo, uint32_t milis)
{
    SERVO_VALIDATE(servo, kSE_NULL);
    SERVO_DATA_VALIDATE(servo, kSE_NULL);

    servo->servo_data->milis_to_complete_move = milis;
    return kSE_SUCCESS;
}

static inline enum servo_direction _SE_servo_get_direction(struct _se_servo_data *data)
{
    enum servo_direction direction = eSERVO_DIRECT_CLOCK_WISE;
    if (data->expect_angle < data->current_angle)
    {
        direction = eSERVO_DIRECT_COUNTER_CLOCKWISE;
    }

    return direction;
}

SE_ret_t SE_servo_start(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, kSE_NULL);
    SERVO_DATA_VALIDATE(servo, kSE_NULL);
    const struct SE_controller_info *info_ref = servo->controller->get_info_ref(servo->controller);
    SE_servo_data_t *data = servo->servo_data;

    data->direction = _SE_servo_get_direction(data);
    uint32_t delta_angle = abs(data->expect_angle - data->current_angle);
    data->milis_to_complete_move = delta_angle * 1000 / data->speed;

    SE_DEBUG("Start angle %d end angle %d", data->current_angle, data->expect_angle);
    uint32_t unit_per_deg = (info_ref->units_for_180_degree - info_ref->units_for_0_degree) / 180;
    data->start_units = info_ref->units_for_0_degree + data->current_angle * unit_per_deg;
    data->end_units = info_ref->units_for_0_degree + data->expect_angle * unit_per_deg;
    SE_DEBUG("Start units %d end units %d", data->start_units, data->end_units);
    data->delta_units = (data->end_units > data->start_units) ? (data->end_units - data->start_units):
                        (data->start_units - data->end_units);
    SE_DEBUG("ms to complete move %d, delta units: %d", data->milis_to_complete_move,
             data->delta_units);
    data->await_action = eSERVO_ASYNC_MOVE;
    return kSE_SUCCESS;
}

SE_ret_t SE_servo_pause(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, kSE_NULL);
    SERVO_DATA_VALIDATE(servo, kSE_NULL);

    servo->servo_data->await_action = eSERVO_ASYNC_STOP;
    return kSE_SUCCESS;
}

uint32_t SE_servo_get_delta_unit_to_move(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, 0);
    SERVO_DATA_VALIDATE(servo, 0);
    return servo->servo_data->delta_units;
}

uint32_t SE_servo_get_start_move_milis(SE_servo_t *servo)
{
    SERVO_VALIDATE(servo, 0);
    SERVO_DATA_VALIDATE(servo, 0);

    return servo->servo_data->milis_start;
}

SE_ret_t SE_servo_on_destination_reach(SE_servo_t *servo, SE_servo_dest_reach_cb_t callback)
{
    SERVO_VALIDATE(servo, kSE_NULL);
    SERVO_DATA_VALIDATE(servo, kSE_NULL);

    if (callback == NULL)
    {
        SE_set_error("Call back is null, will not set");
        return kSE_NULL;
    }

    servo->servo_data->reach_cb = callback;
    return kSE_SUCCESS;
}