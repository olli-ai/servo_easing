#include "dummy_controller.h"

#include <stdbool.h>
#include "SE_servo.h"
#include "SE_errors.h"
#include "SE_logging.h"

#define DUMMY_MAX_SERVO (16)
#define CONTROLLER_VALIDATE(controller, invalid) \
    if (controller == NULL)                      \
    {                                            \
        SE_set_error("Controller is null");      \
        return invalid;                          \
    }

#define DEFAULT_DUMMY_UNITS_FOR_0_DEGREE 111                         // 111.411 = 544 us
#define DEFAULT_DUMMY_UNITS_FOR_45_DEGREE (111 + ((491 - 111) / 4))  // 206
#define DEFAULT_DUMMY_UNITS_FOR_90_DEGREE (111 + ((491 - 111) / 2))  // 301 = 1472 us
#define DEFAULT_DUMMY_UNITS_FOR_135_DEGREE (491 - ((491 - 111) / 4)) // 369
#define DEFAULT_DUMMY_UNITS_FOR_180_DEGREE 491                       // 491.52 = 2400 us
#define PULSE_UNIT_US(period_us) ((period_us) * 100 / 400)
#define DEFAULT_DUMMY_PERIOD_US      (40000)

static SE_ret_t dummy_init_device(struct SE_controller *controller);
static void dummy_deinit_device(struct SE_controller *controller);
static SE_ret_t dummy_open_servo(struct SE_controller *controller, uint8_t servo_id);
static SE_ret_t dummy_close_servo(struct SE_controller *controller, uint8_t servo_id);
static SE_ret_t dummy_set_duty(struct SE_controller *controller, uint8_t servo_id, uint32_t duty_us);
static SE_ret_t dummy_set_period(struct SE_controller *controller, uint8_t servo_id, uint32_t period_us);
static const struct SE_controller_info *dummy_get_info_ref(struct SE_controller *controller);
static struct SE_controller_info dummy_get_info_copy(struct SE_controller *controller);
static SE_ret_t dummy_set_id(struct SE_controller *controller, int id);
static uint32_t dummy_get_pulse_resolution(struct SE_controller *controller, uint8_t servo_id);
static SE_ret_t dummy_servo_callback_register(void *servo);

struct dummy_servo_info
{
    bool enable;
    uint32_t period_us;
    uint32_t duty_us;
    uint32_t pwm_resolution;
};

struct dummy_data
{
    struct SE_controller_info info;
    struct dummy_servo_info servo[DUMMY_MAX_SERVO];
    bool is_open;
};

static struct dummy_data controller_data = {
    .info = {
        .name = "Dummy servo controller",
        .id = 0,
        .max_servo = DUMMY_MAX_SERVO,
        .units_for_0_degree = DEFAULT_DUMMY_UNITS_FOR_0_DEGREE,
        .units_for_180_degree = DEFAULT_DUMMY_UNITS_FOR_180_DEGREE,
    },
    .servo = {
        {
            .duty_us = 0,
            .period_us = DEFAULT_DUMMY_PERIOD_US,
            .enable = false,
            .pwm_resolution = PULSE_UNIT_US(DEFAULT_DUMMY_PERIOD_US),
        }
    },
    .is_open = false,
};

static struct SE_controller dummy_controller = {
    .controller_init = dummy_init_device,
    .controller_deinit = dummy_deinit_device,
    .open_servo = dummy_open_servo,
    .close_servo = dummy_close_servo,
    .set_duty = dummy_set_duty,
    .set_period = dummy_set_period,
    .get_info_ref = dummy_get_info_ref,
    .get_info_copy = dummy_get_info_copy,
    .set_id = dummy_set_id,
    .get_pulse_resolution = dummy_get_pulse_resolution,
    .register_servo_event = dummy_servo_callback_register,
    .controller_data = (void *)&controller_data,
};

struct SE_controller *Dummy_get_controller()
{
    return &dummy_controller;
}

static SE_ret_t dummy_init_device(struct SE_controller *controller)
{
    if (controller == NULL)
    {
        SE_set_error("Controller is null, ignore");
        return kSE_NULL;
    }

    SE_DEBUG("Init controller for controller id %p", controller);
    return kSE_SUCCESS;
}

static void dummy_deinit_device(struct SE_controller *controller)
{
    if (controller == NULL)
    {
        SE_set_error("Controller is null, ignore");
        return;
    }

    SE_DEBUG("Deinit controller id %p", controller);
}

static SE_ret_t dummy_open_servo(struct SE_controller *controller, uint8_t servo_id)
{
    if (controller == NULL)
    {
        SE_set_error("Controller is null, ignore");
        return kSE_NULL;
    }
    SE_DEBUG("Open servo %d", servo_id);
    return kSE_SUCCESS;
}

static SE_ret_t dummy_close_servo(struct SE_controller *controller, uint8_t servo_id)
{
    if (controller == NULL)
    {
        SE_set_error("Controller is null, ignore");
        return kSE_NULL;
    }
    SE_DEBUG("Close servo %d", servo_id);
    return kSE_SUCCESS;
}

static SE_ret_t dummy_set_duty(struct SE_controller *controller, uint8_t servo_id, uint32_t duty)
{
    if (controller == NULL)
    {
        SE_set_error("Controller is null, ignore");
        return kSE_NULL;
    }
    SE_DEBUG("Servo %d set duty to %u", servo_id, duty);
    return kSE_SUCCESS;
}

static SE_ret_t dummy_set_period(struct SE_controller *controller, uint8_t servo_id, uint32_t period_us)
{
    if (controller == NULL)
    {
        SE_set_error("Controller is null, ignore");
        return kSE_NULL;
    }
    SE_DEBUG("Servo %d set period to %u", servo_id, period_us);
    struct dummy_data *data = (struct dummy_data*)controller->controller_data;
    data->servo[servo_id].pwm_resolution = PULSE_UNIT_US(period_us);
    data->servo[servo_id].period_us = period_us;
    SE_DEBUG("Servo pulse resolution is %d unit(s)/us", data->servo[servo_id].pwm_resolution);
    return kSE_SUCCESS;
}

static const struct SE_controller_info *dummy_get_info_ref(struct SE_controller *controller)
{
    const struct SE_controller_info *info_ref = NULL;
    if (controller == NULL)
    {
        SE_set_error("Controller is null, ignore");
        return info_ref;
    }
    struct dummy_data *data = (struct dummy_data *)controller->controller_data;
    info_ref = &data->info;
    return info_ref;
}

static struct SE_controller_info dummy_get_info_copy(struct SE_controller *controller)
{
    struct SE_controller_info info = {0};
    if (controller == NULL)
    {
        SE_set_error("Controller is null, ignore");
        return info;
    }
    struct dummy_data *data = (struct dummy_data *)controller->controller_data;
    info = data->info;
    return info;
}

static SE_ret_t dummy_set_id(struct SE_controller *controller, int id)
{
    if (controller == NULL)
    {
        SE_set_error("Controller is null, ignore");
        return kSE_NULL;
    }

    struct dummy_data *data = (struct dummy_data *)controller->controller_data;
    data->info.id = id;
    return kSE_SUCCESS;
}

static uint32_t dummy_get_pulse_resolution(struct SE_controller *controller, uint8_t servo_id)
{
    CONTROLLER_VALIDATE(controller, 0);
    struct dummy_data *data = (struct dummy_data *)controller->controller_data;
    if (servo_id >= data->info.max_servo)
    {
        SE_set_error("Servo id is out of range");
        return 0;
    }

    return data->servo[servo_id].pwm_resolution;
}

static void _dummy_servo_update_callback(SE_servo_t *servo)
{
    SE_INFO("Servo %d update", servo->id);
}

static SE_ret_t dummy_servo_callback_register(void *servo)
{
    SE_ret_t ret = SE_servo_on_update((SE_servo_t *)servo, _dummy_servo_update_callback);
    return ret;
}