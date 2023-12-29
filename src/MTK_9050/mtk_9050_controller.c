#include "mtk_9050_controller.h"
#include "mtk_9050_pwm.h"

#include <stdbool.h>
#include <stdio.h>
#include <dirent.h>
#include <unistd.h>
#include <malloc.h>
#include <string.h>
#include <errno.h>

#include "SE_servo.h"
#include "SE_errors.h"
#include "SE_logging.h"

#define MTK_9050_MAX_SERVO 16

#define CONTROLLER_VALIDATE(controller, invalid) \
    if (controller == NULL)                      \
    {                                            \
        SE_set_error("Controller is null");      \
        return invalid;                          \
    }

#define DEFAULT_MTK_9050_UNITS_FOR_0_DEGREE 111
#define DEFAULT_MTK_9050_UNITS_FOR_180_DEGREE 491

#define DEFAULT_MTK_9050_PERIOD_US (40000)
#define PULSE_UNIT_US(period_us) ((float)(period_us * 100) / 400)

static SE_ret_t MTK_9050_linux_init_device(struct SE_controller *controller);
static void MTK_9050_linux_deinit_device(struct SE_controller *controller);
static SE_ret_t MTK_9050_linux_open_servo(struct SE_controller *controller, uint8_t servo_id);
static SE_ret_t MTK_9050_linux_close_servo(struct SE_controller *controller, uint8_t servo_id);
static SE_ret_t MTK_9050_linux_set_duty(struct SE_controller *controller, uint8_t servo_id, uint32_t duty_us);
static SE_ret_t MTK_9050_linux_set_period(struct SE_controller *controller, uint8_t servo_id, uint32_t period_us);
static const struct SE_controller_info *MTK_9050_linux_get_info_ref(struct SE_controller *controller);
static struct SE_controller_info MTK_9050_linux_get_info_copy(struct SE_controller *controller);
static SE_ret_t MTK_9050_linux_set_id(struct SE_controller *controller, int id);
static uint32_t MTK_9050_linux_get_pulse_resolution(struct SE_controller *controller, uint8_t servo_id);
static SE_ret_t MTK_9050_linux_servo_callback_register(void *servo);

struct mtk_9050_linux_servo_info
{
    bool enable;
    uint32_t period_us;
    uint32_t duty_us;
    uint32_t pwm_resolution;
    bool is_open;
};

struct mtk_9050_linux_data
{
    struct SE_controller_info info;
    struct mtk_9050_linux_servo_info servo[MTK_9050_MAX_SERVO];
    uint8_t pin_map[MTK_9050_MAX_SERVO];
    bool is_open;
    char *pwm_dev_name;
};

static struct mtk_9050_linux_data controller_data = {
    .info = {
        .name = "MTK_9050_linux servo controller",
        .id = 0,
        .max_servo = MTK_9050_MAX_SERVO,
        .units_for_0_degree = DEFAULT_MTK_9050_UNITS_FOR_0_DEGREE,
        .units_for_180_degree = DEFAULT_MTK_9050_UNITS_FOR_180_DEGREE,
    },
    .servo = {{0}},
    .pin_map = {15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    .is_open = false,
    .pwm_dev_name = NULL,
};

static struct SE_controller mtk_9050_linux_controller = {
    .controller_init = MTK_9050_linux_init_device,
    .controller_deinit = MTK_9050_linux_deinit_device,
    .open_servo = MTK_9050_linux_open_servo,
    .close_servo = MTK_9050_linux_close_servo,
    .set_duty = MTK_9050_linux_set_duty,
    .set_period = MTK_9050_linux_set_period,
    .get_info_ref = MTK_9050_linux_get_info_ref,
    .get_info_copy = MTK_9050_linux_get_info_copy,
    .set_id = MTK_9050_linux_set_id,
    .get_pulse_resolution = MTK_9050_linux_get_pulse_resolution,
    .register_servo_event = MTK_9050_linux_servo_callback_register,
    .controller_data = (void *)&controller_data,
};


static SE_ret_t MTK_9050_linux_init_device(struct SE_controller *controller)
{
    DIR *dir = NULL;
    struct dirent *entry = NULL;
    SE_ret_t ret = kSE_FAILED;
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_linux_data *data = (struct mtk_9050_linux_data *)controller->controller_data;
    if (data->is_open)
    {
        return kSE_SUCCESS;
    }

    dir = opendir("/sys/class/pwm");
    if (dir == NULL)
    {
        SE_set_error("Unable to open folder /sys/class/pwm");
        return ret;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, ".."))
        {
            continue;
        }
        SE_DEBUG("Operation on dir %s", entry->d_name);
        if (mtk_9050_pwm_is_device(entry->d_name))
        {
            char dev_name[128] = "";
            int num_byte = snprintf(dev_name, 128, "/sys/class/pwm/%s", entry->d_name);
            data->pwm_dev_name = malloc(num_byte + 1);
            if (data->pwm_dev_name == NULL)
            {
                SE_ERROR("Unable to allocate memory for dev path %s", dev_name);
                SE_set_error("Unable to allocate memory for MTK_9050 dev name");
                ret = kSE_NULL;
                break;
            }
            strcpy(data->pwm_dev_name, dev_name);
            SE_DEBUG("Found MTK_9050 at dev name %s", data->pwm_dev_name);
            data->is_open = true;
            ret = kSE_SUCCESS;
            break;
        }
    }

    if (ret == kSE_FAILED)
    {
        SE_set_error("Not found any MTK_9050 in /sys/class/pwm");
    }

    closedir(dir);
    return ret;
}

static void MTK_9050_linux_deinit_device(struct SE_controller *controller)
{
    if (controller == NULL)
    {
        SE_set_error("Controller is NULL");
        return;
    }

    struct mtk_9050_linux_data *data = (struct mtk_9050_linux_data *)controller->controller_data;
    for (int i = 0; i < MTK_9050_MAX_SERVO; i++)
    {
        if (data->servo[i].enable == true)
        {
            mtk_9050_pwm_unexport_pin(data->pwm_dev_name, data->pin_map[i]);
            data->servo[i].enable = false;
        }
    }

    free(data->pwm_dev_name);
    data->pwm_dev_name = NULL;
    data->is_open = false;
}

static SE_ret_t _MTK_9050_linux_open_servo(struct mtk_9050_linux_data *data, uint8_t servo_id)
{
    if (servo_id >= data->info.max_servo)
    {
        return kSE_OUT_OF_RANGE;
    }

    SE_ret_t ret = mtk_9050_pwm_export_pin(data->pwm_dev_name, data->pin_map[servo_id]);
    if (ret != kSE_SUCCESS)
    {
        return ret;
    }

    struct mtk_9050_linux_servo_info servo = {
        .duty_us = 0,
        .period_us = DEFAULT_MTK_9050_PERIOD_US,
        .enable = false,
        .pwm_resolution = PULSE_UNIT_US(DEFAULT_MTK_9050_PERIOD_US),
    };

    data->servo[servo_id] = servo;
    data->servo[servo_id].enable = true;

    ret = mtk_9050_pwm_set_period(data->pwm_dev_name, data->pin_map[servo_id], DEFAULT_MTK_9050_PERIOD_US);
    if (ret != kSE_SUCCESS)
    {
        return ret;
    }

    ret = mtk_9050_pwm_enable_pin(data->pwm_dev_name, data->pin_map[servo_id]);
    return ret;
}

static SE_ret_t MTK_9050_linux_open_servo(struct SE_controller *controller, uint8_t servo_id)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_linux_data *data = (struct mtk_9050_linux_data *)controller->controller_data;
    if (data->pwm_dev_name == NULL)
    {
        SE_set_error("The controller is not initialized");
        return kSE_TRY_AGAIN;
    }

    if (servo_id >= data->info.max_servo)
    {
        SE_set_error("Perform open servo out of range");
        return kSE_OUT_OF_RANGE;
    }

    return _MTK_9050_linux_open_servo(data, servo_id);
}

static SE_ret_t _MTK_9050_linux_close_servo(struct mtk_9050_linux_data *data, uint8_t servo_id)
{
    SE_ret_t ret = mtk_9050_pwm_disable_pin(data->pwm_dev_name, data->pin_map[servo_id]);
    if (ret != kSE_SUCCESS)
    {
        SE_WARNING("Unable to disable pin, pin remains open");
    } else
    {
        ret = mtk_9050_pwm_unexport_pin(data->pwm_dev_name, data->pin_map[servo_id]);
        data->servo[servo_id].enable = false;
    }
    return ret;
}

static SE_ret_t MTK_9050_linux_close_servo(struct SE_controller *controller, uint8_t servo_id)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_linux_data *data = (struct mtk_9050_linux_data *)controller->controller_data;
    if (data->pwm_dev_name == NULL)
    {
        SE_set_error("The controller is not initialized");
        return kSE_TRY_AGAIN;
    }

    if (servo_id >= data->info.max_servo)
    {
        SE_set_error("Perform open servo out of range");
        return kSE_OUT_OF_RANGE;
    }

    return _MTK_9050_linux_close_servo(data, servo_id);
}

static SE_ret_t _MTK_9050_linux_set_duty(struct mtk_9050_linux_data *data, uint8_t servo_id, uint32_t duty_us)
{
    if (!data->servo[servo_id].enable)
    {
        SE_set_error("Servo is not open to set duty");
        return kSE_FAILED;
    }

    return mtk_9050_pwm_set_duty(data->pwm_dev_name, data->pin_map[servo_id], duty_us);
}

static SE_ret_t MTK_9050_linux_set_duty(struct SE_controller *controller, uint8_t servo_id, uint32_t duty)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_linux_data *data = (struct mtk_9050_linux_data *)controller->controller_data;
    if (data->pwm_dev_name == NULL)
    {
        SE_set_error("The controller is not initialized");
        return kSE_TRY_AGAIN;
    }

    if (servo_id >= data->info.max_servo)
    {
        SE_set_error("Perform open servo out of range");
        return kSE_OUT_OF_RANGE;
    }

    return _MTK_9050_linux_set_duty(data, servo_id, duty);
}

static SE_ret_t _MTK_9050_linux_set_period(struct mtk_9050_linux_data *data, uint8_t servo_id, uint32_t period_us)
{
    if (!data->servo[servo_id].enable)
    {
        SE_set_error("Servo is not open to set period");
        return kSE_FAILED;
    }

    return mtk_9050_pwm_set_period(data->pwm_dev_name, data->pin_map[servo_id], period_us);
}

static SE_ret_t MTK_9050_linux_set_period(struct SE_controller *controller, uint8_t servo_id, uint32_t period_us)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_linux_data *data = (struct mtk_9050_linux_data *)controller->controller_data;
    if (data->pwm_dev_name == NULL)
    {
        SE_set_error("The controller is not initialized");
        return kSE_TRY_AGAIN;
    }

    if (servo_id >= data->info.max_servo)
    {
        SE_set_error("Perform open servo out of range");
        return kSE_OUT_OF_RANGE;
    }

    return _MTK_9050_linux_set_period(data, servo_id, period_us);
}

static const struct SE_controller_info *MTK_9050_linux_get_info_ref(struct SE_controller *controller)
{
    const struct SE_controller_info *ref_info = NULL;
    CONTROLLER_VALIDATE(controller, NULL);

    struct mtk_9050_linux_data *data = (struct mtk_9050_linux_data *)controller->controller_data;
    ref_info = &data->info;
    return ref_info;
}

static struct SE_controller_info MTK_9050_linux_get_info_copy(struct SE_controller *controller)
{
    struct SE_controller_info info = {0};
    CONTROLLER_VALIDATE(controller, info);

    struct mtk_9050_linux_data *data = (struct mtk_9050_linux_data *)controller->controller_data;
    info = data->info;
    return info;
}

struct SE_controller *mtk_9050_linux_get_controller(void)
{
    return &mtk_9050_linux_controller;
}

static SE_ret_t MTK_9050_linux_set_id(struct SE_controller *controller, int id)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_linux_data *data = (struct mtk_9050_linux_data *)controller->controller_data;
    data->info.id = id;
    return kSE_SUCCESS;
}

static uint32_t MTK_9050_linux_get_pulse_resolution(struct SE_controller *controller, uint8_t servo_id)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_linux_data *data = (struct mtk_9050_linux_data *)controller->controller_data;
    if (servo_id >= data->info.max_servo)
    {
        SE_set_error("Servo id is out of range");
        return 0;
    }

    return data->servo[servo_id].pwm_resolution;
}

static void _MTK_9050_linux_update_callback(SE_servo_t *servo)
{
}

static SE_ret_t MTK_9050_linux_servo_callback_register(void *servo)
{
    return SE_servo_on_update(servo, _MTK_9050_linux_update_callback);
}