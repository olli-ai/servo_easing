#include "mtk_9050_dc_controller.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <dirent.h>
#include <unistd.h>
#include <malloc.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>

#include "SE_errors.h"
#include "SE_servo.h"
#include "SE_logging.h"
#include "SE_controller.h"

#include "mtk_9050_pwm.h"

#define MTK_9050_MAX_MOTOR 4

#define CONTROLLER_VALIDATE(controller, invalid) \
    if (controller == NULL)                      \
    {                                            \
        SE_set_error("Controller is null");      \
        return invalid;                          \
    }

#define DEFAULT_MTK_9050_UNITS_FOR_0_DEGREE 0
#define DEFAULT_MTK_9050_UNITS_FOR_180_DEGREE 400 

#define DEV_NAME "mstar,pwm"
#define DEFAULT_MTK_9050_PERIOD_US (40000)
#define PULSE_UNIT_US(period_us) ((float)(period_us * 100) / 400)

static SE_ret_t MTK_9050_dc_motor_init_device(struct SE_controller *controller);
static void MTK_9050_dc_motor_deinit_device(struct SE_controller *controller);
static SE_ret_t MTK_9050_dc_motor_open_motor(struct SE_controller *controller, uint8_t motor_id);
static SE_ret_t MTK_9050_dc_motor_close_motor(struct SE_controller *controller, uint8_t motor_id);
static SE_ret_t MTK_9050_dc_motor_set_duty(struct SE_controller *controller, uint8_t motor_id, uint32_t duty_us);
static SE_ret_t MTK_9050_dc_motor_set_period(struct SE_controller *controller, uint8_t motor_id, uint32_t period_us);
static const struct SE_controller_info *MTK_9050_dc_motor_get_info_ref(struct SE_controller *controller);
static struct SE_controller_info MTK_9050_dc_motor_get_info_copy(struct SE_controller *controller);
static SE_ret_t MTK_9050_dc_motor_set_id(struct SE_controller *controller, int id);
static uint32_t MTK_9050_dc_motor_get_pulse_resolution(struct SE_controller *controller, uint8_t motor_id);
static SE_ret_t MTK_9050_dc_motor_servo_callback_register(void *servo);

enum move_direction {
    eMOVE_DIRECT_CLOCKWISE = 0,
    eMOVE_DIRECT_COUNT_CLOCKWISE
};

struct mtk_9050_dc_motor_info
{
    uint32_t period_us;
    uint32_t duty_us;
    int64_t last_counter;
    uint32_t pwm_resolution;
    uint16_t delta_move;
    uint8_t direction: 1;
    uint8_t enable: 1;
    uint8_t is_open: 1;
    uint8_t reverse: 5;
};

struct counter_value
{
    int64_t a_forward;
    int64_t b_backward;
    int64_t v_direction;
};

struct encoder_data
{
    struct counter_value channel_a;
    struct counter_value channel_b;
    struct counter_value channel_c;
    struct counter_value channel_d;
    struct counter_value channel_e;
    struct counter_value channel_f;
};

struct motor_capability
{
    uint8_t motor_forward;
    uint8_t motor_backward;
    bool has_feedback;
};

struct mtk_9050_dc_data
{
    struct SE_controller_info info;
    struct mtk_9050_dc_motor_info motor[MTK_9050_MAX_MOTOR];
    struct motor_capability motor_cap[MTK_9050_MAX_MOTOR];
    int encoder_fd;
    bool is_open;
    char *pwm_dev_name;
};

static struct mtk_9050_dc_data controller_data = {
    .info = {
        .name = "MTK_9050_dc_motor",
        .id = 0,
        .max_servo = MTK_9050_MAX_MOTOR,
        .units_for_0_degree = DEFAULT_MTK_9050_UNITS_FOR_0_DEGREE,
        .units_for_180_degree = DEFAULT_MTK_9050_UNITS_FOR_180_DEGREE,
    },
    .motor = {{0}},
    .motor_cap = {
        {.motor_forward = 15, .motor_backward = 14, .has_feedback = true},
        {.motor_forward = 0, .motor_backward = 0, .has_feedback = false},
        {.motor_forward = 0, .motor_backward = 0, .has_feedback = false},
        {.motor_forward = 0, .motor_backward = 0, .has_feedback = false},
    },
    .encoder_fd = -1,
    .is_open = false,
    .pwm_dev_name = NULL,
};

static struct SE_controller mtk_9050_dc_controller = {
    .controller_init = MTK_9050_dc_motor_init_device,
    .controller_deinit = MTK_9050_dc_motor_deinit_device,
    .open_servo = MTK_9050_dc_motor_open_motor,
    .close_servo = MTK_9050_dc_motor_close_motor,
    .set_duty = MTK_9050_dc_motor_set_duty,
    .set_period = MTK_9050_dc_motor_set_period,
    .get_info_ref = MTK_9050_dc_motor_get_info_ref,
    .get_info_copy = MTK_9050_dc_motor_get_info_copy,
    .set_id = MTK_9050_dc_motor_set_id,
    .get_pulse_resolution = MTK_9050_dc_motor_get_pulse_resolution,
    .register_servo_event = MTK_9050_dc_motor_servo_callback_register,
    .controller_data = (void *)&controller_data,
};

static SE_ret_t _MTK_9050_dc_motor_encoder_map(struct SE_controller *controller)
{
    if (access("/dev/encoder", F_OK) == -1)
    {
        SE_ERROR("Not found encoder, ignore");
        return kSE_FAILED;
    }

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    data->encoder_fd = open("/dev/encoder", O_RDWR);
    if (data->encoder_fd < 0)
    {
        SE_ERROR("Unable to open encoder");
        return kSE_FAILED;
    }

    return kSE_SUCCESS;
}

static SE_ret_t MTK_9050_dc_motor_init_device(struct SE_controller *controller)
{
    DIR *dir = NULL;
    struct dirent *entry = NULL;
    SE_ret_t ret = kSE_FAILED;
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    if (data->is_open)
    {
        return kSE_SUCCESS;
    }

    SE_ret_t open_encoder = _MTK_9050_dc_motor_encoder_map(controller);
    if (open_encoder != kSE_SUCCESS)
    {
        SE_WARNING("Unable to open encoder, ignore");
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

static void MTK_9050_dc_motor_deinit_device(struct SE_controller *controller)
{
    if (controller == NULL)
    {
        SE_set_error("Controller is NULL");
        return;
    }

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    for (int i = 0; i < MTK_9050_MAX_MOTOR; i++)
    {
        if (data->motor[i].enable)
        {
            controller->close_servo(controller, i);
            data->motor[i].enable = false;
        }
    }

    free(data->pwm_dev_name);
    data->pwm_dev_name = NULL;
    if (data->encoder_fd > 0)
    {
        close(data->encoder_fd);
        data->encoder_fd = -1;
    }
    data->is_open = false;
}

static SE_ret_t _MTK_9050_dc_motor_export_motor(struct mtk_9050_dc_data *data, uint8_t motor_id)
{
    char forward_pin_name[256] = {'\0'};
    char backward_pin_name[256] = {'\0'};
    snprintf(forward_pin_name, 256, "%s/pwm%d", data->pwm_dev_name, data->motor_cap[motor_id].motor_forward);
    snprintf(backward_pin_name, 256, "%s/pwm%d", data->pwm_dev_name, data->motor_cap[motor_id].motor_backward);
    SE_ret_t ret = kSE_SUCCESS;
    if (access(forward_pin_name, F_OK) != -1)
    {
        SE_INFO("Already export %d, ignore", data->motor_cap[motor_id].motor_forward);
    }
    else
    {
        ret = mtk_9050_pwm_export_pin(data->pwm_dev_name, data->motor_cap[motor_id].motor_forward);
    }

    if (access(backward_pin_name, F_OK) != -1)
    {
        SE_INFO("Already export %d, ignore", data->motor_cap[motor_id].motor_backward);
    }
    else
    {
        ret += mtk_9050_pwm_export_pin(data->pwm_dev_name, data->motor_cap[motor_id].motor_backward);
    }

    return ret;
}

static SE_ret_t _MTK_9050_dc_motor_set_period(struct mtk_9050_dc_data *data, uint8_t motor_id, uint32_t period_us)
{
    if (!data->motor[motor_id].enable)
    {
        SE_set_error("Motor is not open to set period");
        return kSE_FAILED;
    }

    mtk_9050_pwm_set_period(data->pwm_dev_name, data->motor_cap[motor_id].motor_backward, period_us);
    mtk_9050_pwm_set_period(data->pwm_dev_name, data->motor_cap[motor_id].motor_forward, period_us);
    data->motor[motor_id].period_us = period_us;
    return kSE_SUCCESS;
}

static SE_ret_t _MTK_9050_dc_motor_open_motor(struct mtk_9050_dc_data *data, uint8_t motor_id)
{
    if (motor_id >= data->info.max_servo)
    {
        return kSE_OUT_OF_RANGE;
    }

    SE_ret_t ret = _MTK_9050_dc_motor_export_motor(data, motor_id);
    if (ret != kSE_SUCCESS)
    {
        return ret;
    }

    struct mtk_9050_dc_motor_info motor = {
        .duty_us = 0,
        .period_us = DEFAULT_MTK_9050_PERIOD_US,
        .enable = false,
        .pwm_resolution = PULSE_UNIT_US(DEFAULT_MTK_9050_PERIOD_US),
        .last_counter = 0,
        .delta_move = 0,
        .direction = 
    };

    data->motor[motor_id] = motor;
    data->motor[motor_id].enable = true;

    ret = _MTK_9050_dc_motor_set_period(data, motor_id, DEFAULT_MTK_9050_PERIOD_US);
    if (ret != kSE_SUCCESS)
    {
        return ret;
    }

    ret = mtk_9050_pwm_enable_pin(data->pwm_dev_name, data->motor_cap[motor_id].motor_forward);
    ret += mtk_9050_pwm_enable_pin(data->pwm_dev_name, data->motor_cap[motor_id].motor_backward);
    return ret;
}

static SE_ret_t MTK_9050_dc_motor_open_motor(struct SE_controller *controller, uint8_t motor_id)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    if (data->pwm_dev_name == NULL)
    {
        SE_set_error("The controller is not initialized");
        return kSE_TRY_AGAIN;
    }

    if (motor_id >= data->info.max_servo)
    {
        SE_set_error("Perform open servo out of range");
        return kSE_OUT_OF_RANGE;
    }

    return _MTK_9050_dc_motor_open_motor(data, motor_id);
}

static SE_ret_t _MTK_9050_dc_motor_close_motor(struct mtk_9050_dc_data *data, uint8_t motor_id)
{
    if (!data->motor[motor_id].enable)
    {
        SE_set_error("Motor is not open, ignore");
        SE_WARNING("Motor is not open, ignore", motor_id);
        return kSE_FAILED;
    }

    SE_ret_t ret = kSE_SUCCESS;
    ret = mtk_9050_pwm_unexport_pin(data->pwm_dev_name, data->motor_cap[motor_id].motor_forward);
    ret += mtk_9050_pwm_unexport_pin(data->pwm_dev_name, data->motor_cap[motor_id].motor_backward);
    return ret;
}

static SE_ret_t MTK_9050_dc_motor_close_motor(struct SE_controller *controller, uint8_t motor_id)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    if (data->pwm_dev_name == NULL)
    {
        SE_set_error("The controller is not initialized");
        return kSE_TRY_AGAIN;
    }

    if (motor_id >= data->info.max_servo)
    {
        SE_set_error("Perform open servo out of range");
        return kSE_OUT_OF_RANGE;
    }

    return _MTK_9050_dc_motor_close_motor(data, motor_id);
}

static uint8_t _MTK_9050_dc_duty_to_angle(uint32_t duty_us)
{
    return 0;
}

static SE_ret_t _MTK_9050_dc_motor_set_duty(struct mtk_9050_dc_data *data, uint8_t motor_id, uint32_t duty_us)
{
    if (!data->motor[motor_id].enable)
    {
        SE_set_error("Servo is not open to set duty");
        return kSE_FAILED;
    }

    mtk_9050_pwm_set_duty(data->pwm_dev_name, data->motor_cap[motor_id].motor_forward, duty_us);
    return kSE_SUCCESS;
}

static SE_ret_t MTK_9050_dc_motor_set_duty(struct SE_controller *controller, uint8_t motor_id, uint32_t duty)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    if (data->pwm_dev_name == NULL)
    {
        SE_set_error("The controller is not initialized");
        return kSE_TRY_AGAIN;
    }

    if (motor_id >= data->info.max_servo)
    {
        SE_set_error("Perform open servo out of range");
        return kSE_OUT_OF_RANGE;
    }

    return _MTK_9050_dc_motor_set_duty(data, motor_id, duty);
}

static SE_ret_t MTK_9050_dc_motor_set_period(struct SE_controller *controller, uint8_t motor_id, uint32_t period_us)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    if (data->pwm_dev_name == NULL)
    {
        SE_set_error("The controller is not initialized");
        return kSE_TRY_AGAIN;
    }

    if (motor_id >= data->info.max_servo)
    {
        SE_set_error("Perform open servo out of range");
        return kSE_OUT_OF_RANGE;
    }

    return _MTK_9050_dc_motor_set_period(data, motor_id, period_us);
}

static const struct SE_controller_info *MTK_9050_dc_motor_get_info_ref(struct SE_controller *controller)
{
    const struct SE_controller_info *ref_info = NULL;
    CONTROLLER_VALIDATE(controller, NULL);

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    ref_info = &data->info;
    return ref_info;
}

static struct SE_controller_info MTK_9050_dc_motor_get_info_copy(struct SE_controller *controller)
{
    struct SE_controller_info info = {0};
    CONTROLLER_VALIDATE(controller, info);

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    info = data->info;
    return info;
}

struct SE_controller *mtk_9050_dc_motor_get_controller(void)
{
    return &mtk_9050_dc_controller;
}

static SE_ret_t MTK_9050_dc_motor_set_id(struct SE_controller *controller, int id)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    data->info.id = id;
    return kSE_SUCCESS;
}

static uint32_t MTK_9050_dc_motor_get_pulse_resolution(struct SE_controller *controller, uint8_t motor_id)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)controller->controller_data;
    if (motor_id >= data->info.max_servo)
    {
        SE_set_error("Servo id is out of range");
        return 0;
    }

    return data->motor[motor_id].pwm_resolution;
}

static void _MTK_9050_servo_feedback_handle(struct mtk_9050_dc_data *data, SE_servo_t *servo)
{
    if (data->encoder_fd < 0)
    {
        return;
    }

    char *address = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, data->encoder_fd, 0);
    if (address == MAP_FAILED)
    {
        SE_WARNING("mmap operation failed");
        return;
    }

    struct encoder_data *encoder_data = (struct encoder_data *)address;
    SE_INFO("Encoder data is: %lld, %p\n", encoder_data->channel_a.a_forward, address);
    uint8_t delta = (encoder_data->channel_a.a_forward - data->motor[servo->id].last_counter);
    data->motor[servo->id].delta_move += delta;
    SE_INFO("Delta unit is : %d\n", delta);
    data->motor[servo->id].last_counter = encoder_data->channel_a.a_forward;
    munmap(address, 4096);
}

static void _MTK_9050_servo_update(SE_servo_t *servo)
{
    struct mtk_9050_dc_data *data = (struct mtk_9050_dc_data *)servo->controller->controller_data;
    if (data->motor_cap[servo->id].has_feedback)
    {
        _MTK_9050_servo_feedback_handle(data, servo);
    }
}

static SE_ret_t MTK_9050_dc_motor_servo_callback_register(void *servo)
{
    SE_ret_t ret = SE_servo_on_update((SE_servo_t *)servo, _MTK_9050_servo_update);
    return ret;
}