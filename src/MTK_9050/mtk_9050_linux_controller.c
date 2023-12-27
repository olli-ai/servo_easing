#include "mtk_9050_linux_controller.h"

#include <stdbool.h>
#include <stdio.h>
#include <dirent.h>
#include <unistd.h>
#include <malloc.h>
#include <string.h>
#include <errno.h>

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

#define DEV_NAME "mstar,pwm"
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
    .servo = {{
        .duty_us = 0,
        .period_us = DEFAULT_MTK_9050_PERIOD_US,
        .enable = false,
        .pwm_resolution = PULSE_UNIT_US(DEFAULT_MTK_9050_PERIOD_US),
    }},
    .pin_map = {15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    .is_open = false,
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
    .controller_data = (void *)&controller_data,
};

static bool is_pca9685_device(const char *dev_folder)
{
    char file_name[128] = {'\0'};
    bool ret = false;
    sprintf(file_name, "/sys/class/pwm/%s/device/of_node/compatible", dev_folder);
    if (access(file_name, F_OK) == -1)
    {
        SE_DEBUG("Unable to open device name %s, ignore", file_name);
        return ret;
    }
    FILE *file = fopen(file_name, "r");
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    char *device_name = malloc(file_size + 1);
    if (device_name == NULL)
    {
        fclose(file);
        return ret;
    }
    fread(device_name, sizeof(char), file_size, file);
    for (int i = 0; i < file_size; i++)
    {
        if (device_name[i] == '\n' || device_name[i] == '\r')
        {
            device_name[i] = 0;
        }
    }

    fclose(file);
    if (!strcmp(device_name, DEV_NAME))
    {
        SE_DEBUG("Found %s", device_name);
        ret = true;
    }
    free(device_name);
    return ret;
}

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
        if (is_pca9685_device(entry->d_name))
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
    free(data->pwm_dev_name);
    data->pwm_dev_name = NULL;
    data->is_open = false;
}

static SE_ret_t _MTK_9050_linux_export_servo(struct mtk_9050_linux_data *data, uint8_t servo_id)
{
    char servo_name[256] = {'\0'};
    snprintf(servo_name, 256, "%s/pwm%d", data->pwm_dev_name, servo_id);
    SE_ret_t ret = kSE_SUCCESS;
    if (access(servo_name, F_OK) != -1)
    {
        SE_INFO("Already export, ignore");
    }
    else
    {
        char path_buffer[256] = {'\0'};
        snprintf(path_buffer, 256, "%s/export", data->pwm_dev_name);
        FILE *export = fopen(path_buffer, "w");
        if (export == NULL)
        {
            SE_set_error("Unable to open export path");
            SE_ERROR("Unable to open export path at %s", path_buffer);
            ret = kSE_FAILED;
        }
        else
        {
            int ret = fprintf(export, "%d", data->pin_map[servo_id]);
            if (ret < 0)
            {
                SE_WARNING("Open servo error = %d, error_msg = %s", errno, strerror(errno));
                ret = kSE_FAILED;
            }
            fclose(export);
        }
    }

    return ret;
}

static SE_ret_t _MTK_9050_linux_set_period(struct mtk_9050_linux_data *data, uint8_t servo_id, uint32_t period_us)
{
    if (!data->servo[servo_id].enable)
    {
        SE_set_error("Servo is not open to set period");
        return kSE_FAILED;
    }

    char servo_period_path[128] = "";
    snprintf(servo_period_path, 128, "%s/pwm%d/period", data->pwm_dev_name, data->pin_map[servo_id]);
    SE_DEBUG("Servo period path is %s", servo_period_path);
    FILE *servo_period_fd = fopen(servo_period_path, "w");
    if (servo_period_fd == NULL)
    {
        SE_set_error("Servo unable to open period fd");
        return kSE_FAILED;
    }

    int ret = fprintf(servo_period_fd, "%d", period_us);
    if (ret < 0)
    {
        SE_WARNING("Write period for servo error = %d, error_msg = %s", errno, strerror(errno));
    }
    fclose(servo_period_fd);
    SE_DEBUG("Period for servo %d set to %u", servo_id, period_us);
    data->servo[servo_id].period_us = period_us;
    return kSE_SUCCESS;
}


static SE_ret_t _MTK_9050_linux_open_servo(struct mtk_9050_linux_data *data, uint8_t servo_id)
{
    if (servo_id >= data->info.max_servo)
    {
        return kSE_OUT_OF_RANGE;
    }

    SE_ret_t ret = _MTK_9050_linux_export_servo(data, servo_id);
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

    ret = _MTK_9050_linux_set_period(data, servo_id, DEFAULT_MTK_9050_PERIOD_US);
    if (ret != kSE_SUCCESS)
    {
        return ret;
    }

    char path_buffer[256] = {0};
    snprintf(path_buffer, 256, "%s/pwm%d/enable", data->pwm_dev_name, data->pin_map[servo_id]);
    FILE *enable = fopen(path_buffer, "w");
    if (enable == NULL)
    {
        SE_ERROR("Unable to open enable path at %s", path_buffer);
        SE_set_error("Unable to open enable path");
        ret = kSE_FAILED;
    }
    else
    {
        int ret = fprintf(enable, "%d", 1);
        if (ret < 0)
        {
            SE_WARNING("Enable servo error = %d, error_msg = %s", errno, strerror(errno));
        }
        fclose(enable);
    }
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
    char unexport_path[256] = {'\0'};
    snprintf(unexport_path, 256, "%s/unexport", data->pwm_dev_name);
    FILE *unexport = fopen(unexport_path, "w");
    SE_ret_t ret = kSE_SUCCESS;
    if (unexport == NULL)
    {
        SE_set_error("Unable to open export path");
        ret = kSE_FAILED;
    }
    else
    {
        int ret = fprintf(unexport, "%d", data->pin_map[servo_id]);
        if (ret < 0)
        {
            SE_WARNING("Close servo error = %d, error_msg = %s", errno, strerror(errno));
        }
        else
        {
            data->servo[servo_id].enable = false;
        }
        fclose(unexport);
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

    char servo_duty_path[128] = "";
    snprintf(servo_duty_path, 128, "%s/pwm%d/duty_cycle", data->pwm_dev_name, data->pin_map[servo_id]);
    SE_DEBUG("Servo duty path is %s", servo_duty_path);
    FILE *servo_duty = fopen(servo_duty_path, "w");
    if (servo_duty == NULL)
    {
        SE_set_error("Servo unable to open duty fd");
        return kSE_FAILED;
    }

    int ret = fprintf(servo_duty, "%d", duty_us);
    if (ret < 0)
    {
        SE_WARNING("Write duty for servo error = %d, error_msg = %s", errno, strerror(errno));
    }

    fclose(servo_duty);
    SE_DEBUG("Duty servo %d set to %u us", servo_id, duty_us);
    data->servo[servo_id].duty_us = duty_us;
    return kSE_SUCCESS;
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