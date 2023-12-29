#include "mtk_9050_pwm.h"
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <malloc.h>
#include <string.h>

#include "SE_logging.h"
#include "SE_errors.h"

#define DEV_NAME "mstar,pwm"

bool mtk_9050_pwm_is_device(const char *dev_folder)
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

SE_ret_t mtk_9050_pwm_export_pin(const char *dev_folder, uint8_t pin)
{
    char servo_name[256] = {'\0'};
    snprintf(servo_name, 256, "%s/pwm%d", dev_folder, pin);
    SE_ret_t ret = kSE_SUCCESS;
    if (access(servo_name, F_OK) != -1)
    {
        SE_INFO("Already export, ignore");
    }
    else
    {
        char path_buffer[256] = {'\0'};
        snprintf(path_buffer, 256, "%s/export", dev_folder);
        FILE *export = fopen(path_buffer, "w");
        if (export == NULL)
        {
            SE_set_error("Unable to open export path");
            SE_ERROR("Unable to open export path at %s", path_buffer);
            ret = kSE_FAILED;
        }
        else
        {
            int ret = fprintf(export, "%d", pin);
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

SE_ret_t mtk_9050_pwm_enable_pin(const char *dev_name, uint8_t pin)
{
    char path_buffer[256] = {0};
    snprintf(path_buffer, 256, "%s/pwm%d/enable", dev_name, pin);
    FILE *enable = fopen(path_buffer, "w");
    SE_ret_t ret = kSE_SUCCESS;
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

SE_ret_t mtk_9050_pwm_disable_pin(const char *dev_name, uint8_t pin)
{
    char path_buffer[256] = {0};
    snprintf(path_buffer, 256, "%s/pwm%d/enable", dev_name, pin);
    FILE *enable = fopen(path_buffer, "w");
    SE_ret_t ret = kSE_SUCCESS;
    if (enable == NULL)
    {
        SE_ERROR("Unable to open enable path at %s", path_buffer);
        SE_set_error("Unable to open enable path");
        ret = kSE_FAILED;
    }
    else
    {
        int ret = fprintf(enable, "%d", 0);
        if (ret < 0)
        {
            SE_WARNING("Enable servo error = %d, error_msg = %s", errno, strerror(errno));
        }
        fclose(enable);
    }
    return ret;
}

SE_ret_t mtk_9050_pwm_unexport_pin(const char *dev_name, uint8_t pin)
{
    char unexport_path[256] = {'\0'};
    snprintf(unexport_path, 256, "%s/unexport", dev_name);
    FILE *unexport = fopen(unexport_path, "w");
    SE_ret_t ret = kSE_SUCCESS;
    if (unexport == NULL)
    {
        SE_set_error("Unable to open export path");
        ret = kSE_FAILED;
    }
    else
    {
        int ret = fprintf(unexport, "%d", pin);
        if (ret < 0)
        {
            SE_WARNING("Close servo error = %d, error_msg = %s", errno, strerror(errno));
        }
        fclose(unexport);
    }
    return ret;
}

SE_ret_t mtk_9050_pwm_set_duty(const char *dev_name, uint8_t pin, uint32_t duty_us)
{
    char pwm_duty_path[128] = "";
    snprintf(pwm_duty_path, 128, "%s/pwm%d/duty_cycle", dev_name, pin);
    SE_DEBUG("Servo duty path is %s", pwm_duty_path);
    FILE *pwm_duty = fopen(pwm_duty_path, "w");
    if (pwm_duty == NULL)
    {
        SE_set_error("Servo unable to open duty fd");
        return kSE_FAILED;
    }

    int ret = fprintf(pwm_duty, "%d", duty_us);
    if (ret < 0)
    {
        SE_WARNING("Write duty for pwm pin %d error = %d, error_msg = %s", pin, errno, strerror(errno));
    }

    fclose(pwm_duty);
    SE_DEBUG("Duty pin %d set to %u us", pin, duty_us);
    return kSE_SUCCESS;
}

SE_ret_t mtk_9050_pwm_set_period(const char *dev_name, uint8_t pin, uint32_t period_us)
{
    char pwm_period_path[128] = "";
    snprintf(pwm_period_path, 128, "%s/pwm%d/period", dev_name, pin);
    SE_DEBUG("Servo period path is %s", pwm_period_path);
    FILE *pwm_period_fd = fopen(pwm_period_path, "w");
    if (pwm_period_fd == NULL)
    {
        SE_set_error("Servo unable to open period fd");
        return kSE_FAILED;
    }

    int ret = fprintf(pwm_period_fd, "%d", period_us);
    if (ret < 0)
    {
        SE_WARNING("Write period for servo error = %d, error_msg = %s", errno, strerror(errno));
    }
    fclose(pwm_period_fd);
    SE_DEBUG("Period for servo %d set to %u", pin, period_us);
    return kSE_SUCCESS;
}