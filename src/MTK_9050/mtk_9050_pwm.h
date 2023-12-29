#ifndef MTK_9050_LINUX_PWM_H
#define MTK_9050_LINUX_PWM_H
#include <stdbool.h>
#include <stdint.h>

#include "SE_enum.h"

bool mtk_9050_pwm_is_device(const char *dev_folder);
SE_ret_t mtk_9050_pwm_export_pin(const char *dev_name, uint8_t pin);
SE_ret_t mtk_9050_pwm_unexport_pin(const char *dev_name, uint8_t pin);
SE_ret_t mtk_9050_pwm_set_duty(const char *dev_name, uint8_t pin, uint32_t duty_us);
SE_ret_t mtk_9050_pwm_set_period(const char *dev_name, uint8_t pin, uint32_t duty_us);
SE_ret_t mtk_9050_pwm_enable_pin(const char *dev_name, uint8_t pin);
SE_ret_t mtk_9050_pwm_disable_pin(const char *dev_name, uint8_t pin);
#endif /*MTK_9050_LINUX_PWM_H*/