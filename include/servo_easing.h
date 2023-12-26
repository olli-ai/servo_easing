#ifndef SERVO_EASING_H
#define SERVO_EASING_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "SE_enum.h"
#include "SE_servo.h"
#include "SE_controller.h"

struct SE_controller *SE_open_controller(SE_supp_controller_t controller);
SE_ret_t SE_create_servo(SE_servo_t *new_servo, SE_argument_t args);
const char *SE_get_error(void);

#ifdef __cplusplus
}
#endif

#endif /*SERVO_EASING_H*/