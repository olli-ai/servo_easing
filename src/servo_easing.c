#include "servo_easing.h"

#include "stdint.h"
#include "stdlib.h"

#include "SE_controller.h"
#include "SE_errors.h"
#include "SE_logging.h"

#ifdef USE_DUMMY_CONTROLLER
#include "Dummy/dummy_controller.h"
#endif /*USE_DUMMY_CONTROLLER*/

#ifdef USE_PCA9685_CONTROLLER
#include "PCA9685/pca9685_controller.h"
#endif /*USE_PCA9685_CONTROLLER*/

#ifdef USE_PCA9685_LINUX_CONTROLLER
#include "PCA9685_linux/pca9685_linux_controller.h"
#endif /*USE_PCA9685_LINUX_CONTROLLER*/

#ifdef USE_MTK_9050_LINUX_CONTROLLER
#include "MTK_9050/mtk_9050_controller.h"
#endif /*USE_MTK_9050_LINUX_CONTROLLER*/

#ifdef USE_MTK_9050_DC_CONTROLLER
#include "MTK_9050/mtk_9050_dc_controller.h"
#endif /*USE_MTK_9050_DC_CONTROLLER*/


SE_ret_t SE_init(void)
{
    return kSE_SUCCESS;
}

struct SE_controller *SE_open_controller(SE_supp_controller_t controller)
{
    struct SE_controller *controller_instance = NULL;
    switch (controller)
    {
    case eSE_CONTROLLER_ONBOARD:
        break;

#ifdef USE_PCA9685_CONTROLLER
    case eSE_CONTROLLER_PCA9685:
        controller_instance = PCA9685_get_controller());
        break;
#endif /*USE_PCA9685_CONTROLLER*/

#ifdef USE_PCA9685_LINUX_CONTROLLER
    case eSE_CONTROLLER_LINUX_PCA9685:
        controller_instance = PCA9685_linux_get_controller();
        break;
#endif /*USE_PCA9685_LINUX_CONTROLLER*/

#ifdef USE_MTK_9050_LINUX_CONTROLLER
    case eSE_CONTROLLER_MTK_9050:
        controller_instance = mtk_9050_linux_get_controller();
        break;
#endif /*USE_MTK_9050_LINUX_CONTROLLER*/

#ifdef USE_MTK_9050_DC_CONTROLLER
    case eSE_CONTROLLER_DC_MTK_9050:
        controller_instance = mtk_9050_dc_motor_get_controller();
        break;
#endif

#ifdef USE_DUMMY_CONTROLLER
    case eSE_DUMMY_CONTROLLER:
        controller_instance = Dummy_get_controller();
        break;
#endif /*USE_DUMMY_CONTROLLER*/

    default:
        SE_WARNING("Controller %d is not supported by library", controller);
        break;
    }

    if (controller_instance != NULL)
    {
        SE_controller_register(controller_instance);
    }

    return controller_instance;
}

SE_ret_t SE_create_servo(SE_servo_t *new_inst, SE_argument_t args)
{
    struct SE_controller *controller = SE_controller_get(args.controller_id);
    SE_ret_t ret_code = kSE_SUCCESS;
    if (controller != NULL)
    {
        ret_code = controller->open_servo(controller, args.servo_id);
    }
    else
    {
        SE_set_error("Unable to get controller, controller is NULL");
        ret_code = kSE_FAILED;
    }

    switch (ret_code)
    {
    case kSE_OUT_OF_RANGE:
        SE_set_error("Servo is out of range");
        SE_ERROR("Servo id is out of range");
        break;
    case kSE_FAILED:
        SE_ERROR("Open servo failed");
        break;
    case kSE_SUCCESS:
        SE_INFO("Servo init for instance");
        ret_code = SE_servo_init(new_inst, &args);
        if (ret_code != kSE_SUCCESS)
        {
            break;
        }
        new_inst->id = args.servo_id;
        new_inst->controller = controller;
        controller->set_period(controller, args.servo_id, args.period_us);
        controller->register_servo_event((void*) new_inst);
        break;
    default:
        break;
    }
    return ret_code;
}