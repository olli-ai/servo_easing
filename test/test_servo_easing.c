#include <unistd.h>
#include <stdbool.h>

#include "servo_easing.h"
#include "SE_ticks.h"
#include "SE_logging.h"

static int last_angle = 180;

static void on_destination_reach_cb(SE_servo_t *servo)
{
    SE_DEBUG("destination reach callback");
    if (last_angle == 180)
    {
        last_angle = 0;
    } else if (last_angle == 0)
    {
        last_angle = 180;
    }

    SE_servo_set_angle(servo, last_angle);
    SE_servo_start(servo);
}

int main()
{
    struct SE_controller *controller = SE_open_controller(eSE_DUMMY_CONTROLLER);
    if (controller == NULL)
    {
        SE_WARNING("%s", "Unable to open controller");
        return -1;
    }
    SE_DEBUG("Open controller sucessful at %p", controller);

    SE_ret_t ret = SE_controller_init(controller);
    if (ret != kSE_SUCCESS)
    {
        SE_ERROR("Init controller failed, error %s", SE_get_error());
        return -1;
    }

    SE_argument_t servo_args = {
        .controller_id = controller->get_info_ref(controller)->id,
        .easing_type = eSE_EASE_QUARACTIC,
        .move_type = eSE_MOV_IN_OUT,
        .servo_id = 0,
        .speed = 80,
        .period_us = 40000,
        .init_angle = 70,
    };

    SE_servo_t servo;
    SE_DEBUG("Create servo");
    SE_create_servo(&servo, servo_args);
    SE_DEBUG("Set easing to %d deg", 0);
    SE_servo_set_angle(&servo, 0);
    SE_servo_on_destination_reach(&servo, on_destination_reach_cb);
    ret = SE_servo_start(&servo);
    if (ret != kSE_SUCCESS)
    {
        SE_WARNING("Start easing error %s", SE_get_error());
    } else
    {
        SE_DEBUG("%s", "Servo start easing");
    }
    int milis_to_move = SE_servo_get_milis_to_complete_move(&servo);

    do
    {
        SE_tick_update(10);
        SE_servo_update(&servo);
        usleep(10000);
        milis_to_move -= 10;
    } while ((milis_to_move) > 0);
    return 0;
}