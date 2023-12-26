#include "SE_controller.h"
#include "stdlib.h"
#include "SE_def.h"
#include "SE_logging.h"

static struct SE_controller *p_controller[MAX_CONTROLLER] = {0};

SE_ret_t SE_controller_register(struct SE_controller *controller)
{
    for (int i = 0; i < MAX_CONTROLLER; i++)
    {
        if (p_controller[i] == 0)
        {
            p_controller[i] = controller;
            controller->set_id(controller, i);
            return kSE_SUCCESS;
        }
    }
    return kSE_NO_MEM;
}

int SE_controller_get_available_controller(struct SE_controller_info *info)
{
    int j = 0;
    for (int i = 0; i < MAX_CONTROLLER; i++)
    {
        if (p_controller[i] != 0)
        {
            info[j] = p_controller[i]->get_info_copy(p_controller[i]);
            j++;
        }
    }
    return j;
}

SE_ret_t SE_controller_init(struct SE_controller *controller)
{
    return controller->controller_init(controller);
}

struct SE_controller* SE_controller_get(int id)
{
    if (id >= MAX_CONTROLLER)
    {
        SE_WARNING("id out of range %d > %d", id, MAX_CONTROLLER);
        return NULL;
    }

    return p_controller[id];
}