#ifndef SE_CONTROLLER_H
#define SE_CONTROLLER_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "SE_enum.h"
#include "stdint.h"

struct SE_controller_info {
    const char *name;
    uint8_t id;
    uint8_t max_servo;
    uint32_t units_for_0_degree;
    uint32_t units_for_180_degree;
};

struct SE_controller {
    SE_ret_t (*controller_init)(struct SE_controller*);
    void (*controller_deinit)(struct SE_controller*);
    SE_ret_t (*open_servo)(struct SE_controller*, uint8_t servo_id);
    SE_ret_t (*close_servo)(struct SE_controller*, uint8_t servo_id);
    SE_ret_t (*set_duty)(struct SE_controller*, uint8_t servo_id, uint32_t duty_us);
    SE_ret_t (*set_period)(struct SE_controller*, uint8_t servo_id, uint32_t preiod_us);
    SE_ret_t (*set_id)(struct SE_controller*, int id);
    uint32_t (*get_pulse_resolution)(struct SE_controller*, uint8_t servo_id);
    const struct SE_controller_info *(*get_info_ref)(struct SE_controller*);
    struct SE_controller_info (*get_info_copy)(struct SE_controller*);
    void *controller_data;
};

SE_ret_t SE_controller_register(struct SE_controller *controller);
struct SE_controller* SE_controller_get(int controller_id);
int SE_controller_get_available_controller(struct SE_controller_info *info);
SE_ret_t SE_controller_init(struct SE_controller *controller);

#ifdef __cplusplus
}
#endif
#endif /*SE_CONTROLLER_H*/