#include "pca9685_controller.h"

#define PCA9685_MAX_SERVO 16
#define DEFAULT_PCA9685_UNITS_FOR_0_DEGREE    111 // 111.411 = 544 us
#define DEFAULT_PCA9685_UNITS_FOR_45_DEGREE  (111 + ((491 - 111) / 4)) // 206
#define DEFAULT_PCA9685_UNITS_FOR_90_DEGREE  (111 + ((491 - 111) / 2)) // 301 = 1472 us
#define DEFAULT_PCA9685_UNITS_FOR_135_DEGREE (491 - ((491 - 111) / 4)) // 369
#define DEFAULT_PCA9685_UNITS_FOR_180_DEGREE  491 // 491.52 = 2400 us

#define PULSE_UNIT_US(period_us) ((period_us) / 4096)
#define DEFAULT_DMMY_PERIOD_US      (20000)

static SE_ret_t PCA9685_init_device(struct SE_controller *controller);
static void PCA9685_deinit_device(struct SE_controller *controller);
static SE_ret_t PCA9685_open_servo(struct SE_controller *controller, uint8_t servo_id);
static SE_ret_t PCA9685_close_servo(struct SE_controller *controller, uint8_t servo_id);
static SE_ret_t PCA9685_set_duty(struct SE_controller *controller, uint8_t servo_id, uint32_t duty_us);
static SE_ret_t PCA9685_set_period(struct SE_controller *controller, uint8_t servo_id, uint32_t period_us);
static const struct SE_controller_info *PCA9685_get_info_ref(struct SE_controller *controller);
static struct SE_controller_info PCA9685_get_info_copy(struct SE_controller *controller);
static SE_ret_t PCA9685_set_id(struct SE_controller *controller, int id);
static float PCA9685_get_pulse_resolution(struct SE_controller *controller, uint8_t servo_id);

struct pca9685_servo_info
{
    uint8_t enable;
    uint32_t period_us;
    uint32_t duty;
    uint32_t pwm_resolution;
};

struct pca9685_data
{
    struct SE_controller_info info;
    struct pca9685_servo_info servo[PCA9685_MAX_SERVO];
    bool is_open;
};

static struct pca9685_data controller_data = {
    .info = {
        .name = "PCA9685 servo controller",
        .id = 0,
        .max_servo = PCA9685_MAX_SERVO,
        .units_for_0_degree = DEFAULT_PCA9685_UNITS_FOR_0_DEGREE,
        .units_for_180_degree = DEFAULT_PCA9685_UNITS_FOR_180_DEGREE,
    },
    .servo = {0},
    .is_open = false,
};

static struct SE_controller pca9685_controller = {
    .controller_init = PCA9685_init_device,
    .controller_deinit = PCA9685_deinit_device,
    .open_servo = PCA9685_open_servo,
    .close_servo = PCA9685_close_servo,
    .set_duty = PCA9685_set_duty,
    .set_period = PCA9685_set_period,
    .get_info_ref = PCA9685_get_info_ref,
    .get_info_copy = PCA9685_get_info_copy,
    .set_id = PCA9685_set_id,
    .get_pulse_resolution = PCA9685_get_pulse_resolution,
    .controller_data = (void *)&controller_data,
};

static SE_ret_t PCA9685_init_device(struct SE_controller *controller)
{
    return kSE_SUCCESS;
}

static SE_ret_t PCA9685_open_servo(struct SE_controller *controller, uint8_t servo_id)
{
    return kSE_SUCCESS;
}

static SE_ret_t PCA9685_close_servo(struct SE_controller *controller, uint8_t servo_id)
{
    return kSE_SUCCESS;
}

static SE_ret_t PCA9685_set_duty(struct SE_controller *controller, uint8_t servo_id, uint32_t duty)
{
    return kSE_SUCCESS;
}

static SE_ret_t PCA9685_set_period(struct SE_controller *controller, uint8_t servo_id, uint32_t freq)
{
    return kSE_SUCCESS;
}

static SE_ret_t PCA9685_get_info_ref(struct SE_controller *controller)
{
    return kSE_SUCCESS;
}

static SE_ret_t PCA9685_get_info_copy(struct SE_controller *controller)
{
    return kSE_SUCCESS;
}

struct SE_controller* PCA9685_get_controller(void)
{
    return &pca9685_controller;
}

static SE_ret_t PCA9685_set_id(struct SE_controller *controller, int id)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);

    struct pca9685_data *data = (struct pca9685_data *)controller->controller_data;
    data->id = id;
    return kSE_SUCCESS;
}

static uint32_t PCA9685_get_pulse_resolution(struct SE_controller *controller, uint8_t servo_id)
{
    CONTROLLER_VALIDATE(controller, kSE_NULL);
    struct pca9685_data *data = (struct pca9685_data *)controller->controller_data;

    if (servo_id >= data->info.max_servo)
    {
        SE_set_error("Servo id is out of range");
        return 0;
    }

    return data->servo[servo_id].pwm_resolution;
}