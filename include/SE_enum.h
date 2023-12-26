#ifndef SERVO_EASING_ENUM_H
#define SERVO_EASING_ENUM_H
typedef enum _servo_easing_ret {
    kSE_SUCCESS = 0,
    kSE_FAILED,
    kSE_NULL,
    kSE_OUT_OF_RANGE,
    kSE_NO_MEM,
    kSE_BUSY,
    kSE_TRY_AGAIN,
    kSE_NOT_SUPPORTED,
} SE_ret_t;

typedef enum _servo_easing_type {
    eSE_EASE_LINEAR = 0,
    eSE_EASE_QUARACTIC,
    eSE_EASE_CUBIC,
    eSE_EASE_QUARTIC,
    eSE_EASE_SINE,
    eSE_EASE_CIRCULAR,
    eSE_EASE_BACK,
    eSE_EASE_ELASTIC,
    eSE_EASE_BOUNCE,
    eSE_EASE_PRECISION,
    eSE_EASE_LAST,
} SE_easing_t;

typedef enum _servo_easing_mov {
    eSE_MOV_IN = 0,
    eSE_MOV_OUT,
    eSE_MOV_IN_OUT,
    eSE_MOV_BOUNCING_OUT_IN,
    eSE_MOV_LAST,
} SE_easing_mov_t;

typedef enum _supported_controller {
    eSE_CONTROLLER_ONBOARD = 0,
    eSE_CONTROLLER_PCA9685,
    eSE_CONTROLLER_LINUX_PCA9685,
    eSE_CONTROLLER_MTK_9050,
    eSE_DUMMY_CONTROLLER,
} SE_supp_controller_t;
#endif /*SERVO_EASING_ENUM_H*/
