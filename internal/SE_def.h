#ifndef SE_DEF_H
#define SE_DEF_H
#ifdef __cplusplus
extern "C"
}
#endif

#ifndef MAX_CONTROLLER
#define MAX_CONTROLLER      10
#endif /*MAX_CONTROLLER*/

#ifndef MAX_SERVO_INSTANCES
#define MAX_SERVO_INSTANCES 20
#endif /*MAX_SERVO_INSTANCES*/

#if !defined(USE_PRINTF_LOG) && !defined(USE_OLLI_LOG) && !defined(USE_NONE_LOG)
#define USE_NONE_LOG
#endif

#ifdef __cplusplus
}
#endif
#endif /*SE_DEF_H*/