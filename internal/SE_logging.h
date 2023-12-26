#ifndef SE_LOGGING_H
#define SE_LOGGING_H
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef USE_NONE_LOG
#define SE_DEBUG(fmt, ...)
#define SE_INFO(fmt, ...)
#define SE_WARNING(fmt, ...)
#define SE_ERROR(fmt, ...)
#endif /*USE_NONE_LOG*/

#ifdef USE_PRINTF_LOG
#include "log.h"

#define SE_DEBUG(fmt, ...)      log_debug(fmt, ##__VA_ARGS__)
#define SE_INFO(fmt, ...)       log_info(fmt, ##__VA_ARGS__)
#define SE_WARNING(fmt, ...)    log_warn(fmt, ##__VA_ARGS__)
#define SE_ERROR(fmt, ...)      log_error(fmt, ##__VA_ARGS__)
#endif /*USE_PRINTF_LOG*/

#ifdef USE_OLLI_LOG
#include "olli_syslog.h"

#define SE_DEBUG(...)      OLLI_LOG(OLLI_DEBUG, ##__VA_ARGS__)
#define SE_INFO(...)       OLLI_LOG(OLLI_INFO, ##__VA_ARGS__)
#define SE_WARNING(...)    OLLI_LOG(OLLI_WARNING, ##__VA_ARGS__)
#define SE_ERROR(...)      OLLI_LOG(OLLI_ERR, ##__VA_ARGS__)
#endif /*USE_PRINTF_LOG*/

#ifdef __cplusplus
}
#endif

#endif /*SE_LOGGING_H*/