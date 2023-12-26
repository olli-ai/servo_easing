#include "SE_errors.h"
#include "servo_easing.h"
#include "string.h"

static char msg_buffer[256] = {'\0'};

const char *SE_get_error()
{
    return msg_buffer;
}

void SE_set_error(const char *msg)
{
    strncpy(msg_buffer, msg, strlen(msg));
    msg_buffer[255] = '\0';
}