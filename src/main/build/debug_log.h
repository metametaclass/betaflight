#pragma once

#include "platform.h"

#if defined(USE_DEBUG_LOG) && defined(SIMULATOR_BUILD)

#include <stdio.h>
//void debug_print(const char *fmt, ...);

#define SITL_DEBUG_LOG(message, ...) \
    do {\
        /*debug_print(__func__ " " message, __VA_ARGS__);*/\
        printf("%s: " message "\n", __func__, ## __VA_ARGS__);\
    } while(0)

#else
#define SITL_DEBUG_LOG(x)
#endif