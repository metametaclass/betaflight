#pragma once

#include "platform.h"

#if defined(USE_DEBUG_LOG) && defined(SIMULATOR_BUILD)

#include "wmq_error.h"

#define SITL_DEBUG_LOG WMQ_LOG_DETAIL
#else
#define SITL_DEBUG_LOG(x)
#endif