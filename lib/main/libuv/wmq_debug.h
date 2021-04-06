#pragma once

#include <stddef.h>
#include <stdint.h>

#define LL_VERBOSE 1
#define LL_TRACE 5
#define LL_DEBUG 10
#define LL_DETAIL 20
#define LL_DETAILINFO 25
#define LL_INFO 30
#define LL_WARN 40
#define LL_ERROR 50
#define LL_FATAL 60
#define LL_NO_LOG 70;

#define WMQ_LOG_OPTION_USE_ODS 1
#define WMQ_LOG_OPTION_USE_STDERR 2
#define WMQ_LOG_OPTION_SHOW_PID 4
#define WMQ_LOG_OPTION_SHOW_TID 8
#define WMQ_LOG_OPTION_SHOW_TIME 16
#define WMQ_LOG_OPTION_SHOW_TICKS 32
#define WMQ_LOG_OPTION_USE_FILE 64
#define WMQ_LOG_OPTION_SHOW_LOOP_TIME 128

void debug_set_level(int level, int log_options);

void debug_init_from_config_inner();
void debug_init_from_config();

int debug_check_is_log(int level);

void debug_print(int level, const char *fmt, ...);

//void debug_print_w(int level, const wchar_t *fmt, ...);

void debug_print_no_memory(const char *msg);

int debug_update_time();

int debug_set_counters(uint64_t counter, uint64_t time);

void debug_print_hex(int level, const char *msg, void *data, size_t len, size_t start_offset);

void debug_init_error_handlers();

void debug_error_event(const char *name);

void debug_fflush();

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define WMQ_LOG_NOMEMORY(message)                                                         \
    do {                                                                                  \
        debug_print_no_memory(" File:" __FILE__ " Line:" TOSTRING(__LINE__) " " message); \
        debug_print_no_memory(__func__);                                                  \
    } while (0)

#define WMQ_LOG(level, message, ...)                                  \
    do {                                                              \
        debug_print((level), "%s " message, __func__, ##__VA_ARGS__); \
    } while (0)

#define WMQ_LOG2(level, message, ...)               \
    do {                                            \
        debug_print((level), message, __VA_ARGS__); \
    } while (0)

#define WMQ_ERROR(message, ...)                                                                                 \
    do {                                                                                                        \
        debug_error_event(__func__ " " message);                                                                \
        debug_print(LL_ERROR, __func__ " File:" __FILE__ " Line:" TOSTRING(__LINE__) " " message, __VA_ARGS__); \
    } while (0)

#define WMQ_LOGW(level, message, ...)                                  \
    do {                                                               \
        debug_print_w(level, __FUNCTIONW__ L" " message, __VA_ARGS__); \
    } while (0)

#define WMQ_LOG_HEX(level, msg, data, len, address)      \
    do {                                                 \
        debug_print_hex(level, msg, data, len, address); \
    } while (0)

#define WMQ_LOG_FULL(level, message, ...)                                                                    \
    do {                                                                                                     \
        debug_print(level, __func__ " " message " File:" __FILE__ " Line:" TOSTRING(__LINE__), __VA_ARGS__); \
    } while (0)

#define WMQ_NOT_IMPLEMENTED()                                                                           \
    do {                                                                                                \
        debug_print(LL_ERROR, __func__ " not implemented. File:" __FILE__ " Line:" TOSTRING(__LINE__)); \
        return UV_ENOSYS;                                                                               \
    } while (0)
