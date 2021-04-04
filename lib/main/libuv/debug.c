#include "debug.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <time.h>

//log level
static int g_log_level = LL_INFO;

//start of exe tickcount
static uint32_t g_start_tick = 0;

//debug file
static FILE *debug_file;

//debug flags
// use
#ifdef _WIN32
static int g_use_ods = 0;
#endif

static int g_use_stderr = 1;
static int g_show_pid = 0;
static int g_show_tid = 0;
static int g_show_time = 0;
static int g_show_ticks = 0;
static int g_use_file = 0;
static int g_show_loop_time = 0;

#ifdef _WIN32
static volatile __LONG32 g_last_time_update = 0xFFFFFFFF;
#endif

static char g_time[32];
static int g_time_size;

static volatile uint64_t g_counter = 0x0;
static volatile uint64_t g_loop_time = 0x0;

#ifdef _WIN32
//#include <windef.h>
#include <assert.h>
#include <windows.h>
typedef uint32_t ticks_t;
typedef uint32_t pid_t;

ticks_t getTick() {
    return GetTickCount();
}

pid_t getpid() {
    return GetCurrentProcessId();
}

pid_t gettid() {
    return GetCurrentThreadId();
}

#else
#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

typedef uint32_t ticks_t;

ticks_t getTick() {
    struct timespec ts;
    unsigned theTick = 0U;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    theTick = ts.tv_nsec / 1000000;
    theTick += ts.tv_sec * 1000;
    return theTick;
}

pid_t gettid() {
    return syscall(SYS_gettid);
}

#endif

void do_update_time(ticks_t ticks, int counter) {
    int r;
    time_t dest;
    struct tm tmdata;
#ifndef _WIN32
    struct tm *result;
#endif

    //char tmp[128];

    /*
    if(g_use_ods || g_use_stderr){
        sprintf(tmp, __FUNCTION__ "[%.*s][%8d] [%8d] [%8d]", g_time_size, g_time, GetCurrentThreadId(), ticks, counter);
        if(g_use_ods){
           OutputDebugStringA(tmp);
        }
        if(g_use_stderr){
            fprintf(stderr, __FUNCTION__ "[%.*s][%8d] [%8d] [%8d]\n", g_time_size, g_time, GetCurrentThreadId(), ticks, counter);
        }
    }
    */

    time(&dest);

#ifdef _WIN32
    r = gmtime_s(&tmdata, &dest);
    if (r) {
        //OutputDebugString(__FUNCTIONW__ L" gmtime_s error");
        goto error;
    }
#else
    result = gmtime_r(&dest, &tmdata);
    if (!result) {
        //OutputDebugString(__FUNCTIONW__ L" gmtime_s error");
        goto error;
    }
#endif

    r = strftime(g_time, sizeof(g_time), "%Y-%m-%dT%H:%M:%S", &tmdata);
    if (r <= 0) {
        if (g_use_stderr) {
            fprintf(stderr, "%s strftime error %d %s\n ", __func__, errno, strerror(errno));
        }
        //OutputDebugString(__FUNCTIONW__ L" strftime error");
        goto error;
    }
    g_time_size = r;
    return;
error:
    memset(g_time, '-', sizeof(g_time));
    g_time_size = sizeof(g_time) - 1;
    g_time[g_time_size] = '\0';
}

int debug_update_time_ticks(ticks_t ticks) {
#ifdef _WIN32
    DWORD current, seconds;
    int counter = 0;

    seconds = ticks / 100;  //tenths of seconds;

    do {
        //read current value;
        current = InterlockedCompareExchange(&g_last_time_update, 0, 0);
        //current = g_last_time_update;
        counter++;
        if (seconds == current) {
            return 0;  //do nothing
        }

    } while (InterlockedCompareExchange(&g_last_time_update, seconds, current) != current);

    do_update_time(ticks, counter);
    return 1;
#else
    do_update_time(ticks, 0);
    return 1;
#endif
}

int debug_update_time() {
    return debug_update_time_ticks(getTick() - g_start_tick);
}

int debug_set_counters(uint64_t counter, uint64_t time) {
    if (g_counter != counter && g_log_level < LL_DEBUG) {
        if (g_use_stderr) {
            fprintf(stderr, "--------------------------------------------------------------------------------------------------------------\n");
        }
        if (g_use_file && debug_file) {
            fprintf(debug_file, "--------------------------------------------------------------------------------------------------------------\n");
        }
    }
    g_counter = counter;
    g_loop_time = time;

    return 0;
}

//void debug_init_file();

void debug_set_level(int level, int log_options) {
    int use_stderr = (log_options & WMQ_LOG_OPTION_USE_STDERR) != 0;
    g_log_level = level;
#ifdef _WIN32
    g_use_ods = (log_options & WMQ_LOG_OPTION_USE_ODS) != 0;
#endif

    g_show_pid = (log_options & WMQ_LOG_OPTION_SHOW_PID) != 0;
    g_show_tid = (log_options & WMQ_LOG_OPTION_SHOW_TID) != 0;

    g_show_time = (log_options & WMQ_LOG_OPTION_SHOW_TIME) != 0;
    g_show_ticks = (log_options & WMQ_LOG_OPTION_SHOW_TICKS) != 0;

    g_use_file = (log_options & WMQ_LOG_OPTION_USE_FILE) != 0;

    g_show_loop_time = (log_options & WMQ_LOG_OPTION_SHOW_LOOP_TIME) != 0;

    memset(g_time, '-', sizeof(g_time));
    g_time_size = sizeof(g_time) - 1;
    g_time[g_time_size] = '\0';

    // if (g_use_file) {
    //     debug_init_file();
    // }

    g_start_tick = getTick();

    g_use_stderr = 0;

#ifdef _WIN32
    debug_print(LL_INFO, "debug_set_level: %8d %d ODS:%d stderr:%d %s", getpid(), level, g_use_ods, use_stderr, GetCommandLineA());
#else
    debug_print(LL_INFO, "debug_set_level: %8d %d stderr:%d", getpid(), level, use_stderr);
#endif
    g_use_stderr = use_stderr;

#ifdef _WIN32
    if (!(g_use_ods || g_use_stderr || g_use_file)) {
#else
    if (!(g_use_stderr || g_use_file)) {
#endif
        g_log_level = LL_NO_LOG;
    }
}

char *str_level(int level) {
    if (level < LL_TRACE) {
        return "VERBOS";
    }
    if (level < LL_DEBUG) {
        return "TRACE ";
    }
    if (level < LL_DETAIL) {
        return "DEBUG ";
    }
    if (level < LL_INFO) {
        return "DETAIL";
    }
    if (level == LL_INFO) {
        return "INFO  ";
    }
    if (level <= LL_WARN) {
        return "WARN  ";
    }
    if (level <= LL_ERROR) {
        return "ERROR ";
    }

    return "FATAL ";
}

void print_to_file(FILE *file, ticks_t ticks, pid_t pid, pid_t tid, char *level_s, char *buffer) {
    int c;

    if (g_show_loop_time) {
        fprintf(file, "[%8" PRId64 "][%8" PRId64 "]", g_counter, g_loop_time);
    }

    c = ((g_show_time) ? 8 : 0) + (g_show_ticks ? 4 : 0) + (g_show_pid ? 2 : 0) + (g_show_tid ? 1 : 0);
    switch (c) {
        case 0:
            fprintf(file, "[%s] %s\n", level_s, buffer);
            break;
        case 1:
            fprintf(file, "[%8u][%s] %s\n", tid, level_s, buffer);
            break;
        case 2:
            fprintf(file, "[%8u][%s] %s\n", pid, level_s, buffer);
            break;
        case 3:
            fprintf(file, "[%8u][%8u][%s] %s\n", pid, tid, level_s, buffer);
            break;
        case 4:
            fprintf(file, "[%8d][%s] %s\n", ticks, level_s, buffer);
            break;
        case 5:
            fprintf(file, "[%8d][%8d][%s] %s\n", ticks, tid, level_s, buffer);
            break;
        case 6:
            fprintf(file, "[%8d][%8d][%s] %s\n", ticks, pid, level_s, buffer);
            break;
        case 7:
            fprintf(file, "[%8d][%8d][%8d][%s] %s\n", ticks, pid, tid, level_s, buffer);
            break;
        case 8:
            fprintf(file, "[%.*s][%s] %s\n", g_time_size, g_time, level_s, buffer);
            break;
        case 9:
            fprintf(file, "[%.*s][%8d][%s] %s\n", g_time_size, g_time, tid, level_s, buffer);
            break;
        case 10:
            fprintf(file, "[%.*s][%8d][%s] %s\n", g_time_size, g_time, pid, level_s, buffer);
            break;
        case 11:
            fprintf(file, "[%.*s][%8d][%8d][%s] %s\n", g_time_size, g_time, pid, tid, level_s, buffer);
            break;
        case 12:
            fprintf(file, "[%.*s][%8d][%s] %s\n", g_time_size, g_time, ticks, level_s, buffer);
            break;
        case 13:
            fprintf(file, "[%.*s][%8d][%8d][%s] %s\n", g_time_size, g_time, ticks, tid, level_s, buffer);
            break;
        case 14:
            fprintf(file, "[%.*s][%8d][%8d][%s] %s\n", g_time_size, g_time, ticks, pid, level_s, buffer);
            break;
        case 15:
            fprintf(file, "[%.*s][%8d][%8d][%8d][%s] %s\n", g_time_size, g_time, ticks, pid, tid, level_s, buffer);
            break;
    }
}

void debug_print(int level, const char *fmt, ...) {
    int n, size = 100;
    va_list ap;
    char *buffer = NULL;
    //char* stderr_fmt;
    char *level_s = str_level(level);

    if (level < g_log_level) {
        return;
    }

    while (1) {
        char *newbuffer = (char *)realloc(buffer, size);
        if (newbuffer == NULL) {
            free(buffer);
            return;
        }
        buffer = newbuffer;
        va_start(ap, fmt);
#ifdef _WIN32
        n = vsnprintf_s(buffer, size, _TRUNCATE, fmt, ap);
#else
        n = vsnprintf(buffer, size, fmt, ap);
#endif
        //n = _vsn
        va_end(ap);
        if (n > -1 && n < size) {
            break;
        }
        if (n > -1)
            size = n + 1;
        else
            size *= 2;
    }
#ifdef _WIN32
    if (g_use_ods) {
        char *new_buffer = (char *)malloc(size + 20);  //[12345678][DETAIL] .... - 10+8+1+size+zero
        pid_t tid = gettid();
        if (new_buffer == NULL) {
            debug_print_no_memory(buffer);
            return;
        }
        memmove(new_buffer, buffer, size);
        sprintf(new_buffer, "[%8d][%s] %s", tid, level_s, buffer);
        OutputDebugStringA(new_buffer);
        free(new_buffer);
    }
#endif

    if (g_use_stderr | g_use_file) {
        ticks_t ticks = getTick() - g_start_tick;
        pid_t pid = g_show_pid ? getpid() : 0;
        pid_t tid = g_show_tid ? gettid() : 0;
        if (g_show_time) {
            debug_update_time_ticks(ticks);
        }
        if (g_use_stderr) {
            print_to_file(stderr, ticks, pid, tid, level_s, buffer);
        }
        if (g_use_file && debug_file) {
            print_to_file(debug_file, ticks, pid, tid, level_s, buffer);
        }
    }

    free(buffer);
}

const char no_memory[] = "no memory:";

void debug_print_no_memory(const char *msg) {
#ifdef _WIN32
    OutputDebugStringA("No memory");
    OutputDebugStringA(msg);
#endif
    if (g_use_stderr) {
        fwrite(no_memory, 1, strlen(no_memory), stderr);
        fwrite(msg, 1, strlen(msg), stderr);
        fwrite("\n", 1, 1, stderr);
        //fprintf(stderr, "%s\n", msg);
    }
}

#define HEX_BUFFER_SIZE 160
const char *debug_hex_chars = "0123456789ABCDEF";

void debug_print_hex(int level, const char *msg, void *data, size_t len, size_t address) {
    size_t pos = 0;
    size_t size;
    size_t start_offset;
    int i, c, buffer_pos, buffer_pos_char;
    char hex_buffer[HEX_BUFFER_SIZE];

    if (level < g_log_level) {
        return;
    }

    debug_print(level, "%s offset:%" PRIxPTR " data:%p len:%d", msg, address, data, len);

    memset(hex_buffer, 32, sizeof(hex_buffer));
    start_offset = address % 16;
    size = len + start_offset;

    //buffer_pos = sprintf_s(hex_buffer, HEX_BUFFER_SIZE, "%10X: ", address);
    //buffer_pos = _snprintf(hex_buffer, HEX_BUFFER_SIZE, "%10X: ", address);
    buffer_pos = sprintf(hex_buffer, "%10" PRIxPTR ": ", address);

    address = address - start_offset;
    buffer_pos_char = 63;
    i = 0;

    while (pos < size) {
        if (i == 16) {
            assert(buffer_pos_char < HEX_BUFFER_SIZE);
            hex_buffer[buffer_pos_char] = '\0';
            debug_print(level, "%s", hex_buffer);

            address += 16;
            //buffer_pos = sprintf_s(hex_buffer, HEX_BUFFER_SIZE, "%10X: ", address);
            //buffer_pos = _snprintf(hex_buffer, HEX_BUFFER_SIZE, "%10X: ", address);

            //TODO: check/limit buffer size
            buffer_pos = sprintf(hex_buffer, "%10" PRIxPTR ": ", address);
            buffer_pos_char = 63;
            i = 0;
        }
        if (i == 8) {
            hex_buffer[buffer_pos++] = '|';
        }

        if (pos < start_offset) {
            c = -1;
        } else {
            c = ((unsigned char *)data)[pos - start_offset];
        }
        assert(buffer_pos < HEX_BUFFER_SIZE);
        if (c < 0) {
            hex_buffer[buffer_pos++] = ' ';
            hex_buffer[buffer_pos++] = ' ';
            hex_buffer[buffer_pos++] = ' ';
            hex_buffer[buffer_pos_char++] = ' ';
        } else {
            hex_buffer[buffer_pos++] = debug_hex_chars[c >> 4];
            hex_buffer[buffer_pos++] = debug_hex_chars[c & 0x0F];
            hex_buffer[buffer_pos++] = ' ';
            if (c < 32) {
                hex_buffer[buffer_pos_char++] = '.';
            } else {
                hex_buffer[buffer_pos_char++] = c;
            }
        }

        i++;
        pos++;
    }

    if (i > 0) {
        while (i < 16) {
            hex_buffer[buffer_pos++] = ' ';
            hex_buffer[buffer_pos++] = ' ';
            hex_buffer[buffer_pos++] = ' ';
            hex_buffer[buffer_pos_char++] = ' ';
            i++;
        }
        assert(buffer_pos_char < HEX_BUFFER_SIZE);
        hex_buffer[buffer_pos_char] = '\0';
        debug_print(level, "%s", hex_buffer);
    }
}
