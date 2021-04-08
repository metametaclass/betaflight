/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "fc/init.h"

#include "scheduler/scheduler.h"


#if defined(USE_LIBUV)

#include <stdio.h>

#define USING_UV_SHARED

#include <stdlib.h>
#include <uv.h>

#include "wmq_debug.h"
#include "libuv_compat.h"
#include "wmq_error.h"

#include "loop_utils.h"


//stdin read callback
static void on_tty_read(uv_stream_t* tty_in, ssize_t nread, const uv_buf_t* buf) {
    int rc;
    if (nread <= 0) {
        WMQ_LOG(LL_INFO, "EOF or error: %d", nread);
        close_all_handles(tty_in->loop);
    } else {
        debug_print_hex(LL_DEBUG, "", buf->base, nread, 0);
        rc = sitl2_parse_command_line(buf->base, nread);
        WMQ_CHECK_ERROR(rc, "sitl2_parse_command_line");
        if(rc){
            debug_print_hex(LL_WARN, "", buf->base, nread, 0);
        }
    }
}

//initialize stdin reader
int init_stdin(uv_loop_t* loop, uv_tty_t* tty) {
    int rc;
    //_open_osfhandle()

    rc = uv_tty_init(loop, tty, 0, 1);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_tty_init");

    rc = uv_read_start((uv_stream_t*)tty, on_alloc_buffer, on_tty_read);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_read_start");
    return 0;
}

//handle enumerator/visitor callback
void on_walk_handles(uv_handle_t* handle, void* arg) {
    UNUSED(arg);
    WMQ_LOG(LL_INFO, "%s", uv_handle_type_name(uv_handle_get_type(handle)));
    uv_close(handle, NULL);
}

void run_timer_cb(uv_timer_t *timer) {
    UNUSED(timer);
    //TODO: shift emulation time?
    scheduler();
    processLoopback();
}

static uint64_t hr_time;

void on_idle(uv_idle_t *idle) {
    UNUSED(idle);
    // uint64_t next_time = uv_hrtime();
    // if (next_time-hr_time>10000){
    //     scheduler();
    //     processLoopback();
    //     hr_time = next_time;
    // }
    scheduler();
}

int main(int argc, char** argv) {
    UNUSED(argc);
    UNUSED(argv);
    int rc;
    uv_tty_t tty = {0};
    uv_timer_t timer = {0};
    uv_idle_t idle = {0};

    debug_set_level(LL_DETAIL, WMQ_LOG_OPTION_USE_ODS | WMQ_LOG_OPTION_USE_STDERR | WMQ_LOG_OPTION_SHOW_TIME | WMQ_LOG_OPTION_SHOW_PID | WMQ_LOG_OPTION_SHOW_TID);
    WMQ_LOG(LL_INFO, "starting, sizeof(long unsigned int):%zu sizeof(int):%zu", sizeof(long unsigned int), sizeof(int));

    //init libuv message loop
    rc = uv_loop_init(&libuv_loop);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_loop_init");

    init();

    uv_timer_init(&libuv_loop, &timer);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_timer_init");

    uv_timer_start(&timer, run_timer_cb, 0, 1);

    //use idle handler for scheduler
    hr_time = uv_hrtime();

    rc = uv_idle_init(&libuv_loop, &idle);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_idle_init");

    rc = uv_idle_start(&idle, on_idle);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_idle_start");

    //init STDIN reader
    rc = init_stdin(&libuv_loop, &tty);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "init_stdin");

    rc = uv_run(&libuv_loop, UV_RUN_DEFAULT);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_run");

    uv_walk(&libuv_loop, on_walk_handles, NULL);

    //close message loop
    rc = uv_loop_close(&libuv_loop);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_loop_close");

    return 0;

}

#else
void run(void);

int main(void)
{
    init();

    run();

    return 0;
}

void FAST_CODE FAST_CODE_NOINLINE run(void)
{
    while (true) {
        scheduler();
        processLoopback();
#ifdef SIMULATOR_BUILD
        delayMicroseconds_real(50); // max rate 20kHz
#endif
    }
}
#endif