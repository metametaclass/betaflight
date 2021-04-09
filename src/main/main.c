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
//#include <uv.h>

#include "wmq_debug.h"
//#include "libuv_compat.h"
#include "wmq_error.h"

//#include "loop_utils.h"
#include "sitl2_state.h"



int main(int argc, char** argv) {
    UNUSED(argc);
    UNUSED(argv);
    int rc;

    debug_set_level(LL_DETAIL, WMQ_LOG_OPTION_USE_ODS | WMQ_LOG_OPTION_USE_STDERR | WMQ_LOG_OPTION_SHOW_TIME | WMQ_LOG_OPTION_SHOW_PID | WMQ_LOG_OPTION_SHOW_TID);
    WMQ_LOG(LL_INFO, "starting, sizeof(long unsigned int):%zu sizeof(int):%zu", sizeof(long unsigned int), sizeof(int));

    rc = sitl2_init(&simulator_state);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "sitl2_init");

    init();

    rc = sitl2_run(&simulator_state);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_run");

    rc = sitl2_finit(&simulator_state);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_run");

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