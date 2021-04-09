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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <errno.h>
#include <time.h>

#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/motor.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
const timerHardware_t timerHardware[1]; // unused

#include "drivers/accgyro/accgyro_fake.h"
#include "flight/imu.h"

#include "config/feature.h"
#include "config/config.h"
#include "scheduler/scheduler.h"

#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"

#include "fc/runtime_config.h"

#include "sitl2_state.h"

#include "uv.h"

#include "wmq_debug.h"
#include "wmq_error.h"
#include "libuv_compat.h"
#include "loop_utils.h"


uint32_t SystemCoreClock;

//static servo_packet pwmPkt;

#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)


// system
void systemInit(void) {

    WMQ_LOG_INFO("");

    SystemCoreClock = 500 * 1e6; // fake 500MHz

    simulator_state.start_hrtime = uv_hrtime();

    // serial can't been slow down
    rescheduleTask(TASK_SERIAL, 1);
}

void systemReset(void){
    WMQ_LOG_INFO("");
    sitl2_close_event_loop(&simulator_state);
    //exit(0);
}

void systemResetToBootloader(bootloaderRequestType_e requestType) {
    UNUSED(requestType);

    WMQ_LOG_INFO("");
    exit(0);
}

void timerInit(void) {
    WMQ_LOG_INFO("");
}

void timerStart(void) {
    WMQ_LOG_INFO("");
}

void failureMode(failureMode_e mode) {
    WMQ_LOG_INFO("%d", mode);
    exit(0);
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(repeatCount);
    WMQ_LOG_INFO("LED flash for failureMode: %d", mode);
}



//TODO: use something like yield/coroutines to run another uv_loop iteration
void delay(uint32_t ms)
{
    uv_sleep(ms);
}

void delayMicroseconds(timeUs_t us)
{
    UNUSED(us);
    uv_sleep(1);
}

//TODO: use float division 
//https://stackoverflow.com/questions/55832817/why-float-division-is-faster-than-integer-division-in-c

// uint64_t micros64(void) {
//     return sitl2_micros64(&simulator_state);
// }

uint32_t micros(void) {
    return sitl2_micros64(&simulator_state) & 0xFFFFFFFF;
}

// uint64_t millis64(void) {
//     return sitl2_millis64(&simulator_state);
// }

uint32_t millis(void) {
    return sitl2_millis64(&simulator_state) & 0xFFFFFFFF;
}


// ADC part
uint16_t adcGetChannel(uint8_t channel) {
    UNUSED(channel);
    return 0;
}




// stack part
char _estack;
char _Min_Stack_Size;

// fake EEPROM
static FILE *eepromFd = NULL;

void FLASH_Unlock(void) {
    if (eepromFd != NULL) {
        WMQ_LOG_DETAIL("eepromFd != NULL");
        return;
    }

    // open or create
    eepromFd = fopen(EEPROM_FILENAME,"rb+");
    if (eepromFd != NULL) {
        // obtain file size:
        fseek(eepromFd , 0 , SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);

        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n == lSize) {
            WMQ_LOG_INFO("loaded '%s', size = %zu / %zu", EEPROM_FILENAME, lSize, sizeof(eepromData));
        } else {
            WMQ_LOG_ERROR("failed to read '%s' %zu %zu", EEPROM_FILENAME, n, lSize);
            return;
        }
    } else {
        WMQ_LOG_INFO("created '%s', size = %zu", EEPROM_FILENAME, sizeof(eepromData));
        if ((eepromFd = fopen(EEPROM_FILENAME, "wb+")) == NULL) {
            WMQ_LOG_ERROR("failed to create '%s'", EEPROM_FILENAME);
            return;
        }
        if (fwrite(eepromData, sizeof(eepromData), 1, eepromFd) != 1) {
            WMQ_LOG_ERROR("write failed: %s", strerror(errno));
        }
    }
}

void FLASH_Lock(void) {
    // flush & close
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_SET);
        size_t written = fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
        WMQ_LOG_INFO("saved '%s' %zu", EEPROM_FILENAME, written);
    } else {
        WMQ_LOG_ERROR("eeprom is not unlocked");
    }
}

FLASH_Status FLASH_ErasePage(uintptr_t Page_Address) {
    //UNUSED(Page_Address);
    WMQ_LOG_DEBUG("%x", Page_Address);
    return FLASH_COMPLETE;
}

FLASH_Status FLASH_ProgramWord(uintptr_t addr, uint32_t value) {
    if ((addr >= (uintptr_t)eepromData) && (addr < (uintptr_t)ARRAYEND(eepromData))) {
        *((uint32_t*)addr) = value;
        WMQ_LOG_DEBUG("%p = %08x", (void*)addr, *((uint32_t*)addr));
    } else {
        WMQ_LOG_WARN("%p out of range", (void*)addr);
    }
    return FLASH_COMPLETE;
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);
    WMQ_LOG_DETAIL("");
}

void spektrumBind(rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);
    WMQ_LOG_DETAIL("");
}

void unusedPinsInit(void)
{
    WMQ_LOG_DETAIL("");
}
