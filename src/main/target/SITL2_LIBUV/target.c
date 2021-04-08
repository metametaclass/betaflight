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

#include "uv.h"

#include "wmq_debug.h"
#include "wmq_error.h"
#include "libuv_compat.h"
#include "loop_utils.h"

#include "fc/runtime_config.h"

uv_loop_t libuv_loop;

uint32_t SystemCoreClock;

static servo_packet pwmPkt;

//static struct timespec start_time;
//static double simRate = 1.0;

#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)

static uint64_t start_hrtime;

// system
void systemInit(void) {

    WMQ_LOG_INFO("");

    SystemCoreClock = 500 * 1e6; // fake 500MHz

    start_hrtime = uv_hrtime();
    //clock_gettime(CLOCK_MONOTONIC, &start_time);

    // serial can't been slow down
    rescheduleTask(TASK_SERIAL, 1);
}

void systemReset(void){
    WMQ_LOG_INFO("");
    close_all_handles(&libuv_loop);
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

/*
// Time part
// Thanks ArduPilot
uint64_t nanos64_real() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec*1e9 + ts.tv_nsec) - (start_time.tv_sec*1e9 + start_time.tv_nsec);
}

uint64_t micros64_real() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint64_t millis64_real() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint64_t micros64() {
    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out*1e-3;
//    return micros64_real();
}

uint64_t millis64() {
    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out*1e-6;
//    return millis64_real();
}

uint32_t micros(void) {
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis(void) {
    return millis64() & 0xFFFFFFFF;
}

void microsleep(uint32_t usec) {
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}

void delayMicroseconds(uint32_t us) {
    microsleep(us / simRate);
}

void delayMicroseconds_real(uint32_t us) {
    microsleep(us);
}

void delay(uint32_t ms) {
    uint64_t start = millis64();

    while ((millis64() - start) < ms) {
        microsleep(1000);
    }
}
*/

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

uint32_t micros(void) {
    uint64_t hrtime = uv_hrtime();
    return ((hrtime - start_hrtime)/1000) & 0xFFFFFFFF;
}

uint32_t millis(void) {
    uint64_t hrtime = uv_hrtime();
    return ((hrtime - start_hrtime)/1000000) & 0xFFFFFFFF;
}

// PWM part
pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

// real value to send
static int16_t motorsPwm[MAX_SUPPORTED_MOTORS];
static int16_t servosPwm[MAX_SUPPORTED_SERVOS];
static int16_t idlePulse;

void servoDevInit(const servoDevConfig_t *servoConfig) {
    UNUSED(servoConfig);
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        servos[servoIndex].enabled = true;
    }
}

static motorDevice_t motorPwmDevice; // Forward

pwmOutputPort_t *pwmGetMotors(void) {
    return motors;
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static void pwmDisableMotors(void)
{
    WMQ_LOG_DETAIL("");
    motorPwmDevice.enabled = false;
}

static bool pwmEnableMotors(void)
{
    WMQ_LOG_DETAIL("");
    motorPwmDevice.enabled = true;
    return true;
}

static void pwmWriteMotor(uint8_t index, float value)
{
    motorsPwm[index] = value - idlePulse;
}

static void pwmWriteMotorInt(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, (float)value);
}

static void pwmShutdownPulsesForAllMotors(void)
{
    WMQ_LOG_DETAIL("");
    motorPwmDevice.enabled = false;
}

bool pwmIsMotorEnabled(uint8_t index) {
    return motors[index].enabled;
}

static void pwmCompleteMotorUpdate(void)
{
    // send to simulator
    // for gazebo8 ArduCopterPlugin remap, normal range = [0.0, 1.0], 3D rang = [-1.0, 1.0]

    double outScale = 1000.0;
    if (featureIsEnabled(FEATURE_3D)) {
        outScale = 500.0;
    }

    pwmPkt.motor_speed[3] = motorsPwm[0] / outScale;
    pwmPkt.motor_speed[0] = motorsPwm[1] / outScale;
    pwmPkt.motor_speed[1] = motorsPwm[2] / outScale;
    pwmPkt.motor_speed[2] = motorsPwm[3] / outScale;

    //WMQ_LOG_DETAIL("%u:%u,%u,%u,%u", idlePulse, motorsPwm[0], motorsPwm[1], motorsPwm[2], motorsPwm[3]);
}

void pwmWriteServo(uint8_t index, float value) {
    servosPwm[index] = value;
}

static motorDevice_t motorPwmDevice = {
    .vTable = {
        .postInit = motorPostInitNull,
        .convertExternalToMotor = pwmConvertFromExternal,
        .convertMotorToExternal = pwmConvertToExternal,
        .enable = pwmEnableMotors,
        .disable = pwmDisableMotors,
        .isMotorEnabled = pwmIsMotorEnabled,
        .updateStart = motorUpdateStartNull,
        .write = pwmWriteMotor,
        .writeInt = pwmWriteMotorInt,
        .updateComplete = pwmCompleteMotorUpdate,
        .shutdown = pwmShutdownPulsesForAllMotors,
    }
};

motorDevice_t *motorPwmDevInit(const motorDevConfig_t *motorConfig, uint16_t _idlePulse, uint8_t motorCount, bool useUnsyncedPwm)
{
    UNUSED(motorConfig);
    UNUSED(useUnsyncedPwm);

    if (motorCount > 4) {
        return NULL;
    }

    idlePulse = _idlePulse;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        motors[motorIndex].enabled = true;
    }
    motorPwmDevice.count = motorCount; // Never used, but seemingly a right thing to set it anyways.
    motorPwmDevice.initialized = true;
    motorPwmDevice.enabled = false;

    return &motorPwmDevice;
}

// ADC part
uint16_t adcGetChannel(uint8_t channel) {
    UNUSED(channel);
    return 0;
}

//command line parser

typedef struct {
    int is_exit;
} sitl2_cli_context_t;

int sitl2_cli_STATUS(sitl2_cli_context_t *ctx){
    printf("System Uptime: %d seconds\n", millis() / 1000);


    const int gyroRate = getTaskDeltaTimeUs(TASK_GYRO) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTimeUs(TASK_GYRO)));
    int rxRate = getCurrentRxRefreshRate();
    if (rxRate != 0) {
        rxRate = (int)(1000000.0f / ((float)rxRate));
    }
    const int systemRate = getTaskDeltaTimeUs(TASK_SYSTEM) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTimeUs(TASK_SYSTEM)));
    printf("CPU:%d%%, cycle time: %d, GYRO rate: %d, RX rate: %d, System rate: %d\n",
            constrain(getAverageSystemLoadPercent(), 0, 100), getTaskDeltaTimeUs(TASK_GYRO), gyroRate, rxRate, systemRate);

    // Battery meter
    //printf("Voltage: %d * 0.01V (%dS battery - %s)\n", getBatteryVoltage(), getBatteryCellCount(), getBatteryStateString());

    printf("Arming disable flags:");
    armingDisableFlags_e flags = getArmingDisableFlags();
    while (flags) {
        const int bitpos = ffs(flags) - 1;
        flags &= ~(1 << bitpos);
        printf(" %s", armingDisableFlagNames[bitpos]);
    }
    printf("\n");
    return 0;
}

int sitl2_cli_EXIT(sitl2_cli_context_t *ctx){
    ctx->is_exit = 1;
    return 0;
}

#include "ragel_cli/sitl2_command_line.inc"

int sitl2_parse_command_line(char *data, size_t size){
    sitl2_cli_context_t ctx = {0};
    int rc = parse_sitl2_cli_command(&ctx, data, size);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "parse_sitl2_cli_command");
    if(ctx.is_exit){
        WMQ_LOG_DETAIL("exiting");
        close_all_handles(&libuv_loop);
    }
    return rc;
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
