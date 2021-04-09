#include "wmq_debug.h"

#include "drivers/pwm_output.h"
#include "config/feature.h"

#include "sitl2_state.h"

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
    for(int i=0; i<MAX_SUPPORTED_MOTORS; i++){
        simulator_state.motor_speed[i] = motorsPwm[i];
    }

    /*
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
    */
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
