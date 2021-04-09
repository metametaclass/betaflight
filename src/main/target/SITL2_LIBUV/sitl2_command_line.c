#include "sitl2_command_line.h"

#include "scheduler/scheduler.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "common/maths.h"

#include "wmq_error.h"
#include "time.h"

//command line parser

typedef struct {
    sitl2_state_t *state;
    int is_exit;
    int watch_all;
    int watch_any;
    int watch_time;
    int watch_task_rate;
    int watch_arm_flags;
    int watch_motors;
} sitl2_cli_context_t;



void sitl2_status_print_time(sitl2_state_t *state){
    printf("System Uptime: %lu microseconds, %lu seconds, %lu scheduler calls, %.3f seconds in scheduler\n", sitl2_micros64(state),  sitl2_millis64(state) / 1000, state->scheduler_calls, state->scheduler_nanoseconds * 1e-9);
}

void sitl2_status_print_task_rate(){
    const int gyroRate = getTaskDeltaTimeUs(TASK_GYRO) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTimeUs(TASK_GYRO)));
    int rxRate = getCurrentRxRefreshRate();
    if (rxRate != 0) {
        rxRate = (int)(1000000.0f / ((float)rxRate));
    }
    const int systemRate = getTaskDeltaTimeUs(TASK_SYSTEM) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTimeUs(TASK_SYSTEM)));
    printf("CPU:%d%%, cycle time: %d, GYRO rate: %d, RX rate: %d, System rate: %d\n",
            constrain(getAverageSystemLoadPercent(), 0, 100), getTaskDeltaTimeUs(TASK_GYRO), gyroRate, rxRate, systemRate);
}

void sitl2_status_print_arm_flags(){
    printf("Arming disable flags:");
    armingDisableFlags_e flags = getArmingDisableFlags();
    while (flags) {
        const int bitpos = ffs(flags) - 1;
        flags &= ~(1 << bitpos);
        printf(" %s", armingDisableFlagNames[bitpos]);
    }
    printf("\n");
}

int sitl2_cli_STATUS(sitl2_cli_context_t *ctx){
    UNUSED(ctx);

    sitl2_status_print_time(ctx->state);

    sitl2_status_print_task_rate();

    // Battery meter
    //printf("Voltage: %d * 0.01V (%dS battery - %s)\n", getBatteryVoltage(), getBatteryCellCount(), getBatteryStateString());

    sitl2_status_print_arm_flags();
    return 0;
}

typedef struct status_watch_s {
    uv_timer_t timer;
    int active;
    int time;
    int task_rate;
    int arm_flags;
    int motors;
} status_watch_t;

static status_watch_t status_watch = { 0 };


void on_watch_timer(uv_timer_t *t){
    status_watch_t *watch = container_of(t, status_watch_t, timer);
    uv_loop_t *loop = t->loop;
    sitl2_state_t *state = container_of(loop, sitl2_state_t, loop);

    UNUSED(t);
    if(watch->time) {
        sitl2_status_print_time(state);
    }
    if(watch->task_rate) {
        sitl2_status_print_task_rate();
    }
    if(watch->arm_flags) {
        sitl2_status_print_arm_flags();
    }
}

int sitl2_cli_WATCH(sitl2_cli_context_t *ctx){
    int rc;
    uv_loop_t *loop = &ctx->state->loop;

    if(status_watch.active) {
        //printf("watch timer already active\n");
        WMQ_LOG_WARN("watch timer already active");
        return WMQE_INVALIDOP;
    }
    printf("starting watch timer\n");
    memset(&status_watch, 0, sizeof(status_watch));

    rc = uv_timer_init(loop, &status_watch.timer);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_timer_init");

    if (!ctx->watch_any) {
        //default
        status_watch.time = 1;
    } else {

        status_watch.time = ctx->watch_all || ctx->watch_time;
        status_watch.task_rate = ctx->watch_all || ctx->watch_task_rate;
        status_watch.motors = ctx->watch_all || ctx->watch_motors;
        status_watch.arm_flags = ctx->watch_all || ctx->watch_arm_flags;
    }

    rc = uv_timer_start(&status_watch.timer, on_watch_timer, 0, 1000);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_timer_start");
    status_watch.active = 1;

    return 0;
}

int sitl2_cli_WATCH_STOP(sitl2_cli_context_t *ctx){
    int rc;
    //uv_loop_t *loop = &ctx->state->loop;

    if(!status_watch.active){
        WMQ_LOG_WARN("watch timer inactive");
        return WMQE_INVALIDOP;
    }
    printf("stopping watch timer\n");
    rc = uv_timer_stop(&status_watch.timer);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_timer_stop");
    status_watch.active = 0;

    return 0;
}

int sitl2_cli_HELP(sitl2_cli_context_t *ctx){
    UNUSED(ctx);
    printf("command line usage:\n");
    printf(" h, help -> exit program\n");
    printf(" e, q, exit, quit -> exit program\n");
    printf(" st, status -> show betaflight status\n");
    printf(" w, watch[(all|*,time|t,motors|m,arm|a,...)] -> run watch timer. prints specified stats each 1 second\n");
    printf(" ws, watch_stop -> stop watch timer\n");
    return 0;
}

int sitl2_cli_EXIT(sitl2_cli_context_t *ctx){
    ctx->is_exit = 1;
    return 0;
}

#include "ragel_cli/sitl2_command_line.inc"

int sitl2_parse_command_line(sitl2_state_t *state, char *data, size_t size){
    sitl2_cli_context_t ctx = {0};
    ctx.state = state;

    int rc = parse_sitl2_cli_command(&ctx, data, size);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "parse_sitl2_cli_command");
    if(ctx.is_exit){
        WMQ_LOG_DETAIL("exiting");
        sitl2_close_event_loop(state);
    }
    return rc;
}
