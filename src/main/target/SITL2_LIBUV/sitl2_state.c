#include "wmq_error.h"
#include "libuv_compat.h"
#include "loop_utils.h"

#include "sitl2_state.h"
#include "sitl2_command_line.h"

#include "scheduler/scheduler.h"
#include "fc/init.h"



sitl2_state_t simulator_state = { 0 };


//stdin read callback
static void on_tty_read(uv_stream_t* tty_in, ssize_t nread, const uv_buf_t* buf) {
    uv_loop_t *loop = tty_in->loop;
    sitl2_state_t *state = container_of(loop, sitl2_state_t, loop);

    int rc;
    if (nread <= 0) {
        WMQ_LOG_WARN("EOF or error: %d", nread);
        close_all_handles(tty_in->loop);
    } else {
        debug_print_hex(LL_DEBUG, "", buf->base, nread, 0);
        rc = sitl2_parse_command_line(state, buf->base, nread);
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
    WMQ_LOG_INFO("%s", uv_handle_type_name(uv_handle_get_type(handle)));
    uv_close(handle, NULL);
}

void run_timer_cb(uv_timer_t *timer) {
    uv_loop_t *loop = timer->loop;
    sitl2_state_t *state = container_of(loop, sitl2_state_t, loop);

    if(state->is_simulated_time){
        state->sim_timer_calls++;
        uint64_t start = uv_hrtime();
        for(uint32_t i=0; i<state->steps_count; i++) {
            state->sim_time_ns += state->sim_time_step_ns;
            sitl2_scheduler_with_stats(state);
        }
        uint64_t end = uv_hrtime();
        state->sim_timer_ns += (end-start);
    }else{
        sitl2_scheduler_with_stats(state);
        processLoopback();
    }
}


void on_idle(uv_idle_t *idle) {
    uv_loop_t *loop = idle->loop;
    sitl2_state_t *state = container_of(loop, sitl2_state_t, loop);
    sitl2_scheduler_with_stats(state);
}


int sitl2_init(sitl2_state_t *state) {
    int rc;

    //init libuv message loop
    rc = uv_loop_init(&state->loop);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_loop_init");

    rc = uv_timer_init(&state->loop, &state->timer);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_timer_init");

    rc = uv_timer_start(&state->timer, run_timer_cb, 0, 1);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_timer_start");

    //use idle handler for scheduler
    rc = uv_idle_init(&state->loop, &state->idle);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_idle_init");

    rc = uv_idle_start(&state->idle, on_idle);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_idle_start");

    //init STDIN reader
    rc = init_stdin(&state->loop, &state->tty);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "init_stdin");

    state->start_hrtime = uv_hrtime();

    return 0;
}

int sitl2_finit(sitl2_state_t *state){
    int rc;

    uv_walk(&state->loop, on_walk_handles, NULL);

    //close message loop
    rc = uv_loop_close(&state->loop);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_loop_close");

    return 0;

}

int sitl2_run(sitl2_state_t *state){
    int rc;

    rc = uv_run(&state->loop, UV_RUN_DEFAULT);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_run");

    return 0;
}

void sitl2_close_event_loop(sitl2_state_t *state){
    close_all_handles(&state->loop);
}


//current time in nanoseconds
uint64_t sitl2_current_time_ns(sitl2_state_t *state){
    if(state->is_simulated_time){
        return state->sim_time_ns;
    }else{
        return uv_hrtime() - state->start_hrtime;
    }
}

//current time in microseconds
int64_t sitl2_current_time_us(sitl2_state_t *state){
    return sitl2_current_time_ns(state) / 1000;
}

//current time in milliseconds
int64_t sitl2_current_time_ms(sitl2_state_t *state){
    return sitl2_current_time_ns(state) / 1000000;
}


//scheduler call with stats
void sitl2_scheduler_with_stats(sitl2_state_t *state){
    uint64_t begin = uv_hrtime();
    scheduler();
    uint64_t end = uv_hrtime();
    state->scheduler_nanoseconds += (end-begin);
    state->scheduler_calls++;
}


//start working in simulated time
int sitl2_start_simulated_time(sitl2_state_t *state){
    if(state->is_simulated_time){
        WMQ_LOG_WARN("already in simulated time");
        return WMQE_INVALIDOP;
    }

    state->is_simulated_time = 1;
    state->sim_time_ns = 0;
    state->time_prev_ns = 0;
    state->sim_time_step_ns = 1000;//1 us
    state->steps_count = 2000;
    state->scheduler_calls = 0;
    state->scheduler_nanoseconds = 0;

    //TODO: swap betaflight realtime/simulated state?
    return 0;
}

//stop working in simulated time
int sitl2_stop_simulated_time(sitl2_state_t *state){
    if(!state->is_simulated_time){
        WMQ_LOG_WARN("already in simulated time");
        return WMQE_INVALIDOP;
    }
    state->is_simulated_time = 0;

    //reset start time
    state->start_hrtime = uv_hrtime();
    state->time_prev_ns = 0;
    state->scheduler_calls = 0;
    state->scheduler_nanoseconds = 0;

    //TODO: swap betaflight realtime/simulated state?
    return 0;
}