#include "uv.h"

#include "wmq_error.h"
#include "wmq_debug.h"
#include "libuv_compat.h"
#include <stdlib.h>


//buffer allocation callback
void on_alloc_buffer(uv_handle_t* handle, size_t size, uv_buf_t* buf) {
    (void)(handle);
    buf->base = malloc(size);
    buf->len = size;
}


//handle visitor callback to close all handles
void on_walk_close_handle(uv_handle_t* handle, void* arg) {
    (void)(arg);
    WMQ_LOG(LL_INFO, "closing %s", uv_handle_type_name(uv_handle_get_type(handle)));
    uv_close(handle, NULL);
}

//close all handles and end the loop
void close_all_handles(uv_loop_t* loop) {
    uv_walk(loop, on_walk_close_handle, NULL);
}