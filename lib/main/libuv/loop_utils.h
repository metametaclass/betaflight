//libuv event loop related functions
#pragma once

#include "uv.h"
#include "stdint.h"

//buffer allocation callback
void on_alloc_buffer(uv_handle_t* handle, size_t size, uv_buf_t* buf);

//close all handles and end the loop
void close_all_handles(uv_loop_t* loop);