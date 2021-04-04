#pragma once

//libuv version compatibility layer

#if UV_VERSION_MINOR < 19

uv_handle_type uv_handle_get_type(const uv_handle_t* handle);

const char* uv_handle_type_name(uv_handle_type type);

#endif
