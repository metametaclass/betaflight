
#include <uv.h>

#if UV_VERSION_MINOR < 19
uv_handle_type uv_handle_get_type(const uv_handle_t* handle) {
    return handle->type;
}

const char* uv_handle_type_name(uv_handle_type type) {
    switch (type) {
#define XX(uc, lc) \
    case UV_##uc:  \
        return #lc;
        UV_HANDLE_TYPE_MAP(XX)
#undef XX
        case UV_FILE:
            return "file";
        case UV_HANDLE_TYPE_MAX:
        case UV_UNKNOWN_HANDLE:
            return NULL;
    }
    return NULL;
}
#endif


#if UV_VERSION_MINOR < 34

#ifdef _WIN32
void uv_sleep(int msec) {
  Sleep(msec);
}
#else

#include <unistd.h>

void uv_sleep(int msec) {
  usleep(msec * 1000);
}
#endif

#endif
