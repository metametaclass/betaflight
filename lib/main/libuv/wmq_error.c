#include "wmq_error.h"

#include "string.h"

//const char* wmq_strerror(int err) {
//  return strerror(err);
//}

/*
#define WMQ_ERR_NAME_GEN(name, _) case WMQ_ ## name: return #name;
const char* wmq_err_name(int err) {
  switch (err) {
    WMQ_ERRNO_MAP(WMQ_ERR_NAME_GEN)
    default:
      assert(0);
      return NULL;
  }
}
#undef WMQ_ERR_NAME_GEN
*/

#define WMQ_STRERROR_GEN_UV(name, msg) \
    case UV_##name:                    \
        return #name " " msg;
#define WMQ_STRERROR_GEN_WMQ(name, msg) \
    case WMQE_##name:                   \
        return #name " " msg;
const char* wmq_strerror(int err) {
    switch (err) {
        UV_ERRNO_MAP(WMQ_STRERROR_GEN_UV)
        WMQ_ERRNO_MAP(WMQ_STRERROR_GEN_WMQ)
        default:
            return "Unknown system error";
    }
}
#undef WMQ_STRERROR_GEN_UV
#undef WMQ_STRERROR_GEN_WMQ

#define WMQ_STRERROR_GEN_UV(name, msg) \
    case UV_##name:                    \
        return #name;
#define WMQ_STRERROR_GEN_WMQ(name, msg) \
    case WMQE_##name:                   \
        return #name;
const char* wmq_strerror_short(int err) {
    switch (err) {
        UV_ERRNO_MAP(WMQ_STRERROR_GEN_UV)
        WMQ_ERRNO_MAP(WMQ_STRERROR_GEN_WMQ)
        default:
            return "Unknown system error";
    }
}
#undef WMQ_STRERROR_GEN_UV
#undef WMQ_STRERROR_GEN_WMQ

int wmq_errno_to_wmqerror(int _errno) {
    switch (_errno) {
        case ENOENT:
            return UV_ENOENT;
        case EACCES:
            return UV_EACCES;
        default:
            WMQ_LOG(LL_WARN, "unknown code %d %s", _errno, strerror(_errno));
            return UV_UNKNOWN;
    }
}