#pragma once

#include "debug.h"
#include "uv.h"

#define CANARY_VALUE 0xA55AA55A

const char* wmq_strerror(int err);

const char* wmq_strerror_short(int err);

int wmq_errno_to_wmqerror(int _errno);

#define WMQ_CHECK_ERROR(result, message)                                                                                           \
    do {                                                                                                                           \
        if (result) {                                                                                                              \
            debug_print(LL_ERROR, "%s " __FILE__ ",%d " message " error %d %s", __func__, __LINE__, result, wmq_strerror(result)); \
        }                                                                                                                          \
    } while (0)

#define WMQ_CHECK_ERROR_AND_RETURN_RESULT(result, message)                                                                         \
    do {                                                                                                                           \
        if (result) {                                                                                                              \
            debug_print(LL_ERROR, "%s " __FILE__ ",%d " message " error %d %s", __func__, __LINE__, result, wmq_strerror(result)); \
            return result;                                                                                                         \
        }                                                                                                                          \
    } while (0)

#define WMQ_CHECK_ERROR_AND_GOTO_ERROR(result, message)                                                                            \
    do {                                                                                                                           \
        if (result) {                                                                                                              \
            debug_print(LL_ERROR, "%s " __FILE__ ",%d " message " error %d %s", __func__, __LINE__, result, wmq_strerror(result)); \
            goto error;                                                                                                            \
        }                                                                                                                          \
    } while (0)

#define WMQ_CHECK_ERROR_AND_RETURN_RESULT_AND_FREE_BUFFER(result, message, buffer, free_fn) \
    do {                                                                                    \
        if (result) {                                                                       \
            debug_print(LL_ERROR, __FUNCTION__                                              \
                        " "__FILE__                                                         \
                        ",%d " message " error %d %s",                                      \
                        __LINE__, result, wmq_strerror(result));                            \
            free_fn(buffer);                                                                \
            return result;                                                                  \
        }                                                                                   \
    } while (0)

#define WMQ_CHECK_ERROR_AND_RETURN_VOID(result, message)                                                                           \
    do {                                                                                                                           \
        if (result) {                                                                                                              \
            debug_print(LL_ERROR, "%s " __FILE__ ",%d " message " error %d %s", __func__, __LINE__, result, wmq_strerror(result)); \
            return;                                                                                                                \
        }                                                                                                                          \
    } while (0)

#define WMQ_CHECK_ERROR_SET_STATUS_RETURN_VOID(result, message, __status)                                                          \
    do {                                                                                                                           \
        if (result) {                                                                                                              \
            debug_print(LL_ERROR, "%s " __FILE__ ",%d " message " error %d %s", __func__, __LINE__, result, wmq_strerror(result)); \
            (__status)->status = WMQ_LINK_STATUS_ERROR;                                                                            \
            return;                                                                                                                \
        }                                                                                                                          \
    } while (0)

//UV errors - negative values
#define WMQE__MAX (UV_ERRNO_MAX - 1)

#define WMQE__TOO_LARGE (WMQE__MAX - 1)
#define WMQE__STOMP_PARSER_ERROR (WMQE__MAX - 2)
#define WMQE__INVALIDOP (WMQE__MAX - 3)
#define WMQE__TEST_FAILED (WMQE__MAX - 4)
#define WMQE__UNKNOWN_STOMP_COMMAND (WMQE__MAX - 5)
#define WMQE__INVALID_STOMP_STATE (WMQE__MAX - 6)
#define WMQE__INVALID_SWITCHBOARD_STATE (WMQE__MAX - 7)
#define WMQE__DUPLICATE_MESSAGE_ID (WMQE__MAX - 8)
#define WMQE__MESSAGE_NOT_FOUND (WMQE__MAX - 9)
#define WMQE__DUPLICATE_OBJECT_NAME (WMQE__MAX - 10)
#define WMQE__OBJECT_NOT_FOUND (WMQE__MAX - 11)
#define WMQE__NO_RECEIVERS (WMQE__MAX - 12)
#define WMQE__MESSAGE_NOT_PROCESSED (WMQE__MAX - 13)
#define WMQE__MESSAGE_PARTIALLY_PROCESSED (WMQE__MAX - 14)
#define WMQE__INVALID_CONFIG (WMQE__MAX - 15)
#define WMQE__ADDRESS_PARSE_ERROR (WMQE__MAX - 16)
#define WMQE__SYSCOMMAND_PARSE_ERROR (WMQE__MAX - 17)
#define WMQE__RETURN_MESSAGE (WMQE__MAX - 18)
#define WMQE__SYSTEM_ERROR (WMQE__MAX - 19)
#define WMQE__AUTH_FAILED (WMQE__MAX - 20)
#define WMQE__INVALID_PROTOCOL_VERSION (WMQE__MAX - 21)
#define WMQE__INVALID_CLI_COMMAND (WMQE__MAX - 22)
#define WMQE__TOO_MANY_MESSAGES (WMQE__MAX - 23)
#define WMQE__MESSAGE_PROCESS_EXCEPTION (WMQE__MAX - 24)
#define WMQE__UNKNOWN_ENUM_VALUE (WMQE__MAX - 25)
#define WMQE__PARSE_ERROR (WMQE__MAX - 26)
#define WMQE__STOMP_PROTOCOL_ERROR (WMQE__MAX - 27)
#define WMQE__EXTERNAL_ERROR (WMQE__MAX - 28)
#define WMQE__MESSAGE_PROCESS_ERROR (WMQE__MAX - 29)
#define WMQE__PROXY_RESPONSE_ERROR (WMQE__MAX - 30)
#define WMQE__CERTIFICATE_INVALID (WMQE__MAX - 31)

#define WMQ_ERRNO_MAP(XX)                                              \
    XX(TOO_LARGE, "data too large")                                    \
    XX(STOMP_PARSER_ERROR, "STOMP parser error")                       \
    XX(INVALIDOP, "invalid operation")                                 \
    XX(TEST_FAILED, "test condition failed")                           \
    XX(UNKNOWN_STOMP_COMMAND, "unknown STOMP command")                 \
    XX(INVALID_STOMP_STATE, "invalid STOMP parser state")              \
    XX(INVALID_SWITCHBOARD_STATE, "invalid switchboard state")         \
    XX(DUPLICATE_MESSAGE_ID, "duplicate message id")                   \
    XX(MESSAGE_NOT_FOUND, "message not found")                         \
    XX(DUPLICATE_OBJECT_NAME, "duplicate switch object name")          \
    XX(OBJECT_NOT_FOUND, "object not found")                           \
    XX(NO_RECEIVERS, "no message receivers found")                     \
    XX(MESSAGE_NOT_PROCESSED, "message not processed")                 \
    XX(MESSAGE_PARTIALLY_PROCESSED, "message was processed partially") \
    XX(INVALID_CONFIG, "invalid node config")                          \
    XX(ADDRESS_PARSE_ERROR, "address parse error")                     \
    XX(SYSCOMMAND_PARSE_ERROR, "address parse error")                  \
    XX(RETURN_MESSAGE, "return message to new")                        \
    XX(SYSTEM_ERROR, "system API error")                               \
    XX(AUTH_FAILED, "authentication failed")                           \
    XX(INVALID_PROTOCOL_VERSION, "invalid protocol version")           \
    XX(INVALID_CLI_COMMAND, "invalid command")                         \
    XX(TOO_MANY_MESSAGES, "too many messages")                         \
    XX(MESSAGE_PROCESS_EXCEPTION, "message process exception")         \
    XX(UNKNOWN_ENUM_VALUE, "unknown enum value")                       \
    XX(PARSE_ERROR, "string parser error")                             \
    XX(STOMP_PROTOCOL_ERROR, "stomp protocol error")                   \
    XX(EXTERNAL_ERROR, "third party library call error")               \
    XX(MESSAGE_PROCESS_ERROR, "message process error")                 \
    XX(PROXY_RESPONSE_ERROR, "proxy response error")                   \
    XX(CERTIFICATE_INVALID, "certificate validation error")

typedef enum {
#define XX(code, _) WMQE_##code = WMQE__##code,
    WMQ_ERRNO_MAP(XX)
#undef XX
        WMQ_ERRNO_MAX = WMQE__MAX - 1024  //1024 codes
} wmq_errno_t;
