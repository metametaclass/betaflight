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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "io/serial.h"
#include "serial_libuv.h"
#include "uv.h"
#include "wmq_error.h"

#define BASE_PORT 5760

static uv_loop_t *libuv_loop;

void init_serial_libuv_loop(uv_loop_t *loop){
    libuv_loop = loop;
}


static const struct serialPortVTable libuvSerialVTable; // Forward declaration
static libuvSerialPort_t libuvSerialPorts[SERIAL_PORT_COUNT];


//buffer allocation callback
static void on_alloc_buffer(uv_handle_t* handle, size_t size, uv_buf_t* buf) {
    UNUSED(handle);
    buf->base = malloc(size);
    buf->len = size;
}

//TCP client connection close callback
static void on_close_client_handle(uv_handle_t* handle) {
    WMQ_LOG(LL_INFO, "");
    uv_tcp_t *client = (uv_tcp_t *)handle;
    libuvSerialPort_t *port = container_of(client, libuvSerialPort_t, client);
    port->connected = false;
}

size_t libuvSerialRxBufferFreeSize(serialPort_t *port){
    uint32_t bytesUsed;

    if (port->txBufferHead >= port->txBufferTail) {
        bytesUsed = port->txBufferHead - port->txBufferTail;
    } else {
        bytesUsed = port->txBufferSize + port->txBufferHead - port->txBufferTail;
    }
    uint32_t bytesFree = (port->txBufferSize - 1) - bytesUsed;

    return bytesFree;
}

//TCP client connection read callback
static void on_incoming_tcp_read(uv_stream_t* client_handle, ssize_t nread, const uv_buf_t* buf) {
    uv_tcp_t *client = (uv_tcp_t *)client_handle;
    libuvSerialPort_t *port = container_of(client, libuvSerialPort_t, client);

    if (nread <= 0) {
        WMQ_LOG(LL_INFO, "close client_handle");
        uv_close((uv_handle_t*)client_handle, on_close_client_handle);
    } else {
        debug_print_hex(LL_DEBUG, "", buf->base, nread, 0);
        if(port->port.rxCallback){
            //feed read callback
            for(ssize_t idx = 0; idx<nread;idx++){
                port->port.rxCallback(buf->base[idx], port->port.rxCallbackData);
            }
        } else {
            //find free bytes in receive buffer
            size_t freeBuffer = libuvSerialRxBufferFreeSize(&port->port);
            if ((size_t)nread > freeBuffer) {
                WMQ_LOG(LL_WARN, "RX buffer overflow: %z > %zu", nread, freeBuffer);
            }

            //copy nread bytes from buf->base to receive buffer
            for(ssize_t idx = 0; idx<nread;idx++) {
                port->port.rxBuffer[port->port.rxBufferHead] = buf->base[idx];
                if (port->port.rxBufferHead + 1 >= port->port.rxBufferSize) {
                    port->port.rxBufferHead = 0;
                } else {
                    port->port.rxBufferHead++;
                }
            }

            //force call scheduler
            scheduler();
        }
    }
}


//tcp server client connection callback
void on_accept_cb(uv_stream_t* server_handle, int status) {
    uv_loop_t* loop = server_handle->loop;
    //uv_tcp_t* client;
    //uv_tcp_t temp_client;
    libuvSerialPort_t *port = container_of((uv_tcp_t *)server_handle, libuvSerialPort_t, server);
    int rc;

    if (status != 0) {
        WMQ_LOG(LL_ERROR, "error %d \"%s\"", status, uv_strerror(status));
        return;
    } else {
        WMQ_LOG(LL_TRACE, "");
    }

    if (port->connected){
        WMQ_LOG(LL_ERROR, "port already has connected client");

        //TODO: close client connection?
        //rc = uv_tcp_init(libuv_loop, &temp_client);
        //WMQ_CHECK_ERROR_AND_RETURN_VOID(rc, "on_accept_cb: uv_tcp_init");
        //uv_accept(server_handle, &tmpfile)
        return;
    }
    
    //initialize libuv handle
    rc = uv_tcp_init(loop, &port->client);
    WMQ_CHECK_ERROR_AND_RETURN_VOID(rc, "on_accept_cb: uv_tcp_init");

    //accept connection
    rc = uv_accept(server_handle, (uv_stream_t*)&port->client);
    WMQ_CHECK_ERROR_AND_RETURN_VOID(rc, "uv_accept");
    port->connected = true;

    //start reading data
    uv_read_start((uv_stream_t*)&port->client, on_alloc_buffer, on_incoming_tcp_read);
}


static libuvSerialPort_t* libuvSerialInit(libuvSerialPort_t *s, int id)
{
    struct sockaddr addr = {0};
    int rc;

    WMQ_LOG(LL_INFO, "%d", id);

    if (s->initialized) {
        WMQ_LOG(LL_WARN, "port %d is already initialized", id);
        return s;
    }
    

    s->connected = false;
    s->clientCount = 0;
    s->id = id;

    rc = uv_tcp_init(libuv_loop, &s->server);
    WMQ_CHECK_ERROR(rc, "uv_tcp_init");
    if(rc){
        return NULL;
    }

    addr.sa_family = AF_INET;
    rc = uv_ip4_addr("0.0.0.0", BASE_PORT+id+1, (struct sockaddr_in*)&addr);
    WMQ_CHECK_ERROR(rc, "uv_ip4_addr");
    if(rc){
        return NULL;
    }

    //dns_request.rc = uv_getaddrinfo(&loop, &dns_request, get_addrinfo_cb, resolver->host, port, &hints);

    rc = uv_tcp_bind(&s->server, &addr, 0);
    WMQ_CHECK_ERROR(rc, "uv_tcp_bind");
    if(rc){
        return NULL;
    }

    rc = uv_listen((uv_stream_t*)&s->server, SOMAXCONN, on_accept_cb);
    WMQ_CHECK_ERROR(rc, "uv_listen");
    if(rc){
        return NULL;
    }

    //unreference handle, close loop after tty closing
    uv_unref((uv_handle_t*)&s->server);

    s->initialized = true; 
    
    return s;
}

serialPort_t *libuvSerialOpen(int id, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    libuvSerialPort_t *s = NULL;

#if defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6) || defined(USE_UART7) || defined(USE_UART8)
    if (id >= 0 && id < SERIAL_PORT_COUNT) {
        s = libuvSerialInit(&libuvSerialPorts[id], id);
    }
#endif
    if (!s)
        return NULL;

    s->port.vTable = &libuvSerialVTable;

    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    s->port.rxBufferSize = RX_BUFFER_SIZE;
    s->port.txBufferSize = TX_BUFFER_SIZE;
    s->port.rxBuffer = s->rxBuffer;
    s->port.txBuffer = s->txBuffer;

    // callback works for IRQ-based RX ONLY
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    return (serialPort_t *)s;
}

uint32_t libuvSerialTotalRxBytesWaiting(const serialPort_t *instance)
{
    libuvSerialPort_t *s = (libuvSerialPort_t*)instance;
    uint32_t count;

    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        count = s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        count = s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }

    return count;
}

uint32_t libuvSerialTotalTxBytesFree(const serialPort_t *instance)
{
    libuvSerialPort_t *s = (libuvSerialPort_t*)instance;
    return (uint32_t)libuvSerialRxBufferFreeSize(&s->port);
}

bool isLibuvSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    libuvSerialPort_t *s = (libuvSerialPort_t *)instance;
    
    bool isEmpty = s->port.txBufferTail == s->port.txBufferHead;
    
    return isEmpty;
}

uint8_t libuvSerialRead(serialPort_t *instance)
{
    uint8_t ch;
    libuvSerialPort_t *s = (libuvSerialPort_t *)instance;

    if(s->port.rxBufferTail == s->port.rxBufferHead){
        WMQ_LOG(LL_DEBUG, "rx buffer is empty");
        return 0;
    }

    ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }

    return ch;
}

typedef struct write_req_s {
  uv_write_t req;
  uv_buf_t buf;
  size_t original_size;
} write_req_t;

void after_write_cb(uv_write_t* req, int status)
{
    UNUSED(status);
    write_req_t* wr = (write_req_t*) req;
    free(wr->buf.base);
    free(wr);
}


int libuvWrite(uv_tcp_t *client, const void *data, size_t size){
    write_req_t *req = malloc(sizeof(write_req_t));
    if(!req){
        WMQ_LOG_NOMEMORY("write_some: malloc 1 error");
        return UV_ENOMEM;
    }
    char *buffer = (char*)malloc(size);
    if(!buffer) {
        WMQ_LOG_NOMEMORY("write_some: malloc 2 error");
        free(req);
        return UV_ENOMEM;
    }
    req->buf = uv_buf_init(buffer, size);
    req->original_size = size;
    memcpy(req->buf.base, data, size);

    int rc = uv_write(&req->req, (uv_stream_t *) client, &req->buf, 1, after_write_cb); 
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "uv_write");

    return 0;
}

void libuvSerialWrite(serialPort_t *instance, uint8_t ch)
{
    libuvSerialPort_t *s = (libuvSerialPort_t *)instance;
    int rc;

    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }

    if(!s->connected){
        WMQ_LOG(LL_ERROR, "client disconnected");
        return;
    }
    if (s->port.txBufferHead < s->port.txBufferTail) {
        // send data till end of buffer
        int chunk = s->port.txBufferSize - s->port.txBufferTail;
        rc = libuvWrite(&s->client, (const void *)&s->port.txBuffer[s->port.txBufferTail], chunk);
        if(rc){
            //force_stop(libuv_loop);
            return;
        }
        s->port.txBufferTail = 0;
    }
    int chunk = s->port.txBufferHead - s->port.txBufferTail;
    if (chunk) {
        rc = libuvWrite(&s->client, (const void *)&s->port.txBuffer[s->port.txBufferTail], chunk);
        WMQ_CHECK_ERROR_AND_RETURN_VOID(rc, "libuvWrite");
    }

    s->port.txBufferTail = s->port.txBufferHead;

}


static const struct serialPortVTable libuvSerialVTable = {
        .serialWrite = libuvSerialWrite,
        .serialTotalRxWaiting = libuvSerialTotalRxBytesWaiting,
        .serialTotalTxFree = libuvSerialTotalTxBytesFree,
        .serialRead = libuvSerialRead,
        .serialSetBaudRate = NULL,
        .isSerialTransmitBufferEmpty = isLibuvSerialTransmitBufferEmpty,
        .setMode = NULL,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
};
