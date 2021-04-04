INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(ROOT)/lib/main/libuv

MCU_COMMON_SRC  := \
            $(ROOT)/lib/main/libuv/debug.c \
            $(ROOT)/lib/main/libuv/libuv_compat.c \
            $(ROOT)/lib/main/libuv/wmq_error.c


#Flags
ARCH_FLAGS      =
DEVICE_FLAGS    =
LD_SCRIPT       = src/main/target/SITL2_LIBUV/pg.ld
STARTUP_SRC     =

TARGET_FLAGS    = -D$(TARGET)
MCU_FLASH_SIZE  := 2048

ARM_SDK_PREFIX  =

MCU_EXCLUDES = \
            drivers/adc.c \
            drivers/bus_i2c.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/dma.c \
            drivers/pwm_output.c \
            drivers/timer.c \
            drivers/system.c \
            drivers/rcc.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_pinconfig.c \
            drivers/rx/rx_xn297.c \
            drivers/display_ug2864hsweg01.c \
            telemetry/crsf.c \
            telemetry/ghst.c \
            telemetry/srxl.c \
            io/displayport_oled.c

TARGET_MAP  = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

LIBS := -lm -luv

ifeq ($(MINGW),1)
LIBS     += \
              -lws2_32 \
              -lwsock32
else
LIBS     += \
              -lc \
              -lrt
endif


LD_FLAGS    := \
              -lm \
              -lpthread \
              $(LIBS) \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -T$(LD_SCRIPT)

ifneq ($(filter SITL_STATIC,$(OPTIONS)),)
LD_FLAGS     += \
              -static \
              -static-libgcc
endif

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Ofast
OPTIMISE_SPEED      := -Ofast
OPTIMISE_SIZE       := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
endif
