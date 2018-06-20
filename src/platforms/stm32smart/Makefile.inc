CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
SWO_TRACE_MODE ?= 2
FORCE_FLASH_128K ?= 0

CFLAGS += -Istm32/include -mcpu=cortex-m3 -mthumb \
	-DSTM32F1 -DBLACKMAGIC -DSWO_TRACE_MODE=$(SWO_TRACE_MODE) -I../libopencm3/include \
	-Iplatforms/stm32

LDFLAGS_BOOT := $(LDFLAGS) -lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,platforms/stm32/blackmagic.ld -nostartfiles -lc \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m3 -Wl,-gc-sections \
	-L../libopencm3/lib
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8002000

ifeq ($(ENABLE_DEBUG), 1)
LDFLAGS += --specs=rdimon.specs
else
LDFLAGS += --specs=nosys.specs
endif

VPATH += platforms/stm32

SRC += 	cdcacm.c	\
	usbuart.c	\
	serialno.c	\
	timing.c	\
	timing_stm32.c

ifeq ($(SWO_TRACE_MODE), 1)
	SRC += traceswo_stm32smart.c
else
	SRC += traceswoasync.c
endif

DFU_OBJ = usbdfu.o dfu_f1.o

ifeq ($(FORCE_FLASH_128K), 1)
	DFU_OBJ += dfucore_stm32smart.o
else
	DFU_OBJ += dfucore.o
endif

all:	blackmagic.bin blackmagic_dfu.bin blackmagic_dfu.hex

blackmagic_dfu: $(DFU_OBJ)
	@echo "  LD      $@"
	$(Q)$(CC) $^ -o $@ $(LDFLAGS_BOOT)

host_clean:
	-$(Q)$(RM) -f blackmagic.bin blackmagic_dfu blackmagic_dfu.bin blackmagic_dfu.hex

