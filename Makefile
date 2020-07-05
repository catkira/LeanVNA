# paths to libraries
MCULIB         ?= /media/sf__shared_git/mculib
OPENCM3_DIR    ?= /media/sf__shared_git/libopencm3-gd32f3

# device config
BOARDNAME		= board_v2_2
DEVICE          = gd32f303cc_nofpu


OBJS			+= main2.o $(BOARDNAME)/board.o vna_measurement.o common.o synthesizers.o gitversion.hpp
OBJS			+= globals.o flash.o command_parser.o stream_fifo.o
OBJS			+= sin_rom.o gain_cal.o
OBJS            += $(MCULIB)/message_log.o $(MCULIB)/printf.o $(MCULIB)/fastwiring.o $(MCULIB)/si5351.o $(MCULIB)/dma_adc.o $(MCULIB)/dma_driver.o $(MCULIB)/usbserial.o

CFLAGS          += -O2 -g
CPPFLAGS		+= -O2 -g -ffast-math -fstack-protector-strong -I$(BOARDNAME) -I$(MCULIB)/include -DMCULIB_DEVICE_STM32F103 -DSTM32F103 -DSTM32F1 -D_XOPEN_SOURCE=600
CPPFLAGS		+= -Wall -Wno-unused-function
#CPPFLAGS		+= -DDISPLAY_ST7796
CPPFLAGS		+=  -ffunction-sections -fdata-sections
#C++ only flags, CPP is used for both C++ and C files
CXXFLAGS		+= -std=c++17 -fno-exceptions -fno-rtti

# safe g++ flags
CPPFLAGS		+= -funsigned-char -fwrapv -fno-delete-null-pointer-checks -fno-strict-aliasing

LDFLAGS         += -static -nostartfiles -Wl,--exclude-libs,libssp -Wl,--print-memory-usage
LDFLAGS			+= -Wl,--gc-sections
LDLIBS          += -Wl,--start-group -lgcc -lnosys -Wl,--end-group -lm

GITVERSION		= "$(shell git log -n 1 --pretty=format:"git-%ad%h" --date=format:"%Y%m%d-")"

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

LDSCRIPT = ./gd32f303cc_with_bootloader.ld

.PHONY: clean all

all: binary.elf binary.hex binary.bin

gitversion.hpp: .git/HEAD .git/index
	echo "#define GITVERSION \"$(GITVERSION)\"" > $@

clean:
	$(Q)$(RM) -rf binary.* *.o

flash: binary.hex
	./st-flash --reset --format ihex write binary.hex


include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
