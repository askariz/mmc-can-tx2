#/******
#	MMC Gimbal Control Demo
# 	Author: ChrisRiz
#	Date:	2018-03-20
#/*****

DESTDIR ?=
PREFIX ?= /usr/local
GCC = gcc
MAKEFLAGS = -k

CFLAGS    = -O2 -Wall -Wno-parentheses

CPPFLAGS += -Iinclude \
	    -D_FILE_OFFSET_BITS=64 \
	    -DSO_RXQ_OVFL=40 \
	    -DPF_CAN=29 \
	    -DAF_CAN=PF_CAN \
	    -D_GNU_SOURCE

PROGRAMS = mmc_gimbal_ctrl

define all-c-files-under
$(shell find $(1) -name "*."$(2) -and -not -name ".*" )
endef

define all-subdir-c-files
$(call all-c-files-under,.,"c")
endef

CSRCS	 = $(call all-subdir-c-files)

COBJS	:= $(CSRCS:.c=.o)

all: mmc_gimbal_ctrl

mmc_gimbal_ctrl: $(COBJS)
	$(GCC) $(CFLAGS) $(COBJS) -o mmc_gimbal_ctrl $(LIBS)

$(COBJS) : %.o : %.c
	$(GCC) $(CFLAGS) -c $< -o $@

clean :
	rm mmc_gimbal_ctrl
	rm *.o