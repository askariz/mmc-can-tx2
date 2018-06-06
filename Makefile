#/******
#	MMC Gimbal Control Demo
# 	Author: ChrisRiz
#	Date:	2018-03-20
#/*****

DESTDIR ?=
PREFIX ?= /usr/local

MAKEFLAGS = -k

CFLAGS    = -O2 -Wall -Wno-parentheses

CPPFLAGS += -Iinclude \
	    -D_FILE_OFFSET_BITS=64 \
	    -DSO_RXQ_OVFL=40 \
	    -DPF_CAN=29 \
	    -DAF_CAN=PF_CAN \
	    -D_GNU_SOURCE

PROGRAMS = mmc_gimbal_ctrl

all: $(PROGRAMS)
	mkdir -p bin
	cp -f $(PROGRAMS) bin

clean:
	rm -f $(PROGRAMS) *.o
	rm -rf $(DESTDIR)bin
	

install:
	mkdir -p $(DESTDIR)$(PREFIX)/bin
	cp -f $(PROGRAMS) $(DESTDIR)$(PREFIX)/bin

distclean:
	rm -f $(PROGRAMS) $(LIBRARIES) *.o *~

mmc_gimbal_ctrl.o:	lib.h
mmc_gimbal_ctrl:	mmc_gimbal_ctrl.o	lib.o
