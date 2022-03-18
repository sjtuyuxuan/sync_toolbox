XC32PATH=/opt/microchip/xc32/v4.00/bin
HARMONYPATH=
NU32PATH=/home/yuxuan/winter2022/ME333/embedded_computing_code/01_Quickstart
PORT=/dev/ttyUSB0
TERMEMU=

#This file is used to:
#	1. compile .c files into .o files
#	2. link the .o files into a .elf binary
#	3. convert the .elf into a .hex
#	4. write the .hex file to the pic using the bootloader utility
# The following must be edited by you so that
# make knows where to find the compiler tools (xc32)
# and the harmony library.
#
# XC32PATH is the path to the xc32 bin directory. This directory contains
# tools such as the compiler (xc32-gcc) and hex converter 
# (xc32-bin2hex)
# Windows example (note the use of slashes not backslashes):
# XC32PATH=C:/Program Files (x86)/Microchip/xc32/v1.40/bin
# Mac example:
# XC32PATH =/Applications/microchip/xc32/v1.40/bin
#
# HARMONYPATH is the path to the harmony framework directory
# Windows example:
# HARMONYPATH=C:/Program Files (x86)/Microchip/harmony/v1_06
# Mac example:
# HARMONYPATH=/Users/kevin/harmony/v1_06
#
# NU32PATH is the path to the nu32utility.
# You compiled this and named it nu32utility
# Example:
# if your nu32utility is in
# /home/student/PIC32/nu32utilty, then
# NU32PATH=/home/student/PIC32
#
# PORT is the COM port used by the PIC32
# Windows Example:
# PORT=\\.\COM22
# Mac Example:
# PORT=/dev/tty.usbserial=00001024A
#
# TERMEMU is your terminal emulator
# Windows example:
# TERMEMU=C:/Program Files (x86)/PuTTY/putty.exe
# Mac example (screen is pre-installed and already on your execution path 
# so you can safely omit the full path)
# TERMEMU=screen
#END user configurable variables

#the c compiler
CC="$(XC32PATH)/xc32-gcc"

#the hexfile maker
HX="$(XC32PATH)/xc32-bin2hex"

#object dumper
OBJDMP="$(XC32PATH)/xc32-objdump"

#the bootloader utility
WRITE="$(NU32PATH)/nu32utility"

#the output target $(TARGET).hex
TARGET=out

#the linker script
LINKSCRIPT=NU32bootloaded.ld

#additional linker flags
LINKFLAGS=-Map=$(TARGET).map

#if we have specified a linker script add it
ifdef LINKSCRIPT
	LINKFLAGS:=--script=$(LINKSCRIPT),$(LINKFLAGS)
endif

OBJS := $(patsubst %.c, %.o,$(wildcard *.c))
HDRS := $(wildcard *.h)
PROC = 32MX795F512H
CFLAGS=-g -O1 -x c

#if on windows use a different RM
ifdef ComSpec
	RM = del /Q
endif

#what to do when make all
.PHONY : all
all : $(TARGET).hex $(TARGET).dis
# Turn the elf file into a hex file.
$(TARGET).hex : $(TARGET).elf
	@echo Creating hex file
	$(HX) $(TARGET).elf

# perform disassembly
$(TARGET).dis : $(TARGET).elf
	$(OBJDMP) -S $< > $@

# Link all the object files into an elf file.
$(TARGET).elf : $(OBJS)
	@echo Linking elf file
	$(CC) -mprocessor=$(PROC) -o $(TARGET).elf $(OBJS) -Wl,$(LINKFLAGS)

# Create an object file for each C file.
%.o : %.c $(HDRS)
	@echo Creating object file $@
	$(CC) $(CFLAGS) -c -mprocessor=$(PROC) -o $@ $<
#path to harmony framework, harmony peripherals, harmony dsp library, harmony libq
.PHONY: clean
# Erase all hex, map, object, and elf files.
clean :
	$(RM) *.hex *.map *.o *.elf *.dep *.dis       

.PHONY: write
# After making, call the NU32utility to program via bootloader.
write : $(TARGET).hex $(TARGET).dis
	$(WRITE) "$(PORT)" $(TARGET).hex 

.PHONY: putty
putty :
	"$(TERMEMU)" -serial -sercfg 230400,R "$(PORT)"

.PHONY: screen
screen :
	"$(TERMEMU)" "$(PORT)" 230400,crtscts
