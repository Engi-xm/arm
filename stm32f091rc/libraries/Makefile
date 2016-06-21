##########------------------------------------------------------##########
##########                  Project specifics                   ##########
##########------------------------------------------------------##########
## Set library path
LIBDIR = ../../libraries
## Set controller path
## NOTE: might not work if there's more than 1 folder in /media/
MCUDIR = /media/*/NUCLEO
## Set Cortex version
CORTEX = m0

##########------------------------------------------------------##########
##########                  Program Locations                   ##########
##########------------------------------------------------------##########
CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

##########------------------------------------------------------##########
##########                   Makefile Magic!                    ##########
##########         Summary:                                     ##########
##########        Compiles $(TARGET).bin file from sources      ##########
##########        found in $(LIBDIR) and $(CURDIR);             ##########
##########        Copies $(TARGET).bin to mounted NUCLEO        ##########
##########------------------------------------------------------##########
## Set microcontroller variables
## Full name (info only)
MCU   := $(shell echo $(notdir $(wildcard $(LIBDIR)/*.ld))       | cut -c 1-11)
## Family name (info only)
MCU_F := $(shell echo $(notdir $(wildcard $(LIBDIR)/system_*.c)) | cut -c 8-16)
## Name as found in main family header
MCU_D := $(shell echo $(notdir $(wildcard $(LIBDIR)/*.s))        | cut -c 9-19)
## Modify MCU_D to usable form
MCU_D_UC  := $(shell echo $(MCU_D) | tr '[:lower:]' '[:upper:]')
MCU_D_MOD := $(subst X,x,$(MCU_D_UC))

## Set TARGET to enclosing directory name
TARGET = $(lastword $(subst /, ,$(CURDIR)))

## Set SOURCES to any source file in CURDIR and helper files from LIBDIR
SOURCES  = $(wildcard *.c)
SOURCES += $(wildcard $(LIBDIR)/*.c)
SOURCES += $(wildcard $(LIBDIR)/*.s)

## Set OBJS objects from SOURCES
OBJS  = $(filter %.o, $(SOURCES:.c=.o))
OBJS += $(filter %.o, $(SOURCES:.s=.o))

## Set compiler flags
CFLAGS    = -Wall -mlittle-endian -mthumb -Os -c
CFLAGS   += -mcpu=cortex-$(CORTEX)
CPPFLAGS  = -D$(MCU_D_MOD)
CPPFLAGS += -I$(LIBDIR)

## Set LINKER variable to linker script in LIBDIR
LINKER = $(wildcard $(LIBDIR)/*.ld)

## Set linker flags
LDFLAGS  = -Wl,--gc-sections -mlittle-endian -mthumb
LDFLAGS += -D$(MCU_D_MOD)
LDFLAGS += -mcpu=cortex-$(CORTEX)
LDFLAGS += -T$(LINKER)

## Set rules
## Default target
all: $(TARGET).bin
	@ls
## Compile .elf to file binary file
$(TARGET).bin: $(TARGET).elf
	@$(OBJCOPY) -O binary $< $@
## Link objects to .elf file
$(TARGET).elf: $(OBJS)
	@$(LD) $(LDFLAGS) $(OBJS) -o $@
## Compile sources (C) to objects
%.o: %.c
	@$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@
## Compile sources (assembly) to objects
%.o: %.s
	@$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@
## Set phony targets
.PHONY: all flash debug clean squeaky_clean

## Convenience rules
## Copy binary to microcontroller
flash: $(TARGET).bin
	@cp $< $(MCUDIR)/
	@ls
## Clean CURDIR
clean:
	@rm -f $(TARGET).o $(TARGET).elf $(TARGET).bin
	@ls
## Clean everything
squeaky_clean:
	@rm -f $(OBJS) $(TARGET).elf $(TARGET).bin
	@ls
## Debug information
debug:
	@echo "Cortex:     " $(CORTEX)
	@echo "MCU family: " $(MCU_F)
	@echo "MCU:        " $(MCU)
	@echo "MCU define: " $(MCU_D_MOD)
	@echo
	@echo "Sources:       " $(SOURCES)
	@echo "Linker script: " $(LINKER)
