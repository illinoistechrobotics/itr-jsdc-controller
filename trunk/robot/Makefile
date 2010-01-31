# Setup the environment
TOOLCHAIN_DIR ?= ../toolchain/
PATH := $(PATH):$(TOOLCHAIN_DIR)
ROBOSTIX_DIR ?= ../robostix/
I2CIO_DIR ?= $(ROBOSTIX_DIR)/gumstix/i2c-io/
COMMON = ../common

# Turn on all warnings
OPTS = -Wall -g -march=armv5te -mtune=xscale
CC = arm-linux-gcc
LIBS = -lpthread

BINARY = robot
LOCAL_OBJ  = robot.o \
			 mod_i2c-io.o \
			 robot_events.o \
			 adc.o 
COMMON_OBJ = robot_comm.o \
			 robot_log.o \
			 robot_queue.o \
			 timer.o 
I2CIO_OBJ  = AvrInfo.o \
			 BootLoader-api.o \
			 Crc8.o \
			 DumpMem.o \
			 Log.o \
			 i2c-api.o \
			 i2c-io-api.o

# look for .h files in these directories
INCLUDES = -I. \
		   -I$(COMMON) \
		   -I$(ROBOSTIX_DIR)/gumstix/Shared \
		   -I$(ROBOSTIX_DIR)/gumstix/Common \
		   -I$(ROBOSTIX_DIR)/Shared \
		   -I$(ROBOSTIX_DIR)/Common \
		   -I$(ROBOSTIX_DIR)/gumstix/i2c-io
	   

# Begin derived variablse
CFLAGS += $(INCLUDES) $(OPTS)
LOCAL_SRC = $(LOCAL_OBJ:%.o=%.c)
COMMON_SRC = $(COMMON_OBJ: %.o=$(COMMON)/%.c)

OBJ = $(LOCAL_OBJ) $(COMMON_OBJ) $(I2CIO_OBJ)
SRC = $(LOCAL_SRC) $(COMMON_SRC)

.PHONY: all
all: $(BINARY)

$(BINARY): $(OBJ)
	$(CC) $(CFLAGS) $(LIBS) $(OBJ) -o $(BINARY)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(COMMON)/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(ROBOSTIX_DIR)/gumstix/Shared/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(ROBOSTIX_DIR)/gumstix/Common/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(ROBOSTIX_DIR)/Shared/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(ROBOSTIX_DIR)/Common/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: $(ROBOSTIX_DIR)/gumstix/i2c-io/%.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	-rm -f $(OBJ) $(BINARY)

