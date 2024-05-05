# set the application name
APP_NAME := mpu6050

# set the compiler
CC := gcc

# set the linked libraries
LIBS := -lm \
	-lpthread \
	-lgpiod

# set all header directories
INC_DIRS := -I ../../src/ \
	    -I ../../interface/ \
	    -I ../../example/ \
	    -I ../../test/ \
	    -I ./interface/inc/

# set the main source
MAIN := $(wildcard ../../src/*.c) \
	$(wildcard ../../example/*.c) \
	$(wildcard ../../test/*.c) \
	$(wildcard ./interface/src/*.c) \
	$(wildcard ./driver/src/*.c) \
	$(wildcard ./src/*.c)

# set flags of the compiler
CFLAGS := -O3 \
	  -DNDEBUG \
	  -D IIC_DEVICE_NAME='"/dev/i2c-2"' \
	  -D GPIO_DEVICE_NAME='"/dev/gpiochip1"' \
	  -D GPIO_DEVICE_LINE=0

# set all .PHONY
.PHONY: all

# set the output list
all: $(APP_NAME)

# set the main app
$(APP_NAME) : $(MAIN)
	$(CC) $(CFLAGS) $^ $(INC_DIRS) $(LIBS) -o $@

# set clean .PHONY
.PHONY: clean

# clean the project
clean :
	rm -rf $(APP_NAME)
