# Sensor-only diagnostic image. Default goal builds only; no flash target.
CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size
CPU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CPPFLAGS := -DSTM32F4 -DSTM32F401xC -I../include -I/usr/arm-none-eabi/include
CFLAGS := $(CPU) -std=gnu11 -Os -g3 -ffunction-sections -fdata-sections \
  -Wall -Wextra -Wshadow -Wundef -Wdouble-promotion
LDLIBS := /usr/arm-none-eabi/lib/libopencm3_stm32f4.a -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group
LDFLAGS := $(CPU) -nostartfiles -T../stm32f401cc.ld -Wl,--gc-sections -Wl,-Map=build/i2c_ack_diag.map --specs=nano.specs --specs=nosys.specs

.PHONY: all check-deps
.DEFAULT_GOAL := all
all: check-deps build/i2c_ack_diag.bin
	@$(SIZE) build/i2c_ack_diag.elf

check-deps:
	@command -v $(CC) >/dev/null
	@command -v $(OBJCOPY) >/dev/null
	@test -f /usr/arm-none-eabi/lib/libopencm3_stm32f4.a

build/.dir:
	@mkdir -p build
	@touch $@

build/ack_main.o: i2c_ack_main.c | build/.dir
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

build/usb_serial.o: ../src/usb_serial.c | build/.dir
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

build/i2c_ack_diag.elf: build/ack_main.o build/usb_serial.o ../stm32f401cc.ld
	$(CC) $(LDFLAGS) build/ack_main.o build/usb_serial.o $(LDLIBS) -o $@

build/i2c_ack_diag.bin: build/i2c_ack_diag.elf
	$(OBJCOPY) -O binary $< $@
