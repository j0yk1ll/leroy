# Dedicated diagnostic image: this Makefile has NO flash/erase target.
# Does not compile production main.c or actuator/IMU drivers.
CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size
CPU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CPPFLAGS := -DSTM32F4 -DSTM32F401xC -I../include -I/usr/arm-none-eabi/include
CFLAGS := $(CPU) -std=gnu11 -Os -g3 -ffunction-sections -fdata-sections \
  -Wall -Wextra -Wshadow -Wundef -Wdouble-promotion
LDLIBS := /usr/arm-none-eabi/lib/libopencm3_stm32f4.a -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group
LDFLAGS := $(CPU) -nostartfiles -T../stm32f401cc.ld -Wl,--gc-sections -Wl,-Map=build/i2c_gpio_diag.map --specs=nano.specs --specs=nosys.specs

.PHONY: build check-deps
.DEFAULT_GOAL := build

build: check-deps build/i2c_gpio_diag.bin
	@$(SIZE) build/i2c_gpio_diag.elf

check-deps:
	@command -v $(CC) >/dev/null
	@command -v $(OBJCOPY) >/dev/null
	@test -f /usr/arm-none-eabi/lib/libopencm3_stm32f4.a

build:
	@mkdir -p build

build/diag_main.o: i2c_gpio_main.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

build/usb_serial.o: ../src/usb_serial.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

build/i2c_gpio_diag.elf: build/diag_main.o build/usb_serial.o ../stm32f401cc.ld
	$(CC) $(LDFLAGS) build/diag_main.o build/usb_serial.o $(LDLIBS) -o $@

build/i2c_gpio_diag.bin: build/i2c_gpio_diag.elf
	$(OBJCOPY) -O binary $< $@
