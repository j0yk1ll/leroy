PROJECT := robot_firmware
BUILD_DIR := build
.DEFAULT_GOAL := all

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

SOURCES := $(wildcard src/*.c)
OBJECTS := $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(SOURCES))
DEPS    := $(OBJECTS:.o=.d)
BUILD_STAMP := $(BUILD_DIR)/.dir

CPUFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CPPFLAGS := -DSTM32F4 -DSTM32F401xC -Iinclude -I/usr/arm-none-eabi/include
CFLAGS   := $(CPUFLAGS) -std=gnu11 -Os -g3 \
            -ffunction-sections -fdata-sections \
            -Wall -Wextra -Wshadow -Wundef -Wdouble-promotion \
            -MMD -MP

LDFLAGS := $(CPUFLAGS) -nostartfiles \
           -Tstm32f401cc.ld \
           -Wl,--gc-sections \
           -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map \
           --specs=nano.specs --specs=nosys.specs

# IMPORTANT: do not add -L/usr/arm-none-eabi/lib here.  Arch's newlib is
# multilib-aware; forcing the generic directory ahead of GCC's selected
# Cortex-M4 hard-float directory causes an ABI mismatch at link time.
# libopencm3 is installed only in the generic prefix, so reference that one
# archive explicitly while allowing GCC to select libc/libgcc/libnosys.
OPENCM3_LIB := /usr/arm-none-eabi/lib/libopencm3_stm32f4.a
LDLIBS := $(OPENCM3_LIB) -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

ELF := $(BUILD_DIR)/$(PROJECT).elf
BIN := $(BUILD_DIR)/$(PROJECT).bin
HEX := $(BUILD_DIR)/$(PROJECT).hex

.PHONY: all build clean flash erase reset probe size check-deps toolchain-info

all:
	$(MAKE) clean
	$(MAKE) build
	$(MAKE) flash

build: check-deps $(ELF) $(BIN) $(HEX) size

check-deps:
	@command -v $(CC) >/dev/null || { echo "missing: $(CC)"; exit 1; }
	@command -v $(OBJCOPY) >/dev/null || { echo "missing: $(OBJCOPY)"; exit 1; }
	@test -f /usr/arm-none-eabi/lib/libopencm3_stm32f4.a || { echo "missing: libopencm3 (install Arch package libopencm3)"; exit 1; }

$(BUILD_STAMP):
	mkdir -p $(BUILD_DIR)
	@touch $@

$(BUILD_DIR)/%.o: src/%.c | $(BUILD_STAMP)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(ELF): $(OBJECTS) stm32f401cc.ld
	$(CC) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

size: $(ELF)
	$(SIZE) $<

toolchain-info:
	@echo "GCC multilib:     $$($(CC) $(CPUFLAGS) -print-multi-directory)"
	@echo "libc_nano:        $$($(CC) $(CPUFLAGS) --specs=nano.specs -print-file-name=libc_nano.a)"
	@echo "libgcc:           $$($(CC) $(CPUFLAGS) -print-libgcc-file-name)"
	@echo "libopencm3:       $(OPENCM3_LIB)"

probe:
	st-info --probe

flash: check-deps $(BIN)
	st-flash --reset write $(BIN) 0x08000000

erase:
	st-flash erase

reset:
	st-flash reset

clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)
