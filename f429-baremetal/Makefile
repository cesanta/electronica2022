TARGET  = firmware
ROOT    ?= $(realpath $(CURDIR)/../../..)
CROSS   ?= arm-none-eabi
CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
						-Wformat-truncation -fno-common -Wconversion \
						-g3 -Os -ffunction-sections -fdata-sections -I. -I$(ROOT) \
            -DMG_ARCH=MG_ARCH_NEWLIB -DMG_IO_SIZE=256 -DMG_ENABLE_PACKED_FS=1 \
						-DMG_ENABLE_CUSTOM_MILLIS=1 -DMG_ENABLE_LINES=1 -DMG_ENABLE_MIP=1 \
						-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = boot.c main.c syscalls.c mongoose.c

all build example: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf
	$(CROSS)-objcopy -O binary $< $@

$(TARGET).elf: $(SOURCES) mcu.h
	$(CROSS)-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

# Note: on "unknown chip id" flash error, wire BOOT0 to VDD and st-flash erase
flash: $(TARGET).bin
	st-flash --reset write $(TARGET).bin 0x8000000

clean:
	@rm -rf $(TARGET).* *.su
