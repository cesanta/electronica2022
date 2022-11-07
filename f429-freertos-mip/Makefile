CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -g3 -Os -ffunction-sections -fdata-sections \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
            -I. -Icmsis \
            -DMG_ARCH=MG_ARCH_FREERTOS -DMG_ENABLE_MIP=1 -DMG_IO_SIZE=256 $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c startup.c syscalls.c mongoose.c

# FreeRTOS-Kernel sources and build flags
FREERTOS_KERNEL = FreeRTOS-Kernel
SOURCES += $(wildcard $(FREERTOS_KERNEL)/*.c)
SOURCES += $(FREERTOS_KERNEL)/portable/MemMang/heap_4.c
SOURCES += $(FREERTOS_KERNEL)/portable/GCC/ARM_CM4F/port.c
CFLAGS += -I$(FREERTOS_KERNEL)/include
CFLAGS += -I$(FREERTOS_KERNEL)/portable/GCC/ARM_CM4F
CFLAGS += -Wno-conversion

build: firmware.elf

firmware.elf: $(FREERTOS_KERNEL) $(SOURCES) 
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	$(DOCKER) arm-none-eabi-objcopy -O binary $< $@

flash: firmware.bin
	st-flash --reset write $? 0x8000000

$(FREERTOS_KERNEL):
	git clone --depth 1 -b V10.5.0 https://github.com/FreeRTOS/FreeRTOS-Kernel $@

clean:
	rm -rf firmware.* $(FREERTOS_KERNEL)
