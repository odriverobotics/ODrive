
# This is only a stub for various commands.
# Tup is used for the actual compilation.

BUILD_DIR = build
FIRMWARE = $(BUILD_DIR)/ODriveFirmware.elf
FIRMWARE_HEX = $(BUILD_DIR)/ODriveFirmware.hex

all:
	@tup --quiet

flash: all
	openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c init -c reset\ halt -c flash\ write_image\ erase\ $(FIRMWARE) -c reset\ run -c exit

gdb: all
	arm-none-eabi-gdb $(FIRMWARE) -x openocd.gdbinit

dfu: all
	../tools/dfu.py $(if $(value SERIAL_NUMBER),--serial-number $(SERIAL_NUMBER),) $(FIRMWARE_HEX)

bmp: all
	arm-none-eabi-gdb --ex 'target extended-remote /dev/stlink' \
		--ex 'monitor swdp_scan' \
		--ex 'attach 1' \
		--ex 'load' $(FIRMWARE)

clean:
	-rm -fR .dep $(BUILD_DIR)

.PHONY: all flash gdb dfu bmp clean

