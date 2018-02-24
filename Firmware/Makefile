
# This is only a stub for various commands.
# Tup is used for the actual compilation.

FIRMWARE = build/ODriveFirmware.elf

all:
	tup

flash: all
	openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c init -c reset\ halt -c flash\ write_image\ erase\ $(FIRMWARE) -c reset\ run -c exit

gdb: all
	arm-none-eabi-gdb $(FIRMWARE) -x openocd.gdbinit

bmp: all
	arm-none-eabi-gdb --ex 'target extended-remote /dev/stlink' \
		--ex 'monitor swdp_scan' \
		--ex 'attach 1' \
		--ex 'load' $(FIRMWARE)

.PHONY: all flash gdb bmp

