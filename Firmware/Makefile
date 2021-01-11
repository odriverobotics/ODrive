
# This is only a stub for various commands.
# Tup is used for the actual compilation.

BUILD_DIR = build
FIRMWARE = $(BUILD_DIR)/ODriveFirmware.elf
FIRMWARE_HEX = $(BUILD_DIR)/ODriveFirmware.hex
PROGRAMMER_CMD=$(if $(value PROGRAMMER),-c 'hla_serial $(PROGRAMMER)',)

include tup.config # source build configuration to get CONFIG_BOARD_VERSION

ifeq ($(shell python -c "import sys; print(sys.version_info.major)"), 3)
	PY_CMD := python -B
else
	PY_CMD := python3 -B
endif

ifneq (,$(findstring v3.,$(CONFIG_BOARD_VERSION)))
  OPENOCD := openocd -f interface/stlink-v2.cfg $(PROGRAMMER_CMD) -f target/stm32f4x.cfg -c init
  GDB := arm-none-eabi-gdb --ex 'target extended-remote | openocd -f "interface/stlink-v2.cfg" -f "target/stm32f4x.cfg" -c "gdb_port pipe; log_output openocd.log"' --ex 'monitor reset halt'
else ifneq (,$(findstring v4.,$(CONFIG_BOARD_VERSION)))
  OPENOCD := openocd -f interface/stlink.cfg $(PROGRAMMER_CMD) -f target/stm32f7x.cfg -c 'reset_config none separate' -c init
  GDB := arm-none-eabi-gdb --ex 'target extended-remote | openocd -f "interface/stlink-v2.cfg" -f "target/stm32f7x.cfg" -c "reset_config none separate" -c "gdb_port pipe; log_output openocd.log"' --ex 'monitor reset halt'
else
  $(error unknown board version)
endif

$(info board version: $(CONFIG_BOARD_VERSION))

all:
	@tup --quiet --no-environ-check
	@$(PY_CMD) interface_generator_stub.py --definitions odrive-interface.yaml --template ../tools/enums_template.j2 --output ../tools/odrive/enums.py

# Copy libfibre files to odrivetool if they were built
	@ ! compgen -G "fibre-cpp/build/libfibre-*.so" > /dev/null || cp fibre-cpp/build/libfibre-*.so ../tools/odrive/pyfibre/fibre/
	@ ! compgen -G "fibre-cpp/build/libfibre-*.dylib" > /dev/null || cp fibre-cpp/build/libfibre-*.dylib ../tools/odrive/pyfibre/fibre/
	@ ! compgen -G "fibre-cpp/build/libfibre-*.dll" > /dev/null || cp fibre-cpp/build/libfibre-*.dll ../tools/odrive/pyfibre/fibre/

libfibre-linux-armhf:
	docker run -it -v "`pwd`/fibre-cpp":/build -v /tmp/fibre-linux-armhf-build:/build/build -w /build fibre-compiler configs/linux-armhf.config
	cp /tmp/fibre-linux-armhf-build/libfibre-*.so ../tools/odrive/pyfibre/fibre/

libfibre-all:
	docker run -it -v "`pwd`/fibre-cpp":/build -v /tmp/libfibre-build:/build/build -w /build fibre-compiler configs/linux-amd64.config
	cp /tmp/libfibre-build/libfibre-linux-amd64.so ../tools/odrive/pyfibre/fibre/
	docker run -it -v "`pwd`/fibre-cpp":/build -v /tmp/libfibre-build:/build/build -w /build fibre-compiler configs/linux-armhf.config
	cp /tmp/libfibre-build/libfibre-linux-armhf.so ../tools/odrive/pyfibre/fibre/
	docker run -it -v "`pwd`/fibre-cpp":/build -v /tmp/libfibre-build:/build/build -w /build fibre-compiler configs/macos-x86.config
	cp /tmp/libfibre-build/libfibre-macos-x86.dylib ../tools/odrive/pyfibre/fibre/
	docker run -it -v "`pwd`/fibre-cpp":/build -v /tmp/libfibre-build:/build/build -w /build fibre-compiler configs/windows-amd64.config
	cp /tmp/libfibre-build/libfibre-windows-amd64.dll ../tools/odrive/pyfibre/fibre/

clean:
	-rm -fR .dep $(BUILD_DIR)

flash-stlink2: all
	$(OPENOCD) \
		-c 'reset halt' \
		-c 'flash write_image erase $(FIRMWARE)' \
		-c 'reset run' \
		-c exit

gdb-stlink2:
	$(GDB) $(FIRMWARE)

# Erase entire STM32
erase-stlink2:
	$(OPENOCD) -c 'reset halt' -c 'flash erase_sector 0 0 last' -c exit

# Sometimes the STM32 will get it's protection bits set for unknown reasons. Unlock it with this command
unlock-stlink2:
	$(OPENOCD) -c 'reset halt' -c 'stm32f2x unlock 0'

flash-bmp: all
	arm-none-eabi-gdb --ex 'target extended-remote $(BMP_PORT)' \
		--ex 'monitor swdp_scan' \
		--ex 'attach 1' \
		--ex 'load' \
		--ex 'detach' \
		--ex 'quit' \
		$(FIRMWARE)

gdb-bmp: all
	arm-none-eabi-gdb --ex 'target extended-remote /dev/stlink' \
		--ex 'monitor swdp_scan' \
		--ex 'attach 1' \
		--ex 'load' $(FIRMWARE)

dfu: all
	python ../tools/odrivetool $(if $(value SERIAL_NUMBER),--serial-number $(SERIAL_NUMBER),) dfu $(FIRMWARE_HEX)

flash: flash-stlink2
gdb: gdb-stlink2
erase: erase-stlink2
unlock: unlock-stlink2

.PHONY: stlink2-config flash-stlink2 gdb-stlink2 erase-stlink2 unlock-stlink2
.PHONY: flash-bmp gdb-bmp
.PHONY: all clean flash gdb erase unlock dfu fibre
