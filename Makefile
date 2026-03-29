SHELL := /bin/bash

BOARD        ?= rpi_pico2/rp2350a/m33
ZEPHYR_DIR   ?= $(HOME)/zephyrproject-v4.3
BUILD_DIR    ?= $(ZEPHYR_DIR)/build
SRC_DIR      ?= $(CURDIR)
MOUNT_POINT  ?= /media/$(USER)/RP2350
SERIAL_PORT  ?= /dev/ttyACM0
UART0_PORT   ?= /dev/ttyUSB0
BAUD         ?= 115200
OPENOCD      ?= openocd
OPENOCD_IFACE ?= interface/stlink.cfg
OPENOCD_TARGET ?= target/rp2350.cfg

.PHONY: setup build build-usb build-hil flash flash-stlink monitor monitor-uart0 clean test test-host test-ztest

setup:
	./setup_zephyr.sh $(ZEPHYR_DIR)

build:
	source $(ZEPHYR_DIR)/.venv/bin/activate && \
	cd $(ZEPHYR_DIR) && \
	west build -b $(BOARD) $(SRC_DIR) --pristine always

build-usb:
	source $(ZEPHYR_DIR)/.venv/bin/activate && \
	cd $(ZEPHYR_DIR) && \
	west build -b $(BOARD) $(SRC_DIR) --pristine always -- -DUSB_CONSOLE=ON

build-hil:
	source $(ZEPHYR_DIR)/.venv/bin/activate && \
	cd $(ZEPHYR_DIR) && \
	west build -b $(BOARD) $(SRC_DIR) --pristine always -- -DHIL_NO_SENSORS=ON

flash:
	cp $(BUILD_DIR)/zephyr/zephyr.uf2 $(MOUNT_POINT)/

flash-stlink:
	$(OPENOCD) -f $(OPENOCD_IFACE) -f $(OPENOCD_TARGET) \
	-c "program $(BUILD_DIR)/zephyr/zephyr.elf verify reset exit"

monitor:
	picocom $(SERIAL_PORT) -b $(BAUD)

monitor-uart0:
	picocom $(UART0_PORT) -b $(BAUD)

test:
	$(MAKE) -C tests test

test-host:
	$(MAKE) -C tests test-host

test-ztest:
	$(MAKE) -C tests test-ztest

clean:
	rm -rf $(BUILD_DIR)
