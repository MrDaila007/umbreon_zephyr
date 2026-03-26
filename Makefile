SHELL := /bin/bash

BOARD        ?= rpi_pico2/rp2350a/m33
ZEPHYR_DIR   ?= $(HOME)/zephyrproject
BUILD_DIR    ?= $(ZEPHYR_DIR)/build
SRC_DIR      ?= $(CURDIR)
MOUNT_POINT  ?= /media/$(USER)/RP2350
SERIAL_PORT  ?= /dev/ttyACM0
BAUD         ?= 115200

.PHONY: setup build build-usb flash monitor clean test test-host test-ztest

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

flash:
	cp $(BUILD_DIR)/zephyr/zephyr.uf2 $(MOUNT_POINT)/

monitor:
	picocom $(SERIAL_PORT) -b $(BAUD)

test:
	$(MAKE) -C tests test

test-host:
	$(MAKE) -C tests test-host

test-ztest:
	$(MAKE) -C tests test-ztest

clean:
	rm -rf $(BUILD_DIR)
