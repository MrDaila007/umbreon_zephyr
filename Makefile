SHELL := /bin/bash

BOARD        ?= rpi_pico2/rp2350a/m33
ZEPHYR_DIR   ?= $(HOME)/zephyrproject-v4.4
BUILD_DIR    ?= $(ZEPHYR_DIR)/build
SRC_DIR      ?= $(CURDIR)
MOUNT_POINT  ?= /media/$(USER)/RP2350
SERIAL_PORT  ?= /dev/ttyACM0
PROBE_UART   ?= /dev/ttyACM0
UART0_PORT   ?= /dev/ttyUSB0
BAUD         ?= 115200
OPENOCD      ?= openocd
OPENOCD_IFACE ?= interface/cmsis-dap.cfg
OPENOCD_TARGET ?= target/rp2350.cfg
OPENOCD_SPEED ?= 5000
PYTHON_BIN   ?= python3
HIL_HOST     ?= 127.0.0.1
HIL_REAL_PORT ?= 8023
HIL_SIM_PORT ?= 8123
HIL_SERIAL_PORT ?= /dev/ttyUSB0
HIL_SERIAL_BAUD ?= 115200
HIL_LOG_DIR ?= /tmp
HIL_DURATION ?= 300
HIL_TGF ?= 500
WIFI_HOST ?= 192.168.4.1
WIFI_PORT ?= 23
WIFI_TRANSPORT ?= tcp
WIFI_DURATION ?= 300
WIFI_START ?= 0
WIFI_LOG_DIR ?= /tmp
SIM_PATH     ?= /home/$(USER)/Documents/roborace/simulation/sim.py

.PHONY: setup build build-usb build-hil flash flash-probe flash-stlink monitor monitor-uart0 monitor-probe clean test test-host test-ztest \
	hil-deps hil-real hil-sim hil-dual hil-smoke hil-endurance hil-motor wifi-run-log

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

flash-probe:
	$(OPENOCD) -f $(OPENOCD_IFACE) -f $(OPENOCD_TARGET) \
	-c "adapter speed $(OPENOCD_SPEED)" \
	-c "program $(BUILD_DIR)/zephyr/zephyr.elf verify reset exit"

flash-stlink:
	$(OPENOCD) -f interface/stlink.cfg -f $(OPENOCD_TARGET) \
	-c "program $(BUILD_DIR)/zephyr/zephyr.elf verify reset exit"

monitor:
	minicom -D $(SERIAL_PORT) -b $(BAUD)

monitor-probe:
	minicom -D $(PROBE_UART) -b $(BAUD)

monitor-uart0:
	minicom -D $(UART0_PORT) -b $(BAUD)

test:
	$(MAKE) -C tests test

test-host:
	$(MAKE) -C tests test-host

test-ztest:
	$(MAKE) -C tests test-ztest

clean:
	rm -rf $(BUILD_DIR)

hil-deps:
	$(PYTHON_BIN) -m pip install -r requirements-hil.txt

hil-real:
	PYTHON_BIN=$(PYTHON_BIN) LISTEN_HOST=$(HIL_HOST) LISTEN_PORT=$(HIL_REAL_PORT) \
	SERIAL_PORT=$(HIL_SERIAL_PORT) SERIAL_BAUD=$(HIL_SERIAL_BAUD) \
	bash tools/run_hil_real.sh

hil-sim:
	PYTHON_BIN=$(PYTHON_BIN) SIM_PY=$(SIM_PATH) LISTEN_HOST=$(HIL_HOST) \
	LISTEN_PORT=$(HIL_SIM_PORT) SIM_HOST=$(HIL_HOST) SIM_PORT=$(HIL_REAL_PORT) \
	bash tools/run_hil_sim.sh

hil-dual:
	PYTHON_BIN=$(PYTHON_BIN) SIM_PY=$(SIM_PATH) LISTEN_HOST=$(HIL_HOST) \
	REAL_PORT=$(HIL_REAL_PORT) SIM_LISTEN_PORT=$(HIL_SIM_PORT) \
	SERIAL_PORT=$(HIL_SERIAL_PORT) SERIAL_BAUD=$(HIL_SERIAL_BAUD) \
	SIM_HOST=$(HIL_HOST) SIM_PORT=$(HIL_REAL_PORT) \
	bash tools/run_hil_dual.sh

hil-smoke:
	$(PYTHON_BIN) tools/hil_runner.py --profile smoke \
		--serial-port $(PROBE_UART) --baud $(BAUD) \
		--raw-log $(HIL_LOG_DIR)/umbreon_hil_smoke.log \
		--summary $(HIL_LOG_DIR)/umbreon_hil_smoke.json \
		--max-speed 6.0 --settle-s 12

hil-endurance:
	$(PYTHON_BIN) tools/hil_runner.py --profile endurance \
		--serial-port $(PROBE_UART) --baud $(BAUD) \
		--duration $(HIL_DURATION) --set-battery --tgf $(HIL_TGF) \
		--raw-log $(HIL_LOG_DIR)/umbreon_hil_endurance.log \
		--summary $(HIL_LOG_DIR)/umbreon_hil_endurance.json \
		--max-speed 6.0

hil-motor:
	$(PYTHON_BIN) tools/hil_runner.py --profile motor --allow-motor \
		--serial-port $(PROBE_UART) --baud $(BAUD) \
		--tgf $(HIL_TGF) \
		--raw-log $(HIL_LOG_DIR)/umbreon_hil_motor.log \
		--summary $(HIL_LOG_DIR)/umbreon_hil_motor.json \
		--max-speed 6.0

wifi-run-log:
	$(PYTHON_BIN) tools/wifi_run_logger.py \
		--host $(WIFI_HOST) --port $(WIFI_PORT) --transport $(WIFI_TRANSPORT) \
		--duration $(WIFI_DURATION) \
		$(if $(filter 1 true yes,$(WIFI_START)),--start,) \
		--raw-log $(WIFI_LOG_DIR)/umbreon_wifi_run.log \
		--summary $(WIFI_LOG_DIR)/umbreon_wifi_run.json
