cmake_minimum_required(VERSION 3.20.0)

set(USB_CONSOLE OFF CACHE BOOL "Use USB CDC-ACM for debug console instead of UART0")
if(USB_CONSOLE)
  list(APPEND EXTRA_CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/usb_console.conf)
  list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/usb_console.overlay)
endif()

set(HIL_NO_SENSORS OFF CACHE BOOL "Disable IMU/VL53 nodes for bare-controller HIL")
if(HIL_NO_SENSORS)
  list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/hil_no_sensors.overlay)
endif()

# Register out-of-tree modules
list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/vl53l0x_enhanced
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/u8g2_zephyr
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(umbreon_zephyr)

target_sources(app PRIVATE
    src/main.c
    src/car.c
    src/tachometer.c
    src/sensors.c
    src/imu.c
    src/wifi_cmd.c
    src/control.c
    src/settings.c
    src/battery.c
    src/tests.c
    src/track_learn.c
    src/encoder.c
    src/display.c
    src/display_hal.c
    src/screens/screen_dashboard.c
    src/screens/screen_menu.c
    src/screens/screen_settings.c
    src/screens/screen_edit.c
    src/screens/screen_tests.c
    src/screens/screen_actions.c
    src/screens/screen_confirm.c
    src/screens/screen_info.c
    src/screens/screen_wifi.c
    src/qrcodegen.c
)
