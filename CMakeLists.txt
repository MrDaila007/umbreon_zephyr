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

find_package(Git QUIET)
set(FW_GIT_HASH "unknown")
set(FW_COMMIT_SUBJECT "unknown")
if(Git_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=7 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE FW_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --pretty=%s
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE FW_COMMIT_SUBJECT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} diff --quiet
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE FW_GIT_DIRTY
    ERROR_QUIET
  )
  if(FW_GIT_DIRTY)
    string(APPEND FW_GIT_HASH "*")
  endif()
endif()

string(REPLACE "\\" "\\\\" FW_COMMIT_SUBJECT "${FW_COMMIT_SUBJECT}")
string(REPLACE "\"" "\\\"" FW_COMMIT_SUBJECT "${FW_COMMIT_SUBJECT}")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/version_git.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/version_git.h
  @ONLY
)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/assets
  ${CMAKE_CURRENT_BINARY_DIR}/generated
)

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
    src/buzzer.c
    src/tests.c
    src/encoder.c
    src/display.c
    src/display_hal.c
    src/assets/umbreon_logo.c
    src/screens/screen_boot.c
    src/screens/screen_dashboard.c
    src/screens/screen_menu.c
    src/screens/screen_settings.c
    src/screens/screen_edit.c
    src/screens/screen_tests.c
    src/screens/screen_actions.c
    src/screens/screen_confirm.c
    src/screens/screen_info.c
    src/screens/screen_wifi.c
    src/screens/screen_calibrating.c
    src/qrcodegen.c
)
