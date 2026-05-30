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

# Register out-of-tree VL53L0X enhanced driver module
list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/vl53l0x_enhanced
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(umbreon_zephyr)

file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/VERSION APP_VERSION_LINES)
foreach(version_line ${APP_VERSION_LINES})
  if(version_line MATCHES "^VERSION_MAJOR[ \t]*=[ \t]*([0-9]+)")
    set(APP_VERSION_MAJOR ${CMAKE_MATCH_1})
  elseif(version_line MATCHES "^VERSION_MINOR[ \t]*=[ \t]*([0-9]+)")
    set(APP_VERSION_MINOR ${CMAKE_MATCH_1})
  elseif(version_line MATCHES "^PATCHLEVEL[ \t]*=[ \t]*([0-9]+)")
    set(APP_VERSION_PATCH ${CMAKE_MATCH_1})
  elseif(version_line MATCHES "^VERSION_TWEAK[ \t]*=[ \t]*([0-9]+)")
    set(APP_VERSION_TWEAK ${CMAKE_MATCH_1})
  elseif(version_line MATCHES "^EXTRAVERSION[ \t]*=[ \t]*(.*)")
    set(APP_VERSION_EXTRA ${CMAKE_MATCH_1})
  endif()
endforeach()

set(APP_VERSION_STRING "${APP_VERSION_MAJOR}.${APP_VERSION_MINOR}.${APP_VERSION_PATCH}")
if(APP_VERSION_TWEAK)
  string(APPEND APP_VERSION_STRING ".${APP_VERSION_TWEAK}")
endif()
if(APP_VERSION_EXTRA)
  string(APPEND APP_VERSION_STRING "${APP_VERSION_EXTRA}")
endif()

target_compile_definitions(app PRIVATE
  APP_VERSION_STRING=\"${APP_VERSION_STRING}\"
)

target_include_directories(app PRIVATE
  include
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
  src/tests.c
  src/track_learn.c
  src/encoder.c
  src/display.c
)
