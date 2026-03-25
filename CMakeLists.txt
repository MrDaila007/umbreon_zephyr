cmake_minimum_required(VERSION 3.20.0)
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
)
