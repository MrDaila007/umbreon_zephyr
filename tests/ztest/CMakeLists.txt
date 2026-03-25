cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(umbreon_unit_tests)

# ── Production source files under test ────────────────────────────────────────
# Only include files whose logic we actually test.
# Hardware-heavy files (main.c, sensors.c, battery.c, control.c) are excluded —
# their testable logic is extracted into stubs or tested indirectly.

set(APP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/main.c
    src/test_cyc_delta.c
    src/test_map_val.c
    src/test_parse_set.c
    src/test_speed.c
)

# ── Expose static functions for testing ───────────────────────────────────────
target_compile_definitions(app PRIVATE UNIT_TEST=1)

# ── Include production headers ────────────────────────────────────────────────
target_include_directories(app PRIVATE
    ${APP_SRC}
    src
)
