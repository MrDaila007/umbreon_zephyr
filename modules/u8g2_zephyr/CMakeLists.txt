if(NOT CONFIG_U8G2)
  return()
endif()

# u8g2 sources are in modules/u8g2/ relative to the project root.
# CMAKE_CURRENT_SOURCE_DIR is modules/u8g2_zephyr/, so ../u8g2/ is correct.
set(U8G2_BASE "${CMAKE_CURRENT_SOURCE_DIR}/../u8g2/csrc")

if(NOT EXISTS "${U8G2_BASE}/u8g2.h")
  message(FATAL_ERROR
    "u8g2 sources not found at ${U8G2_BASE}\n"
    "Clone u8g2 with: git clone --depth 1 https://github.com/olikraus/u8g2 modules/u8g2")
endif()

zephyr_library_named(u8g2)

zephyr_library_sources(
  # Core drawing engine
  ${U8G2_BASE}/u8g2_setup.c
  ${U8G2_BASE}/u8g2_d_setup.c
  ${U8G2_BASE}/u8g2_d_memory.c
  ${U8G2_BASE}/u8g2_font.c
  ${U8G2_BASE}/u8g2_fonts.c
  ${U8G2_BASE}/u8g2_buffer.c
  ${U8G2_BASE}/u8g2_hvline.c
  ${U8G2_BASE}/u8g2_box.c
  ${U8G2_BASE}/u8g2_circle.c
  ${U8G2_BASE}/u8g2_line.c
  ${U8G2_BASE}/u8g2_bitmap.c
  ${U8G2_BASE}/u8g2_intersection.c
  ${U8G2_BASE}/u8g2_ll_hvline.c
  ${U8G2_BASE}/u8g2_cleardisplay.c
  ${U8G2_BASE}/u8g2_arc.c
  # u8x8 transport and protocol layer
  ${U8G2_BASE}/u8x8_byte.c
  ${U8G2_BASE}/u8x8_cad.c
  ${U8G2_BASE}/u8x8_gpio.c
  ${U8G2_BASE}/u8x8_string.c
  ${U8G2_BASE}/u8x8_display.c
  ${U8G2_BASE}/u8x8_setup.c
  ${U8G2_BASE}/u8x8_8x8.c
  # SSD1306 128x64 hardware driver
  ${U8G2_BASE}/u8x8_d_ssd1306_128x64_noname.c
)

# Expose u8g2.h and u8x8.h to the entire build
zephyr_include_directories(${U8G2_BASE})
zephyr_library_include_directories(${U8G2_BASE})
