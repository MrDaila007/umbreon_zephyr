if(CONFIG_SSD1306_OLED)
  add_subdirectory(drivers/display)
  zephyr_include_directories(include)
endif()
