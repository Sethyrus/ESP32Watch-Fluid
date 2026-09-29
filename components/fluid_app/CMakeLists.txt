idf_component_register(
    SRCS "fluid_app.c" "fluid_menu.c" "lv_mem_core_psram.c"
    INCLUDE_DIRS "include"
    REQUIRES
        fluid_engine
        watch_board
        esp_timer
        esp_lcd
        espressif__esp_lcd_touch
        freertos
        heap
        log
        lvgl__lvgl
        nvs_flash
        watch_rtc
        waveshare__esp32_s3_touch_amoled_2_06
)

# lv_mem_core_psram.c implements LVGL's allocator (CONFIG_LV_USE_CUSTOM_MALLOC); link
# this library after LVGL so the linker finds it.
idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
target_link_libraries(${lvgl_lib} INTERFACE ${COMPONENT_LIB})
