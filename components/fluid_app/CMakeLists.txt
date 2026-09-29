idf_component_register(
    SRCS "fluid_app.c"
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
        waveshare__esp32_s3_touch_amoled_2_06
)
