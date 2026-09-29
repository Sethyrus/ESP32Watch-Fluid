# PCF85063 RTC on the shared BSP I2C bus. Local to Fluid for now; a candidate for
# watch_board in ESP32Watch-core once another app needs it.
idf_component_register(
    SRCS "watch_rtc.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        log
        waveshare__esp32_s3_touch_amoled_2_06
)
