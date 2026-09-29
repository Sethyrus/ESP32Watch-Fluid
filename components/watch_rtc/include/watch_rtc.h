#pragma once

#include <stdbool.h>

#include "esp_err.h"

// PCF85063 real-time clock (I2C 0x51) on the BSP bus. The RTC keeps local time (no
// time zone); init copies it into the system clock, so time()/localtime_r() work
// afterwards without further I2C traffic.

// If the RTC lost its time (oscillator-stop flag), or set_from_build is true, the
// firmware build time (__DATE__ __TIME__ of watch_rtc.c) is written first.
esp_err_t watch_rtc_init(bool set_from_build);

// Sets hours and minutes (seconds to 0, date kept) in the RTC and the system clock.
esp_err_t watch_rtc_set_time(int hours, int minutes);
