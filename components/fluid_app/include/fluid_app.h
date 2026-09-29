#pragma once

#include "esp_err.h"

// Brings up the display, touch, buttons and RTC, then starts the simulation (core 1)
// and render (core 0) tasks. The fluid is drawn straight to the panel; LVGL only
// runs for the settings menu (PWR), inside the render task.
esp_err_t fluid_app_start(void);
