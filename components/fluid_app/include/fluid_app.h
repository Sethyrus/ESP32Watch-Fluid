#pragma once

#include "esp_err.h"

// Brings up the display and touch, then starts the simulation (core 1) and render
// (core 0) tasks. Does not use LVGL.
esp_err_t fluid_app_start(void);
