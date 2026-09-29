#pragma once

// Settings menu (LVGL), opened with PWR. It runs inside the render task while the
// simulation is paused, drawing through the same panel and DMA band buffers, so
// LVGL never runs concurrently with the fluid renderer.

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"

typedef struct {
    int style;
    int palette;
    bool clock;
    int brightness; // percent
    bool rainbow_unlocked;
} fluid_settings_t;

typedef struct {
    esp_lcd_panel_handle_t panel;
    esp_lcd_touch_handle_t touch; // may be NULL
    uint16_t *bufs[2];            // DMA band buffers, reused as LVGL draw buffers
    int buf_lines;
    int width;
    int height;
    int style_count;
} fluid_menu_config_t;

typedef struct {
    bool reset_fluid;  // "Reiniciar fluido" pressed
    bool time_changed; // hour/minute were set
} fluid_menu_result_t;

// Once, from the render task.
esp_err_t fluid_menu_init(const fluid_menu_config_t *config);

// Blocks until the menu closes (its own button, or close_requested() returning true).
// background: full-screen RGB565 (native byte order) shown behind the card; may be
// NULL. Edits `settings` in place; brightness is applied at once.
void fluid_menu_run(fluid_settings_t *settings, const uint16_t *background, bool (*close_requested)(void),
                    fluid_menu_result_t *result);
