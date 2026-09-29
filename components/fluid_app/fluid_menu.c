#include "fluid_menu.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "bsp/display.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fluid_render.h"
#include "lvgl.h"
#include "watch_rtc.h"

// Built-in Montserrat fonts have no accented letters, hence "Liquido", "Toxico".
static const char *const s_style_names[] = {"LED", "Pixel", "Liquido"};
static const char *const s_palette_names[FLUID_PALETTE_COUNT] = {"Agua", "Lava", "Toxico", "Arcoiris"};

#define MENU_ACCENT 0x38bdf8
#define MENU_CARD_BG 0x0f172a
#define MENU_BUTTON_BG 0x1f7a8c
#define MENU_DANGER_BG 0x9f1239

static const char *TAG = "fluid_menu";

static struct {
    fluid_menu_config_t cfg;
    lv_display_t *display;
    bool ready;

    // State of the open menu.
    fluid_settings_t *settings;
    fluid_menu_result_t *result;
    bool closing;
    lv_image_dsc_t background;
    lv_obj_t *card;
    lv_obj_t *style_label;
    lv_obj_t *palette_label;
    lv_obj_t *time_label;
    lv_obj_t *time_card;
    lv_obj_t *hour_roller;
    lv_obj_t *minute_roller;
} s_menu;

static uint32_t tick_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Two draw buffers: LVGL renders into one while the other is on the bus, and
// draw_bitmap drains the previous transfer before starting the next, so flushing can
// report ready at once.
static void flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    int w = lv_area_get_width(area);
    int h = lv_area_get_height(area);
    lv_draw_sw_rgb565_swap(px_map, (uint32_t)(w * h));
    esp_err_t err = esp_lcd_panel_draw_bitmap(s_menu.cfg.panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1,
                                              px_map);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "draw failed: %s", esp_err_to_name(err));
    }
    lv_display_flush_ready(display);
}

// The panel needs even start and odd end coordinates (same as the BSP's LVGL port).
static void rounder_cb(lv_event_t *e)
{
    lv_area_t *area = lv_event_get_param(e);
    area->x1 &= ~1;
    area->y1 &= ~1;
    area->x2 |= 1;
    area->y2 |= 1;
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    data->state = LV_INDEV_STATE_RELEASED;
    if (s_menu.cfg.touch == NULL || esp_lcd_touch_read_data(s_menu.cfg.touch) != ESP_OK) {
        return; // the FT3168 NACKs while idle: treat as released
    }
    esp_lcd_touch_point_data_t point = {0};
    uint8_t count = 0;
    if (esp_lcd_touch_get_data(s_menu.cfg.touch, &point, &count, 1) == ESP_OK && count > 0) {
        data->point.x = point.x;
        data->point.y = point.y;
        data->state = LV_INDEV_STATE_PRESSED;
    }
}

esp_err_t fluid_menu_init(const fluid_menu_config_t *config)
{
    s_menu.cfg = *config;
    lv_init();
    lv_tick_set_cb(tick_ms);

    lv_display_t *d = lv_display_create(config->width, config->height);
    if (d == NULL) {
        return ESP_ERR_NO_MEM;
    }
    lv_display_set_color_format(d, LV_COLOR_FORMAT_RGB565);
    uint32_t bytes = (uint32_t)(config->width * config->buf_lines * 2);
    lv_display_set_buffers(d, config->bufs[0], config->bufs[1], bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(d, flush_cb);
    lv_display_add_event_cb(d, rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);

    lv_indev_t *indev = lv_indev_create();
    if (indev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);

    s_menu.display = d;
    s_menu.ready = true;
    return ESP_OK;
}

static void set_font(lv_obj_t *obj, int size)
{
#if LV_FONT_MONTSERRAT_28
    if (size >= 28) {
        lv_obj_set_style_text_font(obj, &lv_font_montserrat_28, LV_PART_MAIN);
        return;
    }
#endif
#if LV_FONT_MONTSERRAT_20
    if (size >= 20) {
        lv_obj_set_style_text_font(obj, &lv_font_montserrat_20, LV_PART_MAIN);
        return;
    }
#endif
    (void)obj;
    (void)size;
}

static lv_obj_t *create_label(lv_obj_t *parent, const char *text, int font)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_hex(0xf8fafc), LV_PART_MAIN);
    set_font(label, font);
    return label;
}

// Same look as the Maze buttons, without the shadow (software shadows are slow).
static lv_obj_t *create_button(lv_obj_t *parent, const char *text, int width, int height, uint32_t color,
                               lv_event_cb_t cb, lv_obj_t **label_out)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, width, height);
    lv_obj_set_style_radius(button, 18, LV_PART_MAIN);
    lv_obj_set_style_bg_color(button, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(button, 0, LV_PART_MAIN);
    lv_obj_remove_flag(button, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(button, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label = create_label(button, text, 20);
    lv_obj_center(label);
    if (label_out != NULL) {
        *label_out = label;
    }
    return button;
}

static lv_obj_t *create_card(lv_obj_t *parent, int width, int height)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, width, height);
    lv_obj_set_style_bg_color(card, lv_color_hex(MENU_CARD_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(card, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_border_color(card, lv_color_hex(MENU_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_border_width(card, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(card, 28, LV_PART_MAIN);
    lv_obj_set_style_pad_all(card, 14, LV_PART_MAIN);
    lv_obj_set_style_pad_row(card, 8, LV_PART_MAIN);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_center(card);
    return card;
}

// Transparent full-width row: caption on the left, control on the right.
static lv_obj_t *create_row(lv_obj_t *parent, const char *caption)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, lv_pct(100), 44);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    if (caption != NULL) {
        create_label(row, caption, 20);
    }
    return row;
}

static int palette_count(void)
{
    return s_menu.settings->rainbow_unlocked ? FLUID_PALETTE_COUNT : FLUID_PALETTE_RAINBOW;
}

static void update_time_label(void)
{
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    lv_label_set_text_fmt(s_menu.time_label, "%02d:%02d", tm.tm_hour, tm.tm_min);
}

static void style_clicked(lv_event_t *e)
{
    (void)e;
    fluid_settings_t *st = s_menu.settings;
    st->style = (st->style + 1) % s_menu.cfg.style_count;
    lv_label_set_text(s_menu.style_label, s_style_names[st->style]);
}

static void palette_clicked(lv_event_t *e)
{
    (void)e;
    fluid_settings_t *st = s_menu.settings;
    st->palette = (st->palette + 1) % palette_count();
    lv_label_set_text(s_menu.palette_label, s_palette_names[st->palette]);
}

static void clock_changed(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    s_menu.settings->clock = lv_obj_has_state(sw, LV_STATE_CHECKED);
}

static void brightness_changed(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    s_menu.settings->brightness = (int)lv_slider_get_value(slider);
    bsp_display_brightness_set(s_menu.settings->brightness);
}

static void reset_clicked(lv_event_t *e)
{
    (void)e;
    s_menu.result->reset_fluid = true;
    s_menu.closing = true;
}

static void exit_clicked(lv_event_t *e)
{
    (void)e;
    s_menu.result->exit_app = true;
    s_menu.closing = true;
}

static void continue_clicked(lv_event_t *e)
{
    (void)e;
    s_menu.closing = true;
}

static void close_time_card(void)
{
    if (s_menu.time_card != NULL) {
        lv_obj_delete(s_menu.time_card);
        s_menu.time_card = NULL;
    }
    lv_obj_remove_flag(s_menu.card, LV_OBJ_FLAG_HIDDEN);
}

static void time_save_clicked(lv_event_t *e)
{
    (void)e;
    int h = (int)lv_roller_get_selected(s_menu.hour_roller);
    int m = (int)lv_roller_get_selected(s_menu.minute_roller);
    esp_err_t err = watch_rtc_set_time(h, m);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RTC write failed (%s); time kept until reboot", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "Time set to %02d:%02d", h, m);
    s_menu.result->time_changed = true;
    close_time_card();
    update_time_label();
}

static void time_cancel_clicked(lv_event_t *e)
{
    (void)e;
    close_time_card();
}

static lv_obj_t *create_roller(lv_obj_t *parent, int count, int selected)
{
    static char options[60 * 3 + 1];
    int len = 0;
    for (int i = 0; i < count; i++) {
        len += snprintf(options + len, sizeof(options) - (size_t)len, i ? "\n%02d" : "%02d", i);
    }
    lv_obj_t *roller = lv_roller_create(parent);
    lv_roller_set_options(roller, options, LV_ROLLER_MODE_NORMAL); // copies the string
    lv_roller_set_visible_row_count(roller, 3);
    lv_roller_set_selected(roller, (uint32_t)selected, LV_ANIM_OFF);
    lv_obj_set_width(roller, 96);
    lv_obj_set_style_bg_color(roller, lv_color_hex(0x1e293b), LV_PART_MAIN);
    lv_obj_set_style_text_color(roller, lv_color_hex(0x94a3b8), LV_PART_MAIN);
    lv_obj_set_style_border_width(roller, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(roller, lv_color_hex(MENU_BUTTON_BG), LV_PART_SELECTED);
    lv_obj_set_style_text_color(roller, lv_color_hex(0xf8fafc), LV_PART_SELECTED);
    set_font(roller, 28);
    return roller;
}

static void time_clicked(lv_event_t *e)
{
    (void)e;
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);

    lv_obj_add_flag(s_menu.card, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *card = create_card(lv_screen_active(), 320, 330);
    s_menu.time_card = card;
    create_label(card, "Ajustar hora", 28);

    lv_obj_t *rollers = create_row(card, NULL);
    lv_obj_set_height(rollers, 170);
    lv_obj_set_flex_align(rollers, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    s_menu.hour_roller = create_roller(rollers, 24, tm.tm_hour);
    create_label(rollers, ":", 28);
    s_menu.minute_roller = create_roller(rollers, 60, tm.tm_min);

    lv_obj_t *buttons = create_row(card, NULL);
    lv_obj_set_height(buttons, 50);
    create_button(buttons, "Cancelar", 130, 46, 0x334155, time_cancel_clicked, NULL);
    create_button(buttons, "Guardar", 130, 46, MENU_BUTTON_BG, time_save_clicked, NULL);
}

static void build_menu(const uint16_t *background)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_clean(screen);
    lv_obj_set_style_bg_color(screen, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    if (background != NULL) {
        lv_image_dsc_t *dsc = &s_menu.background;
        memset(dsc, 0, sizeof(*dsc));
        dsc->header.magic = LV_IMAGE_HEADER_MAGIC;
        dsc->header.cf = LV_COLOR_FORMAT_RGB565;
        dsc->header.w = (uint32_t)s_menu.cfg.width;
        dsc->header.h = (uint32_t)s_menu.cfg.height;
        dsc->header.stride = (uint32_t)(s_menu.cfg.width * 2);
        dsc->data_size = (uint32_t)(s_menu.cfg.width * s_menu.cfg.height * 2);
        dsc->data = (const uint8_t *)background;
        lv_image_cache_drop(dsc); // same descriptor, new pixels on every open
        lv_obj_t *img = lv_image_create(screen);
        lv_image_set_src(img, dsc);
        lv_obj_set_pos(img, 0, 0);
    }

    fluid_settings_t *st = s_menu.settings;
    lv_obj_t *card = create_card(screen, 340, s_menu.cfg.show_exit ? 466 : 410);
    s_menu.card = card;
    create_label(card, "Fluido", 28);

    lv_obj_t *row = create_row(card, "Estilo");
    create_button(row, s_style_names[st->style], 150, 42, MENU_BUTTON_BG, style_clicked, &s_menu.style_label);

    row = create_row(card, "Paleta");
    create_button(row, s_palette_names[st->palette], 150, 42, MENU_BUTTON_BG, palette_clicked,
                  &s_menu.palette_label);

    row = create_row(card, "Reloj");
    lv_obj_t *sw = lv_switch_create(row);
    lv_obj_set_size(sw, 70, 36);
    lv_obj_set_style_bg_color(sw, lv_color_hex(MENU_BUTTON_BG), LV_PART_INDICATOR | LV_STATE_CHECKED);
    if (st->clock) {
        lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_add_event_cb(sw, clock_changed, LV_EVENT_VALUE_CHANGED, NULL);

    row = create_row(card, "Hora");
    create_button(row, "", 150, 42, MENU_BUTTON_BG, time_clicked, &s_menu.time_label);
    update_time_label();

    row = create_row(card, "Brillo");
    lv_obj_t *slider = lv_slider_create(row);
    lv_obj_set_width(slider, 150);
    lv_slider_set_range(slider, 10, 100);
    lv_slider_set_value(slider, st->brightness, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(slider, lv_color_hex(MENU_BUTTON_BG), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_hex(MENU_ACCENT), LV_PART_KNOB);
    lv_obj_add_event_cb(slider, brightness_changed, LV_EVENT_VALUE_CHANGED, NULL);

    row = create_row(card, NULL);
    lv_obj_set_height(row, 50);
    create_button(row, "Reiniciar", 140, 46, MENU_DANGER_BG, reset_clicked, NULL);
    create_button(row, "Continuar", 140, 46, MENU_BUTTON_BG, continue_clicked, NULL);

    if (s_menu.cfg.show_exit) {
        create_button(card, "Salir al launcher", 296, 46, 0x334155, exit_clicked, NULL);
    }
}

void fluid_menu_run(fluid_settings_t *settings, const uint16_t *background, bool (*close_requested)(void),
                    fluid_menu_result_t *result)
{
    memset(result, 0, sizeof(*result));
    if (!s_menu.ready) {
        return;
    }
    s_menu.settings = settings;
    s_menu.result = result;
    s_menu.closing = false;
    s_menu.time_card = NULL;
    if (settings->palette >= palette_count()) {
        settings->palette = 0;
    }
    build_menu(background);
    lv_obj_invalidate(lv_screen_active());

    int last_minute = -1;
    while (!s_menu.closing) {
        uint32_t wait_ms = lv_timer_handler();
        if (close_requested()) {
            s_menu.closing = true;
        }
        time_t now = time(NULL);
        if (now / 60 != last_minute) {
            last_minute = (int)(now / 60);
            update_time_label();
        }
        if (wait_ms > 20) {
            wait_ms = 20;
        }
        vTaskDelay(pdMS_TO_TICKS(wait_ms > 0 ? wait_ms : 1));
    }
    lv_obj_clean(lv_screen_active());
    s_menu.card = NULL;
    s_menu.time_card = NULL;
}
