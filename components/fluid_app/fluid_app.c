#include "fluid_app.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "bsp/display.h"
#include "bsp/esp-bsp.h"
#include "bsp/touch.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fluid_raster.h"
#include "fluid_render.h"
#include "fluid_sim.h"
#include "imu_service.h"
#include "watch_buttons.h"

#define FLUID_W BSP_LCD_H_RES
#define FLUID_H BSP_LCD_V_RES
#define FLUID_BAND_LINES 16 // even, as the panel needs even row starts
#define FLUID_FPS 60
#define FLUID_SIM_CORE 1
#define FLUID_RENDER_CORE 0
#define FLUID_TASK_PRIORITY 5
#define FLUID_SIM_STACK 6144
#define FLUID_RENDER_STACK 4096
#define FLUID_HELPER_STACK 3072
#define FLUID_HELPER_PRIORITY (FLUID_TASK_PRIORITY + 1)
// Render above the helper: it mostly sleeps on the display DMA and, when a band is
// done, must refill the bus at once or the panel link sits idle.
#define FLUID_RENDER_PRIORITY (FLUID_TASK_PRIORITY + 2)
#define FLUID_BOOT_DEBOUNCE_US 30000
#define FLUID_BOOT_LONG_US 700000
#define FLUID_FINGER_RADIUS 30.0f
#define FLUID_FINGER_MAX_SPEED 3000.0f
#define FLUID_STATS_US 5000000
#define FLUID_DT_MIN (1.0f / 120.0f)
#define FLUID_DT_MAX (1.0f / 30.0f)
#define FLUID_TOUCH_RETRY_US 100000

static const char *TAG = "fluid_app";

// Triple buffer between the tasks: the simulation fills `write`, publishes it as
// `ready`, and the renderer takes the latest `ready` into `read`. Neither task waits
// for the other.
typedef struct {
    uint8_t *level;
    uint8_t *foam;
    fluid_style_t style;
    int palette;
} fluid_frame_t;

static fluid_frame_t s_frames[3];
static int s_write = 0;
static int s_ready = 1;
static int s_read = 2;
static bool s_fresh;
static portMUX_TYPE s_frame_lock = portMUX_INITIALIZER_UNLOCKED;

static fluid_t *s_fluid;
static fluid_raster_t s_raster_led;  // FLUID_STYLE_LED pitch
static fluid_raster_t s_raster_fine; // pixel and liquid share a pitch
static fluid_render_t s_render;
static esp_lcd_panel_handle_t s_panel;
static esp_lcd_touch_handle_t s_touch;
static uint16_t *s_dma[2];
static int s_dma_next;
static TaskHandle_t s_render_task;

// Second half of each simulation stage runs on core 0 (fluid_config_t.parallel). The
// render task there mostly waits on the display DMA, so the helper borrows that time.
static TaskHandle_t s_helper_task;
static TaskHandle_t s_sim_task;
static fluid_job_fn s_job_fn;
static void *s_job_ctx;

// Hot buffers go to internal RAM; PSRAM only as a fallback, and it is logged.
static void *alloc_fast(size_t bytes)
{
    void *p = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p == NULL) {
        p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (p != NULL) {
            ESP_LOGW(TAG, "Internal RAM short: %u B placed in PSRAM", (unsigned)bytes);
        }
    }
    return p;
}

static void *alloc_psram(size_t bytes)
{
    void *p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p != NULL ? p : heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
}

static int64_t clock_us(void)
{
    return esp_timer_get_time();
}

static void helper_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        s_job_fn(s_job_ctx, 1);
        xTaskNotifyGive(s_sim_task);
    }
}

// Called only from the sim task. Task notifications order the memory accesses.
static void run_parallel(fluid_job_fn fn, void *ctx)
{
    s_job_fn = fn;
    s_job_ctx = ctx;
    xTaskNotifyGive(s_helper_task);
    fn(ctx, 0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

static fluid_raster_t *raster_for(fluid_style_t style)
{
    return style == FLUID_STYLE_LED ? &s_raster_led : &s_raster_fine;
}

static esp_err_t init_display(void)
{
    const bsp_display_config_t config = {
        .max_transfer_sz = FLUID_W * FLUID_BAND_LINES * BSP_LCD_BITS_PER_PIXEL / 8,
    };
    esp_err_t err = bsp_display_new(&config, &s_panel, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_display_new failed: %s", esp_err_to_name(err));
        return err;
    }

    for (int i = 0; i < 2; i++) {
        s_dma[i] = heap_caps_malloc(FLUID_W * FLUID_BAND_LINES * sizeof(uint16_t),
                                    MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (s_dma[i] == NULL) {
            ESP_LOGE(TAG, "DMA band buffer allocation failed");
            return ESP_ERR_NO_MEM;
        }
    }

    // Clear whatever the panel RAM held before turning the brightness up.
    memset(s_dma[0], 0, FLUID_W * FLUID_BAND_LINES * sizeof(uint16_t));
    for (int y = 0; y < FLUID_H; y += FLUID_BAND_LINES) {
        int lines = FLUID_H - y < FLUID_BAND_LINES ? FLUID_H - y : FLUID_BAND_LINES;
        err = esp_lcd_panel_draw_bitmap(s_panel, 0, y, FLUID_W, y + lines, s_dma[0]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Screen clear failed: %s", esp_err_to_name(err));
            return err;
        }
    }
    s_dma_next = 1;

    err = bsp_display_brightness_set(CONFIG_FLUID_BRIGHTNESS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Brightness set failed: %s", esp_err_to_name(err));
    }
    return ESP_OK;
}

static esp_err_t init_engine(void)
{
    fluid_config_t config = fluid_default_config();
    config.width = FLUID_W;
    config.height = FLUID_H;
    config.cell_size = (float)CONFIG_FLUID_CELL_PX;
    config.fill_fraction = (float)CONFIG_FLUID_FILL_PERCENT / 100.0f;
    config.pressure_iters = CONFIG_FLUID_PRESSURE_ITERS;
    config.max_particles = 12000;
    // dt follows real frame time (up to 1/30 s), so allow faster particles per step.
    config.max_cells_per_step = 3.0f;
    config.alloc = alloc_fast;
    config.clock_us = clock_us;
    config.parallel = run_parallel;
    s_fluid = fluid_create(&config);
    if (s_fluid == NULL) {
        ESP_LOGE(TAG, "fluid_create failed (out of memory?)");
        return ESP_ERR_NO_MEM;
    }

    if (!fluid_raster_init(&s_raster_led, FLUID_W, FLUID_H, fluid_style_pitch(FLUID_STYLE_LED), alloc_fast) ||
        !fluid_raster_init(&s_raster_fine, FLUID_W, FLUID_H, fluid_style_pitch(FLUID_STYLE_LIQUID), alloc_fast)) {
        ESP_LOGE(TAG, "Raster allocation failed");
        return ESP_ERR_NO_MEM;
    }
    s_raster_fine.blur = true;

    int max_cells = 0;
    int max_pitch = 0;
    for (int s = 0; s < FLUID_STYLE_COUNT; s++) {
        const fluid_raster_t *r = raster_for((fluid_style_t)s);
        if (r->cols * r->rows > max_cells) {
            max_cells = r->cols * r->rows;
        }
        if (r->pitch > max_pitch) {
            max_pitch = r->pitch;
        }
    }
    if (!fluid_render_init(&s_render, FLUID_W, FLUID_H, max_cells, max_pitch, true, alloc_fast)) {
        ESP_LOGE(TAG, "Renderer allocation failed");
        return ESP_ERR_NO_MEM;
    }

    // The frames are read once per frame in order; PSRAM keeps internal RAM for the sim.
    for (int i = 0; i < 3; i++) {
        s_frames[i].level = alloc_psram((size_t)max_cells);
        s_frames[i].foam = alloc_psram((size_t)max_cells);
        if (s_frames[i].level == NULL || s_frames[i].foam == NULL) {
            ESP_LOGE(TAG, "Frame buffer allocation failed");
            return ESP_ERR_NO_MEM;
        }
        memset(s_frames[i].level, 0, (size_t)max_cells);
        memset(s_frames[i].foam, 0, (size_t)max_cells);
    }

    const fluid_stats_t *st = fluid_get_stats(s_fluid);
    ESP_LOGI(TAG, "Fluid: cell=%dpx grid=%dx%d particles=%d iters=%d substeps=%d",
             CONFIG_FLUID_CELL_PX, st->cells_x, st->cells_y, st->particles, CONFIG_FLUID_PRESSURE_ITERS,
             CONFIG_FLUID_SUBSTEPS);
    return ESP_OK;
}

// Short press and long press (fires while held) with debounce.
static void poll_boot(int64_t now, bool *short_press, bool *long_press)
{
    static bool raw;
    static bool stable;
    static bool long_fired;
    static int64_t changed_at;
    static int64_t down_at;

    bool pressed = watch_boot_button_is_pressed();
    if (pressed != raw) {
        raw = pressed;
        changed_at = now;
    }
    if (now - changed_at >= FLUID_BOOT_DEBOUNCE_US && raw != stable) {
        stable = raw;
        if (stable) {
            down_at = now;
            long_fired = false;
        } else if (!long_fired) {
            *short_press = true;
        }
    }
    if (stable && !long_fired && now - down_at >= FLUID_BOOT_LONG_US) {
        long_fired = true;
        *long_press = true;
    }
}

// The finger is a moving circular obstacle; its velocity comes from frame deltas.
// The FT3168 NACKs reads while idle (low-power monitor mode), so after a failed read
// it is only polled every FLUID_TOUCH_RETRY_US; a touch wakes it and restores
// per-frame reads.
static void poll_touch(int64_t now, float dt, uint32_t *errors)
{
    static bool down;
    static float last_x;
    static float last_y;
    static int64_t retry_at;

    bool touching = false;
    esp_lcd_touch_point_data_t point = {0};
    if (s_touch != NULL && now >= retry_at) {
        if (esp_lcd_touch_read_data(s_touch) == ESP_OK) {
            uint8_t count = 0;
            touching = esp_lcd_touch_get_data(s_touch, &point, &count, 1) == ESP_OK && count > 0;
        } else {
            (*errors)++;
            retry_at = now + FLUID_TOUCH_RETRY_US;
        }
    }
    if (!touching) {
        down = false;
        fluid_set_obstacle(s_fluid, 0, 0, 0, 0, 0, false);
        return;
    }

    float x = (float)point.x;
    float y = (float)point.y;
    float vx = down ? (x - last_x) / dt : 0.0f;
    float vy = down ? (y - last_y) / dt : 0.0f;
    if (vx > FLUID_FINGER_MAX_SPEED) vx = FLUID_FINGER_MAX_SPEED;
    if (vx < -FLUID_FINGER_MAX_SPEED) vx = -FLUID_FINGER_MAX_SPEED;
    if (vy > FLUID_FINGER_MAX_SPEED) vy = FLUID_FINGER_MAX_SPEED;
    if (vy < -FLUID_FINGER_MAX_SPEED) vy = -FLUID_FINGER_MAX_SPEED;
    down = true;
    last_x = x;
    last_y = y;
    fluid_set_obstacle(s_fluid, x, y, FLUID_FINGER_RADIUS, vx, vy, true);
}

static void sim_task(void *arg)
{
    (void)arg;
    // imu_service is single-task: init and reads both live here.
    esp_err_t err = imu_service_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "IMU unavailable (%s); gravity fixed downwards", esp_err_to_name(err));
    }

    const float g_scale = (float)CONFIG_FLUID_GRAVITY_PX_PER_G;
    fluid_style_t style = FLUID_STYLE_LED;
    int palette = 0;
    uint32_t frame = 0;
    int64_t next_us = esp_timer_get_time();
    int64_t last_step_us = next_us - 1000000 / FLUID_FPS;

    // Per-window stats; stage[] follows the fluid_stats_t order.
    static const char *const stage_names[7] = {"int", "sep", "col", "p2g", "den", "prs", "g2p"};
    uint32_t frames = 0;
    uint32_t overruns = 0;
    uint32_t imu_errors = 0;
    uint32_t touch_errors = 0;
    uint64_t input_total = 0;
    uint64_t sim_total = 0;
    uint32_t sim_max = 0;
    uint64_t raster_total = 0;
    uint64_t dt_total = 0;
    uint64_t stage[7] = {0};
    int64_t last_log = next_us;

    for (;;) {
        int64_t t0 = esp_timer_get_time();

        // Real elapsed time keeps the fluid at physical speed whatever the frame rate.
        // Clamped so a stall slows the fluid down instead of destabilising it.
        float dt = (float)(t0 - last_step_us) * 1e-6f;
        dt = dt < FLUID_DT_MIN ? FLUID_DT_MIN : (dt > FLUID_DT_MAX ? FLUID_DT_MAX : dt);
        last_step_us = t0;

        bool short_press = false;
        bool long_press = false;
        poll_boot(t0, &short_press, &long_press);
        if (short_press) {
            style = (fluid_style_t)((style + 1) % FLUID_STYLE_COUNT);
            fluid_raster_clear(raster_for(style));
            ESP_LOGI(TAG, "Style: %s", fluid_style_name(style));
        }
        if (long_press) {
            palette = (palette + 1) % FLUID_PALETTE_COUNT;
            ESP_LOGI(TAG, "Palette: %s", fluid_palette_name(palette));
        }

        // Absolute gravity: imu_service is never calibrated, so its bias stays 0 and it
        // reports the real gravity projected on the screen (plus any shaking).
        float gx = 0.0f;
        float gy = g_scale;
        imu_service_accel_t accel;
        if (imu_service_is_available()) {
            if (imu_service_read(&accel) == ESP_OK) {
                gx = accel.x * g_scale;
                gy = accel.y * g_scale;
            } else {
                imu_errors++;
            }
        }
        poll_touch(t0, dt, &touch_errors);
        int64_t t_in = esp_timer_get_time();

        fluid_step(s_fluid, gx, gy, dt, CONFIG_FLUID_SUBSTEPS);
        int64_t t1 = esp_timer_get_time();

        fluid_frame_t *out = &s_frames[s_write];
        fluid_raster_run(raster_for(style), s_fluid, out->level, out->foam);
        out->style = style;
        out->palette = palette;
        int64_t t2 = esp_timer_get_time();

        portENTER_CRITICAL(&s_frame_lock);
        int tmp = s_ready;
        s_ready = s_write;
        s_write = tmp;
        s_fresh = true;
        portEXIT_CRITICAL(&s_frame_lock);
        xTaskNotifyGive(s_render_task);

        const fluid_stats_t *st = fluid_get_stats(s_fluid);
        uint32_t sim_us = (uint32_t)(t1 - t_in);
        frames++;
        input_total += (uint64_t)(t_in - t0);
        sim_total += sim_us;
        sim_max = sim_us > sim_max ? sim_us : sim_max;
        raster_total += (uint64_t)(t2 - t1);
        dt_total += (uint64_t)(dt * 1e6f);
        stage[0] += st->us_integrate;
        stage[1] += st->us_separate;
        stage[2] += st->us_collide;
        stage[3] += st->us_p2g;
        stage[4] += st->us_density;
        stage[5] += st->us_pressure;
        stage[6] += st->us_g2p;
        if (t2 - last_log >= FLUID_STATS_US) {
            char stages[96];
            int len = 0;
            for (int i = 0; i < 7; i++) {
                len += snprintf(stages + len, sizeof(stages) - (size_t)len, "%s%s=%" PRIu32, i ? " " : "",
                                stage_names[i], (uint32_t)(stage[i] / frames));
            }
            ESP_LOGI(TAG,
                     "sim: fps=%" PRIu32 " step_avg=%" PRIu32 "us max=%" PRIu32 "us (%s) input=%" PRIu32
                     "us raster=%" PRIu32 "us dt_avg=%" PRIu32 "us overruns=%" PRIu32 " imu_err=%" PRIu32
                     " touch_err=%" PRIu32 " fluid_cells=%d internal_free=%u min=%u",
                     (uint32_t)((uint64_t)frames * 1000000 / (uint64_t)(t2 - last_log)),
                     (uint32_t)(sim_total / frames), sim_max, stages, (uint32_t)(input_total / frames),
                     (uint32_t)(raster_total / frames), (uint32_t)(dt_total / frames), overruns, imu_errors,
                     touch_errors, st->fluid_cells, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
            frames = overruns = imu_errors = touch_errors = sim_max = 0;
            input_total = sim_total = raster_total = dt_total = 0;
            memset(stage, 0, sizeof(stage));
            last_log = t2;
        }

        // Fixed 60 Hz pacing. Always yield at least one tick so the idle task on this
        // core (task watchdog) runs even when a step overruns.
        frame++;
        next_us += 1000000 / FLUID_FPS + ((frame % 3) == 0 ? 1 : 0);
        int64_t now = esp_timer_get_time();
        int64_t wait_us = next_us - now;
        if (wait_us < 1000) {
            overruns++;
            if (wait_us < -(int64_t)(1000000 / FLUID_FPS)) {
                next_us = now; // too far behind: drop the backlog instead of catching up
            }
            vTaskDelay(1);
        } else {
            vTaskDelay(pdMS_TO_TICKS(wait_us / 1000));
        }
    }
}

static void render_task(void *arg)
{
    (void)arg;
    fluid_style_t style = FLUID_STYLE_COUNT; // forces the first configure
    int palette = -1;

    uint32_t frames = 0;
    uint32_t errors = 0;
    uint64_t render_total = 0;
    uint32_t render_max = 0;
    uint64_t bytes_total = 0;
    uint32_t bytes_max = 0;
    uint64_t compute_total = 0;
    int64_t last_log = esp_timer_get_time();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        bool got = false;
        portENTER_CRITICAL(&s_frame_lock);
        if (s_fresh) {
            int tmp = s_read;
            s_read = s_ready;
            s_ready = tmp;
            s_fresh = false;
            got = true;
        }
        portEXIT_CRITICAL(&s_frame_lock);
        if (!got) {
            continue;
        }

        const fluid_frame_t *fr = &s_frames[s_read];
        if (fr->style != style || fr->palette != palette) {
            style = fr->style;
            palette = fr->palette;
            fluid_render_configure(&s_render, style, palette, raster_for(style));
        }

        int64_t t0 = esp_timer_get_time();
        fluid_render_prepare(&s_render, fr->level, fr->foam);
        uint32_t bytes = 0;
        uint32_t compute_us = 0;
        bool failed = false;
        for (int y = 0; y < FLUID_H; y += FLUID_BAND_LINES) {
            int lines = FLUID_H - y < FLUID_BAND_LINES ? FLUID_H - y : FLUID_BAND_LINES;
            int x0;
            int x1;
            if (!fluid_render_band_dirty(&s_render, y, lines, &x0, &x1)) {
                continue;
            }
            // Filling one buffer while the other is on the bus is safe: draw_bitmap
            // drains the previous colour transfer before sending the next window.
            uint16_t *buf = s_dma[s_dma_next];
            s_dma_next ^= 1;
            int64_t c0 = esp_timer_get_time();
            fluid_render_band(&s_render, y, lines, x0, x1, buf);
            compute_us += (uint32_t)(esp_timer_get_time() - c0);
            if (esp_lcd_panel_draw_bitmap(s_panel, x0, y, x1 + 1, y + lines, buf) != ESP_OK) {
                failed = true;
                break;
            }
            bytes += (uint32_t)((x1 - x0 + 1) * lines * 2);
        }
        if (failed) {
            // Keep `prev` as is and resend everything next frame.
            errors++;
            s_render.full_redraw = true;
        } else {
            fluid_render_commit(&s_render);
        }
        uint32_t us = (uint32_t)(esp_timer_get_time() - t0);

        frames++;
        render_total += us;
        render_max = us > render_max ? us : render_max;
        bytes_total += bytes;
        bytes_max = bytes > bytes_max ? bytes : bytes_max;
        compute_total += compute_us;
        int64_t now = esp_timer_get_time();
        if (now - last_log >= FLUID_STATS_US) {
            ESP_LOGI(TAG,
                     "render: fps=%" PRIu32 " style=%s frame_avg=%" PRIu32 "us max=%" PRIu32 "us compute_avg=%" PRIu32
                     "us KB_avg=%" PRIu32 " KB_max=%" PRIu32 " draw_err=%" PRIu32,
                     (uint32_t)((uint64_t)frames * 1000000 / (uint64_t)(now - last_log)), fluid_style_name(style),
                     (uint32_t)(render_total / frames), render_max, (uint32_t)(compute_total / frames),
                     (uint32_t)(bytes_total / frames / 1024), bytes_max / 1024, errors);
            frames = errors = render_max = bytes_max = 0;
            render_total = bytes_total = compute_total = 0;
            last_log = now;
        }
    }
}

esp_err_t fluid_app_start(void)
{
    esp_err_t err = init_display();
    if (err != ESP_OK) {
        return err;
    }

    // The FT3168 NACKs reads while idle; poll_touch() backs off and counts those, so
    // the driver's per-read error logs are only noise (and console time).
    esp_log_level_set("FT5x06", ESP_LOG_NONE);
    esp_log_level_set("lcd_panel.io.i2c", ESP_LOG_NONE);
    err = bsp_touch_new(NULL, &s_touch);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Touch unavailable: %s", esp_err_to_name(err));
        s_touch = NULL;
    }
    err = watch_boot_button_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BOOT button unavailable: %s", esp_err_to_name(err));
    }

    err = init_engine();
    if (err != ESP_OK) {
        return err;
    }

    if (xTaskCreatePinnedToCore(render_task, "fluid_render", FLUID_RENDER_STACK, NULL, FLUID_RENDER_PRIORITY,
                                &s_render_task, FLUID_RENDER_CORE) != pdPASS ||
        xTaskCreatePinnedToCore(helper_task, "fluid_helper", FLUID_HELPER_STACK, NULL, FLUID_HELPER_PRIORITY,
                                &s_helper_task, FLUID_RENDER_CORE) != pdPASS ||
        xTaskCreatePinnedToCore(sim_task, "fluid_sim", FLUID_SIM_STACK, NULL, FLUID_TASK_PRIORITY, &s_sim_task,
                                FLUID_SIM_CORE) != pdPASS) {
        ESP_LOGE(TAG, "Task creation failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
