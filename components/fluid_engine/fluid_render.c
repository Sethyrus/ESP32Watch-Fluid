#include "fluid_render.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    float r, g, b;
} rgb_t;

typedef struct {
    const char *name;
    rgb_t deep;
    rgb_t bright;
    rgb_t foam;
    rgb_t off; // unlit LED
} palette_t;

static const palette_t s_palettes[FLUID_PALETTE_COUNT] = {
    {"agua", {8, 60, 175}, {60, 170, 255}, {225, 248, 255}, {14, 16, 22}},
    {"lava", {140, 18, 0}, {255, 105, 10}, {255, 232, 120}, {22, 14, 12}},
    {"toxic", {0, 100, 45}, {70, 255, 120}, {225, 255, 210}, {12, 20, 14}},
};

static const struct {
    const char *name;
    int pitch;
} s_styles[FLUID_STYLE_COUNT] = {
    [FLUID_STYLE_LED] = {"led", 8},
    [FLUID_STYLE_PIXEL] = {"pixel", 5},
    [FLUID_STYLE_LIQUID] = {"liquid", 5},
};

int fluid_style_pitch(fluid_style_t style)
{
    return s_styles[style].pitch;
}

const char *fluid_style_name(fluid_style_t style)
{
    return s_styles[style].name;
}

const char *fluid_palette_name(int palette)
{
    return s_palettes[palette].name;
}

static void *zalloc(fluid_alloc_fn alloc, size_t bytes)
{
    void *p = alloc ? alloc(bytes) : malloc(bytes);
    if (p != NULL) {
        memset(p, 0, bytes);
    }
    return p;
}

static inline float clamp01(float x)
{
    return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

static inline float smoothstep(float a, float b, float x)
{
    float t = clamp01((x - a) / (b - a));
    return t * t * (3.0f - 2.0f * t);
}

static inline rgb_t mix(rgb_t a, rgb_t b, float t)
{
    rgb_t c = {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t};
    return c;
}

static inline rgb_t scale(rgb_t a, float k)
{
    rgb_t c = {a.r * k, a.g * k, a.b * k};
    return c;
}

static uint16_t to565(const fluid_render_t *r, rgb_t c)
{
    int rr = (int)(clamp01(c.r / 255.0f) * 31.0f + 0.5f);
    int gg = (int)(clamp01(c.g / 255.0f) * 63.0f + 0.5f);
    int bb = (int)(clamp01(c.b / 255.0f) * 31.0f + 0.5f);
    uint16_t v = (uint16_t)((rr << 11) | (gg << 5) | bb);
    return r->swap_bytes ? (uint16_t)((v >> 8) | (v << 8)) : v;
}

// Colour of a lit cell: brightness follows the fill level, foam pulls towards white.
static rgb_t fill_color(const palette_t *pal, float t, int foam)
{
    rgb_t c = scale(mix(pal->deep, pal->bright, t), 0.25f + 0.75f * t);
    return mix(c, pal->foam, (float)foam / 3.0f * 0.65f * t);
}

bool fluid_render_init(fluid_render_t *r, int screen_w, int screen_h, int max_cells, int max_pitch,
                       bool swap_bytes, fluid_alloc_fn alloc)
{
    memset(r, 0, sizeof(*r));
    r->screen_w = screen_w;
    r->screen_h = screen_h;
    r->swap_bytes = swap_bytes;
    r->max_cells = max_cells;
    r->max_pitch = max_pitch;
    r->cur = zalloc(alloc, (size_t)max_cells);
    r->prev = zalloc(alloc, (size_t)max_cells);
    r->shown = zalloc(alloc, (size_t)max_cells);
    r->hysteresis = 10;
    r->tiles = zalloc(alloc, (size_t)64 * max_pitch * max_pitch * sizeof(uint16_t));
    r->lut = zalloc(alloc, (size_t)4 * 256 * sizeof(uint16_t));
    r->col_of_x = zalloc(alloc, (size_t)screen_w * sizeof(int16_t));
    r->sub_x = zalloc(alloc, (size_t)screen_w * sizeof(uint16_t));
    r->row_of_y = zalloc(alloc, (size_t)screen_h * sizeof(int16_t));
    r->sub_y = zalloc(alloc, (size_t)screen_h * sizeof(uint16_t));
    r->full_redraw = true;
    return r->cur && r->prev && r->shown && r->tiles && r->lut && r->col_of_x && r->sub_x && r->row_of_y && r->sub_y;
}

static void build_tiles(fluid_render_t *r)
{
    const palette_t *pal = &s_palettes[r->palette];
    int p = r->pitch;
    float centre = 0.5f * (float)(p - 1);
    float dot_r = 0.40f * (float)p;
    float glow_r = 0.72f * (float)p;

    for (int code = 0; code < 64; code++) {
        int q = code & 15;
        int foam = code >> 4;
        float t = (float)q / 15.0f;
        rgb_t lit = fill_color(pal, t, foam);
        uint16_t *tile = r->tiles + code * p * p;
        for (int ty = 0; ty < p; ty++) {
            for (int tx = 0; tx < p; tx++) {
                rgb_t c;
                if (r->style == FLUID_STYLE_LED) {
                    float dx = (float)tx - centre;
                    float dy = (float)ty - centre;
                    float d = sqrtf(dx * dx + dy * dy);
                    float a = clamp01(dot_r + 0.5f - d);
                    float glow = 0.22f * clamp01(1.0f - d / glow_r) * t;
                    rgb_t dot = q == 0 ? pal->off : lit;
                    c = mix(scale(lit, glow), dot, a);
                } else {
                    c = q == 0 ? (rgb_t){0, 0, 0} : lit;
                }
                tile[ty * p + tx] = to565(r, c);
            }
        }
    }
}

static void build_liquid_lut(fluid_render_t *r)
{
    const palette_t *pal = &s_palettes[r->palette];
    const float thr = 0.34f;
    for (int foam = 0; foam < 4; foam++) {
        for (int v = 0; v < 256; v++) {
            float x = (float)v / 255.0f;
            float edge = smoothstep(thr - 0.07f, thr + 0.07f, x);
            float depth = clamp01((x - thr) / (1.0f - thr));
            rgb_t rim = mix(pal->bright, pal->foam, 0.25f);
            rgb_t c = mix(rim, pal->deep, powf(depth, 0.6f));
            c = mix(c, pal->foam, (float)foam / 3.0f * 0.6f);
            r->lut[foam * 256 + v] = to565(r, scale(c, edge));
        }
    }
}

static inline int floor_div(int a, int b)
{
    return a >= 0 ? a / b : -((-a + b - 1) / b);
}

void fluid_render_configure(fluid_render_t *r, fluid_style_t style, int palette, const fluid_raster_t *g)
{
    r->style = style;
    r->palette = palette;
    r->pitch = g->pitch;
    r->cols = g->cols;
    r->rows = g->rows;
    r->off_x = g->off_x;
    r->off_y = g->off_y;
    r->margin = style == FLUID_STYLE_LIQUID ? 1 : 0;
    r->full_redraw = true;
    int p = r->pitch;

    if (style == FLUID_STYLE_LIQUID) {
        // Bilinear between cell centres, clamped at the grid edges.
        for (int x = 0; x < r->screen_w; x++) {
            float fx = (float)(x - r->off_x) / (float)p - 0.5f + 0.5f / (float)p;
            int c0 = (int)floorf(fx);
            int w = (int)((fx - (float)c0) * 256.0f + 0.5f);
            if (c0 < 0) {
                c0 = 0;
                w = 0;
            } else if (c0 >= r->cols - 1) {
                c0 = r->cols - 2;
                w = 256;
            }
            r->col_of_x[x] = (int16_t)c0;
            r->sub_x[x] = (uint16_t)w;
        }
        for (int y = 0; y < r->screen_h; y++) {
            float fy = (float)(y - r->off_y) / (float)p - 0.5f + 0.5f / (float)p;
            int r0 = (int)floorf(fy);
            int w = (int)((fy - (float)r0) * 256.0f + 0.5f);
            if (r0 < 0) {
                r0 = 0;
                w = 0;
            } else if (r0 >= r->rows - 1) {
                r0 = r->rows - 2;
                w = 256;
            }
            r->row_of_y[y] = (int16_t)r0;
            r->sub_y[y] = (uint16_t)w;
        }
        build_liquid_lut(r);
    } else {
        for (int x = 0; x < r->screen_w; x++) {
            int c = floor_div(x - r->off_x, p);
            bool in = c >= 0 && c < r->cols;
            r->col_of_x[x] = (int16_t)(in ? c : -1);
            r->sub_x[x] = (uint16_t)(in ? (x - r->off_x) - c * p : 0);
        }
        for (int y = 0; y < r->screen_h; y++) {
            int row = floor_div(y - r->off_y, p);
            bool in = row >= 0 && row < r->rows;
            r->row_of_y[y] = (int16_t)(in ? row : -1);
            r->sub_y[y] = (uint16_t)(in ? (y - r->off_y) - row * p : 0);
        }
        build_tiles(r);
    }
}

void fluid_render_prepare(fluid_render_t *r, const uint8_t *level, const uint8_t *foam)
{
    int n = r->cols * r->rows;
    int hyst = r->full_redraw ? 0 : r->hysteresis;
    // Small level jitter at rest would otherwise flip codes every frame and keep the
    // bus busy; a cell only follows the new level once it moved more than `hyst`.
    for (int i = 0; i < n; i++) {
        int d = (int)level[i] - (int)r->shown[i];
        if (d > hyst || d < -hyst || level[i] == 0 || level[i] == 255) {
            r->shown[i] = level[i];
        }
    }
    if (r->style == FLUID_STYLE_LIQUID) {
        for (int i = 0; i < n; i++) {
            r->cur[i] = (uint8_t)((r->shown[i] >> 2) | (foam[i] << 6));
        }
    } else {
        for (int i = 0; i < n; i++) {
            r->cur[i] = (uint8_t)((r->shown[i] >> 4) | (foam[i] << 4));
        }
    }
}

bool fluid_render_band_dirty(const fluid_render_t *r, int y0, int lines, int *x0, int *x1)
{
    if (r->full_redraw) {
        *x0 = 0;
        *x1 = r->screen_w - 1;
        return true;
    }
    int p = r->pitch;
    int m = r->margin;
    int ra = floor_div(y0 - r->off_y, p) - m;
    int rb = floor_div(y0 + lines - 1 - r->off_y, p) + m;
    if (ra < 0) ra = 0;
    if (rb > r->rows - 1) rb = r->rows - 1;
    if (ra > rb) {
        return false;
    }

    int cmin = r->cols;
    int cmax = -1;
    for (int row = ra; row <= rb; row++) {
        const uint8_t *a = r->cur + row * r->cols;
        const uint8_t *b = r->prev + row * r->cols;
        if (memcmp(a, b, (size_t)r->cols) == 0) {
            continue;
        }
        for (int c = 0; c < r->cols; c++) {
            if (a[c] != b[c]) {
                if (c < cmin) cmin = c;
                break;
            }
        }
        for (int c = r->cols - 1; c >= 0; c--) {
            if (a[c] != b[c]) {
                if (c > cmax) cmax = c;
                break;
            }
        }
    }
    if (cmax < 0) {
        return false;
    }

    int px0 = r->off_x + (cmin - m) * p;
    int px1 = r->off_x + (cmax + 1 + m) * p - 1;
    if (px0 < 0) px0 = 0;
    if (px1 > r->screen_w - 1) px1 = r->screen_w - 1;
    px0 &= ~1;
    px1 |= 1;
    if (px1 > r->screen_w - 1) px1 = r->screen_w - 1;
    *x0 = px0;
    *x1 = px1;
    return true;
}

static void render_tiles(const fluid_render_t *r, int y0, int lines, int x0, int x1, uint16_t *dst)
{
    int p = r->pitch;
    int pp = p * p;
    int w = x1 - x0 + 1;
    for (int line = 0; line < lines; line++) {
        int y = y0 + line;
        uint16_t *out = dst + line * w;
        int row = r->row_of_y[y];
        if (row < 0) {
            memset(out, 0, (size_t)w * sizeof(uint16_t));
            continue;
        }
        const uint8_t *codes = r->cur + row * r->cols;
        const uint16_t *tiles = r->tiles + r->sub_y[y] * p;
        for (int x = x0; x <= x1; x++) {
            int c = r->col_of_x[x];
            *out++ = c < 0 ? 0 : tiles[codes[c] * pp + r->sub_x[x]];
        }
    }
}

static void render_liquid(const fluid_render_t *r, int y0, int lines, int x0, int x1, uint16_t *dst)
{
    int w = x1 - x0 + 1;
    int cols = r->cols;
    for (int line = 0; line < lines; line++) {
        int y = y0 + line;
        uint16_t *out = dst + line * w;
        const uint8_t *top = r->cur + r->row_of_y[y] * cols;
        const uint8_t *bot = top + cols;
        uint32_t wy = r->sub_y[y];
        uint32_t iy = 256 - wy;
        const uint8_t *foam_row = wy >= 128 ? bot : top;
        for (int x = x0; x <= x1; x++) {
            int c = r->col_of_x[x];
            uint32_t wx = r->sub_x[x];
            uint32_t ix = 256 - wx;
            uint32_t a = (top[c] & 63u) << 2;
            uint32_t b = (top[c + 1] & 63u) << 2;
            uint32_t cc = (bot[c] & 63u) << 2;
            uint32_t d = (bot[c + 1] & 63u) << 2;
            uint32_t t = a * ix + b * wx;
            uint32_t u = cc * ix + d * wx;
            uint32_t v = (t * iy + u * wy) >> 16;
            uint32_t foam = foam_row[wx >= 128 ? c + 1 : c] >> 6;
            *out++ = r->lut[(foam << 8) | v];
        }
    }
}

void fluid_render_band(const fluid_render_t *r, int y0, int lines, int x0, int x1, uint16_t *dst)
{
    if (r->style == FLUID_STYLE_LIQUID) {
        render_liquid(r, y0, lines, x0, x1, dst);
    } else {
        render_tiles(r, y0, lines, x0, x1, dst);
    }
}

void fluid_render_commit(fluid_render_t *r)
{
    uint8_t *t = r->prev;
    r->prev = r->cur;
    r->cur = t;
    r->full_redraw = false;
}
