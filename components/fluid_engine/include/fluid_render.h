#pragma once

// Renders the raster grid into RGB565 bands and tracks which bands changed, so the
// display only receives what moved. Pure C; the ESP side just ships the bands.

#include <stdbool.h>
#include <stdint.h>

#include "fluid_raster.h"
#include "fluid_sim.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FLUID_STYLE_LED = 0, // round dots with a gap, like an LED matrix
    FLUID_STYLE_PIXEL,   // small solid squares
    FLUID_STYLE_LIQUID,  // smooth surface, bilinear field with an antialiased edge
    FLUID_STYLE_COUNT,
} fluid_style_t;

#define FLUID_PALETTE_COUNT 3

typedef struct {
    int screen_w;
    int screen_h;
    bool swap_bytes; // true for the panel (big-endian RGB565)

    fluid_style_t style;
    int palette;
    int pitch;
    int cols;
    int rows;
    int off_x;
    int off_y;
    int margin;       // grid cells a pixel depends on beyond its own (liquid: 1)
    bool full_redraw; // next frame sends every band
    uint8_t hysteresis; // level change (0..255) needed before a cell is redrawn

    // Internal buffers.
    uint8_t *cur;   // visual codes, cols * rows
    uint8_t *prev;  // codes currently on screen
    uint8_t *shown; // level behind the code on screen, for hysteresis
    uint16_t *tiles; // tile styles: 64 codes * pitch * pitch
    uint16_t *lut;   // liquid: 4 foam * 256 values
    int16_t *col_of_x;
    uint16_t *sub_x; // tile: offset in tile; liquid: weight 0..256
    int16_t *row_of_y;
    uint16_t *sub_y;
    int max_cells;
    int max_pitch;
} fluid_render_t;

int fluid_style_pitch(fluid_style_t style);
const char *fluid_style_name(fluid_style_t style);
const char *fluid_palette_name(int palette);

// max_cells / max_pitch: the largest grid and pitch any style will use.
bool fluid_render_init(fluid_render_t *r, int screen_w, int screen_h, int max_cells, int max_pitch,
                       bool swap_bytes, fluid_alloc_fn alloc);

// Select style and palette for a raster grid; forces a full redraw.
void fluid_render_configure(fluid_render_t *r, fluid_style_t style, int palette, const fluid_raster_t *grid);

// Quantize the raster output into visual codes for this frame.
void fluid_render_prepare(fluid_render_t *r, const uint8_t *level, const uint8_t *foam);

// Whether band [y0, y0 + lines) changed; returns its x range, x0 even and x1 odd (inclusive),
// as the panel requires.
bool fluid_render_band_dirty(const fluid_render_t *r, int y0, int lines, int *x0, int *x1);

// Writes (x1 - x0 + 1) * lines pixels for the band.
void fluid_render_band(const fluid_render_t *r, int y0, int lines, int x0, int x1, uint16_t *dst);

// Mark this frame as displayed.
void fluid_render_commit(fluid_render_t *r);

#ifdef __cplusplus
}
#endif
