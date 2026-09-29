#pragma once

// Turns particles into a coarse "LED" grid: per-cell fill level and foam (speed).
// Runs in the simulation task; the output arrays are what the renderer draws.

#include <stdbool.h>
#include <stdint.h>

#include "fluid_sim.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int pitch;         // px per LED cell
    int cols;
    int rows;
    int off_x;         // screen px of the grid's top-left corner (grid is centred)
    int off_y;
    float full_at;     // density (relative to rest) that maps to a fully lit cell
    float foam_speed;  // px/s that maps to maximum foam
    uint8_t smoothing; // 0..255 weight of the previous level (temporal filter)
    bool blur;         // 1-2-1 spatial blur before normalizing (for small pitches)

    // Internal state (fixed point: splat weights are 0..64 per particle).
    uint16_t *acc_density;
    uint16_t *acc_speed;
    uint16_t *row_tmp;
    uint8_t *state;
    uint8_t *wall;          // 1 = cell centre inside a wall (fluid_set_walls)
    uint32_t walls_version; // of the fluid when `wall` was built
} fluid_raster_t;

// Allocates buffers for a pitch; returns false on allocation failure.
bool fluid_raster_init(fluid_raster_t *raster, int screen_w, int screen_h, int pitch, fluid_alloc_fn alloc);

// Forget the temporal state (call after switching styles or resetting the fluid).
void fluid_raster_clear(fluid_raster_t *raster);

// level: 0..255 fill per cell; foam: 0..3, or FLUID_RASTER_WALL (level 0) on a wall.
// Both cols * rows, row-major.
#define FLUID_RASTER_WALL 4
void fluid_raster_run(fluid_raster_t *raster, const fluid_t *fluid, uint8_t *level, uint8_t *foam);

#ifdef __cplusplus
}
#endif
