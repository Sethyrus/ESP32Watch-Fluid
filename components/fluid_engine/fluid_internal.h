#pragma once

// Shared between fluid_sim.c and fluid_raster.c; not part of the public API.

#include <stdint.h>
#include <string.h>

#include "fluid_sim.h"

// On the ESP32-S3 float division and sqrtf are software calls (__divsf3, sqrtf; not
// even -ffast-math inlines them), about 100+ cycles each. Hot loops use these
// bit-trick approximations refined with Newton steps instead. Positive, normal
// inputs only.

// 1/sqrt(x), two Newton steps: relative error ~5e-6.
static inline float fluid_rsqrt(float x)
{
    uint32_t i;
    memcpy(&i, &x, sizeof(i));
    i = 0x5f375a86u - (i >> 1);
    float y;
    memcpy(&y, &i, sizeof(y));
    float hx = 0.5f * x;
    y = y * (1.5f - hx * y * y);
    y = y * (1.5f - hx * y * y);
    return y;
}

// 1/x, three Newton steps: relative error ~1e-7 (float precision).
static inline float fluid_recip(float x)
{
    uint32_t i;
    memcpy(&i, &x, sizeof(i));
    i = 0x7ef311c3u - i;
    float y;
    memcpy(&y, &i, sizeof(y));
    y = y * (2.0f - x * y);
    y = y * (2.0f - x * y);
    y = y * (2.0f - x * y);
    return y;
}

enum {
    FLUID_CELL_FLUID = 0,
    FLUID_CELL_AIR = 1,
    FLUID_CELL_SOLID = 2,
};

struct fluid {
    fluid_config_t cfg;

    // Pressure grid, column-major (index = i * ny + j). Local coordinates put the
    // screen origin at (h, h), so the outer ring of cells is always solid.
    int nx;
    int ny;
    int num_cells;
    float h;
    float inv_h;
    float *u;
    float *v;
    float *du;
    float *dv;
    float *prev_u;
    float *prev_v;
    float *s;        // 1 = open, 0 = solid (container mask + obstacle)
    float *s_static; // container mask plus walls
    float *density;  // particle density per cell
    uint8_t *cell_type;
    uint16_t *fluid_list;     // interior FLUID cells, rebuilt every substep
    float *inv_s_sum;         // 1 / open neighbour count, per fluid_list entry
    int num_fluid;
    float rest_density;

    // Particles, local coordinates, interleaved x,y.
    int num_particles;
    int capacity; // particles allocated (fill target)
    float radius;
    float *pos;
    float *vel;

    // Spatial hash for push-apart.
    int pnx;
    int pny;
    float p_inv_spacing;
    int32_t *cell_count;  // pnx * pny + 1 prefix sums
    uint16_t *cell_ids;

    // Obstacle (local coordinates).
    bool obstacle_active;
    float obstacle_x;
    float obstacle_y;
    float obstacle_r;
    float obstacle_vx;
    float obstacle_vy;

    // Static walls (fluid_set_walls), screen coordinates: signed distance sampled every
    // wall_step px, in 1/4 px, clamped to int8. Only the band near a wall matters.
    int8_t *wall;
    int wnx;
    int wny;
    float wall_step;
    float inv_wall_step;
    bool walls_active;
    uint32_t walls_version;
    int wall_grow_steps; // steps left with a limited push (after the walls changed)

    uint32_t rng; // fluid_splash

    fluid_stats_t stats;
};
