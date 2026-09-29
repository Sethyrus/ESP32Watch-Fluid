#pragma once

// Shared between fluid_sim.c and fluid_raster.c; not part of the public API.

#include "fluid_sim.h"

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
    float *s_static; // container mask only
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

    fluid_stats_t stats;
};
