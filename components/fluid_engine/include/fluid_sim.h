#pragma once

// FLIP/PIC fluid simulation (after Matthias Mueller, Ten Minute Physics #18, MIT).
// Pure C, no ESP-IDF dependencies, so it also builds on the host (tools/host_bench).
// Coordinates are screen pixels: +x right, +y down. The container is a rounded
// rectangle covering the whole screen.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void *(*fluid_alloc_fn)(size_t bytes);
typedef int64_t (*fluid_clock_fn)(void);

typedef struct {
    float width;              // container size in px
    float height;
    float corner_radius;      // rounded corners in px (0 = square)
    float cell_size;          // pressure grid spacing h in px; dominates CPU cost
    float fill_fraction;      // share of the container filled at reset (0..1)
    int max_particles;        // cap; only the fill target is allocated
    int pressure_iters;       // Gauss-Seidel iterations per substep
    int separation_iters;     // particle push-apart iterations per substep
    float over_relaxation;    // pressure solver SOR factor (~1.9)
    float flip_ratio;         // 0 = PIC (calm), 1 = FLIP (lively); ~0.9
    float max_cells_per_step; // particle speed clamp, in cells per substep
    float damping;            // velocity decay in 1/s (0 = none), helps the fluid settle
    fluid_alloc_fn alloc;     // NULL = malloc; buffers are never freed
    fluid_clock_fn clock_us;  // optional, fills per-stage timings in fluid_stats_t
} fluid_config_t;

typedef struct {
    int particles;
    int cells_x;
    int cells_y;
    int fluid_cells;
    // Microseconds spent in the last fluid_step() (all substeps), 0 without clock_us.
    uint32_t us_integrate;
    uint32_t us_separate;
    uint32_t us_collide;
    uint32_t us_p2g;
    uint32_t us_density;
    uint32_t us_pressure;
    uint32_t us_g2p;
} fluid_stats_t;

typedef struct fluid fluid_t;

// Sensible defaults for the 410x502 watch screen.
fluid_config_t fluid_default_config(void);

// Allocates every buffer up front and fills the container. NULL on allocation failure.
fluid_t *fluid_create(const fluid_config_t *config);

// Refills the container at rest (hex packing from the bottom).
void fluid_reset(fluid_t *fluid);

// Circular obstacle (e.g. a finger). Velocity in px/s. Takes effect on the next step.
void fluid_set_obstacle(fluid_t *fluid, float x, float y, float radius, float vx, float vy, bool active);

// Advances dt seconds split in `substeps`. Gravity in px/s^2, screen axes.
void fluid_step(fluid_t *fluid, float gravity_x, float gravity_y, float dt, int substeps);

const fluid_stats_t *fluid_get_stats(const fluid_t *fluid);

// Read-only particle access, interleaved x,y, in simulation coordinates:
// screen = sim - fluid_screen_offset() on both axes.
int fluid_particle_count(const fluid_t *fluid);
const float *fluid_particle_positions(const fluid_t *fluid);
const float *fluid_particle_velocities(const fluid_t *fluid);
float fluid_particle_radius(const fluid_t *fluid);
float fluid_screen_offset(const fluid_t *fluid);

// Particles per px^2 of fluid at rest (hex packing), for density normalization.
float fluid_rest_particles_per_px2(const fluid_t *fluid);

// Signed distance to the container wall in px (negative inside), screen coordinates.
float fluid_container_sdf(const fluid_t *fluid, float x, float y);

#ifdef __cplusplus
}
#endif
