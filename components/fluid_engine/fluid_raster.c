#include "fluid_raster.h"

#include <stdlib.h>
#include <string.h>

#define SPLAT_ONE 64 // total splat weight of one particle
#define SPEED_MAX 63

static void *zalloc(fluid_alloc_fn alloc, size_t bytes)
{
    void *p = alloc ? alloc(bytes) : malloc(bytes);
    if (p != NULL) {
        memset(p, 0, bytes);
    }
    return p;
}

bool fluid_raster_init(fluid_raster_t *r, int screen_w, int screen_h, int pitch, fluid_alloc_fn alloc)
{
    memset(r, 0, sizeof(*r));
    r->pitch = pitch;
    r->cols = screen_w / pitch;
    r->rows = screen_h / pitch;
    r->off_x = (screen_w - r->cols * pitch) / 2;
    r->off_y = (screen_h - r->rows * pitch) / 2;
    r->full_at = 0.8f;
    r->foam_speed = 1500.0f;
    r->smoothing = 96;
    r->blur = false;

    size_t cells = (size_t)r->cols * (size_t)r->rows;
    r->acc_density = zalloc(alloc, cells * sizeof(uint16_t));
    r->acc_speed = zalloc(alloc, cells * sizeof(uint16_t));
    r->row_tmp = zalloc(alloc, (size_t)r->cols * sizeof(uint16_t));
    r->state = zalloc(alloc, cells);
    return r->acc_density && r->acc_speed && r->row_tmp && r->state;
}

void fluid_raster_clear(fluid_raster_t *r)
{
    memset(r->state, 0, (size_t)r->cols * (size_t)r->rows);
}

static inline void add(fluid_raster_t *r, int c, int row, int w, int speed_q)
{
    if (w == 0 || c < 0 || row < 0 || c >= r->cols || row >= r->rows) {
        return;
    }
    int i = row * r->cols + c;
    r->acc_density[i] = (uint16_t)(r->acc_density[i] + w);
    r->acc_speed[i] = (uint16_t)(r->acc_speed[i] + w * speed_q);
}

// Separable 1-2-1 blur, in place.
static void blur(fluid_raster_t *r, uint16_t *a)
{
    int cols = r->cols;
    int rows = r->rows;
    for (int y = 0; y < rows; y++) {
        uint16_t *row = a + y * cols;
        uint16_t left = row[0];
        for (int x = 0; x < cols; x++) {
            uint16_t c = row[x];
            uint16_t right = x + 1 < cols ? row[x + 1] : c;
            row[x] = (uint16_t)((left + 2 * c + right + 2) >> 2);
            left = c;
        }
    }
    uint16_t *prev = r->row_tmp;
    memcpy(prev, a, (size_t)cols * sizeof(uint16_t));
    for (int y = 0; y < rows; y++) {
        uint16_t *row = a + y * cols;
        const uint16_t *next = y + 1 < rows ? row + cols : row;
        for (int x = 0; x < cols; x++) {
            uint16_t c = row[x];
            uint16_t v = (uint16_t)((prev[x] + 2 * c + next[x] + 2) >> 2);
            prev[x] = c;
            row[x] = v;
        }
    }
}

void fluid_raster_run(fluid_raster_t *r, const fluid_t *f, uint8_t *level, uint8_t *foam)
{
    size_t cells = (size_t)r->cols * (size_t)r->rows;
    memset(r->acc_density, 0, cells * sizeof(uint16_t));
    memset(r->acc_speed, 0, cells * sizeof(uint16_t));

    int np = fluid_particle_count(f);
    const float *pos = fluid_particle_positions(f);
    const float *vel = fluid_particle_velocities(f);
    float inv_p = 1.0f / (float)r->pitch;
    float base_x = fluid_screen_offset(f) + (float)r->off_x;
    float base_y = fluid_screen_offset(f) + (float)r->off_y;
    float speed_k = (float)SPEED_MAX / (r->foam_speed * r->foam_speed);

    // Bilinear splat onto cell centres (mass preserving).
    for (int i = 0; i < np; i++) {
        float fx = (pos[2 * i] - base_x) * inv_p - 0.5f;
        float fy = (pos[2 * i + 1] - base_y) * inv_p - 0.5f;
        int c0 = (int)(fx + 1.0f) - 1; // floor for fx > -1
        int r0 = (int)(fy + 1.0f) - 1;
        int wx1 = (int)((fx - (float)c0) * (float)SPLAT_ONE / 8.0f + 0.5f); // 0..8
        int wy1 = (int)((fy - (float)r0) * (float)SPLAT_ONE / 8.0f + 0.5f);
        int wx0 = 8 - wx1;
        int wy0 = 8 - wy1;
        float v2 = vel[2 * i] * vel[2 * i] + vel[2 * i + 1] * vel[2 * i + 1];
        int sq = v2 * speed_k >= (float)SPEED_MAX ? SPEED_MAX : (int)(v2 * speed_k);
        add(r, c0, r0, wx0 * wy0, sq);
        add(r, c0 + 1, r0, wx1 * wy0, sq);
        add(r, c0, r0 + 1, wx0 * wy1, sq);
        add(r, c0 + 1, r0 + 1, wx1 * wy1, sq);
    }

    // Foam is the mean speed bucket (0..3) of the particles in the cell, taken before
    // the blur so both accumulators still describe the same particles.
    for (size_t i = 0; i < cells; i++) {
        uint16_t d = r->acc_density[i];
        foam[i] = d > 0 ? (uint8_t)(((uint32_t)r->acc_speed[i] / d) >> 4) : 0;
    }

    if (r->blur) {
        blur(r, r->acc_density);
    }

    float expected = fluid_rest_particles_per_px2(f) * (float)(r->pitch * r->pitch) * (float)SPLAT_ONE;
    float k = 255.0f / (expected * r->full_at);
    int keep = r->smoothing;
    int take = 256 - keep;
    for (size_t i = 0; i < cells; i++) {
        uint16_t d = r->acc_density[i];
        float lf = (float)d * k;
        int l = lf >= 255.0f ? 255 : (int)lf;
        int st = (r->state[i] * keep + l * take + 128) >> 8;
        r->state[i] = (uint8_t)st;
        level[i] = (uint8_t)st;
    }
}
