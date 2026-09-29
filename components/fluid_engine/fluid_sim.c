#include "fluid_internal.h"

#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

// Step order and transfer rules follow Mueller's FLIP reference (18-flip.html):
// integrate -> push apart -> collide -> P2G -> density -> pressure -> G2P.
// Differences: rounded-rect container (SDF), gravity as a 2D vector, fluid-cell
// list for the solver, velocity clamp and NaN guard for robustness.

static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline int clampi(int x, int lo, int hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static void *zalloc(fluid_t *f, size_t bytes)
{
    void *p = f->cfg.alloc ? f->cfg.alloc(bytes) : malloc(bytes);
    if (p != NULL) {
        memset(p, 0, bytes);
    }
    return p;
}

// Particles needed for the fill fraction of the rounded container, capped by config.
static int particle_target(const fluid_config_t *c, float radius)
{
    float dx = 2.0f * radius;
    float dy = 0.8660254f * dx;
    float area = c->width * c->height - (4.0f - 3.14159265f) * c->corner_radius * c->corner_radius;
    int target = (int)(c->fill_fraction * area / (dx * dy));
    return target > c->max_particles ? c->max_particles : target;
}

static inline uint32_t elapsed(const fluid_t *f, int64_t *t)
{
    if (f->cfg.clock_us == NULL) {
        return 0;
    }
    int64_t now = f->cfg.clock_us();
    uint32_t d = (uint32_t)(now - *t);
    *t = now;
    return d;
}

fluid_config_t fluid_default_config(void)
{
    fluid_config_t c = {
        .width = 410.0f,
        .height = 502.0f,
        .corner_radius = 114.0f,
        .cell_size = 10.0f,
        .fill_fraction = 0.40f,
        .max_particles = 4000,
        .pressure_iters = 20,
        .separation_iters = 1,
        .over_relaxation = 1.9f,
        .flip_ratio = 0.85f,
        .max_cells_per_step = 2.0f,
        .damping = 0.5f,
        .drift_k = 0.5f,
        .drift_slack = 0.4f,
        .alloc = NULL,
        .clock_us = NULL,
        .parallel = NULL,
    };
    return c;
}

// Signed distance to the rounded rectangle, screen coordinates. Optionally returns the
// outward normal of the closest wall.
static float container_sdf(const fluid_config_t *c, float x, float y, float *nx, float *ny)
{
    float hw = 0.5f * c->width;
    float hh = 0.5f * c->height;
    float r = c->corner_radius;
    float dx = x - hw;
    float dy = y - hh;
    float sx = dx < 0.0f ? -1.0f : 1.0f;
    float sy = dy < 0.0f ? -1.0f : 1.0f;
    float qx = fabsf(dx) - (hw - r);
    float qy = fabsf(dy) - (hh - r);

    if (qx > 0.0f && qy > 0.0f) {
        float q2 = qx * qx + qy * qy;
        float inv_len = fluid_rsqrt(q2); // corner region: qx, qy > 0
        float len = q2 * inv_len;
        if (nx != NULL) {
            *nx = sx * qx * inv_len;
            *ny = sy * qy * inv_len;
        }
        return len - r;
    }
    if (qx > qy) {
        if (nx != NULL) {
            *nx = sx;
            *ny = 0.0f;
        }
        return qx - r;
    }
    if (nx != NULL) {
        *nx = 0.0f;
        *ny = sy;
    }
    return qy - r;
}

float fluid_container_sdf(const fluid_t *f, float x, float y)
{
    return container_sdf(&f->cfg, x, y, NULL, NULL);
}

// Bilinear wall distance (screen px) and its gradient direction, from the int8 field.
static inline float wall_eval(const fluid_t *f, float x, float y, float *gx, float *gy)
{
    float fx = clampf(x * f->inv_wall_step, 0.0f, (float)(f->wnx - 1) - 0.001f);
    float fy = clampf(y * f->inv_wall_step, 0.0f, (float)(f->wny - 1) - 0.001f);
    int i = (int)fx;
    int j = (int)fy;
    float tx = fx - (float)i;
    float ty = fy - (float)j;
    const int8_t *w = f->wall + j * f->wnx + i;
    float v00 = (float)w[0];
    float v10 = (float)w[1];
    float v01 = (float)w[f->wnx];
    float v11 = (float)w[f->wnx + 1];
    if (gx != NULL) {
        *gx = (v10 - v00) * (1.0f - ty) + (v11 - v01) * ty;
        *gy = (v01 - v00) * (1.0f - tx) + (v11 - v10) * tx;
    }
    float top = v00 + (v10 - v00) * tx;
    float bot = v01 + (v11 - v01) * tx;
    return 0.25f * (top + (bot - top) * ty);
}

// Open cells: centre inside the rounded rect and outside every wall.
static void build_static_mask(fluid_t *f)
{
    for (int i = 0; i < f->nx; i++) {
        for (int j = 0; j < f->ny; j++) {
            float open = 0.0f;
            if (i > 0 && j > 0 && i < f->nx - 1 && j < f->ny - 1) {
                float cx = ((float)i + 0.5f) * f->h - f->h;
                float cy = ((float)j + 0.5f) * f->h - f->h;
                bool inside = container_sdf(&f->cfg, cx, cy, NULL, NULL) < 0.0f;
                if (inside && f->walls_active) {
                    inside = wall_eval(f, cx, cy, NULL, NULL) >= 0.0f;
                }
                open = inside ? 1.0f : 0.0f;
            }
            f->s_static[i * f->ny + j] = open;
            f->s[i * f->ny + j] = open;
        }
    }
}

fluid_t *fluid_create(const fluid_config_t *config)
{
    fluid_t tmp = {.cfg = *config};
    fluid_t *f = zalloc(&tmp, sizeof(*f));
    if (f == NULL) {
        return NULL;
    }
    f->cfg = *config;

    f->h = config->cell_size;
    f->inv_h = 1.0f / f->h;
    f->nx = (int)ceilf(config->width * f->inv_h) + 2;
    f->ny = (int)ceilf(config->height * f->inv_h) + 2;
    f->num_cells = f->nx * f->ny;
    f->radius = 0.3f * f->h;

    float p_spacing = 2.2f * f->radius;
    f->p_inv_spacing = 1.0f / p_spacing;
    f->pnx = (int)floorf((float)f->nx * f->h * f->p_inv_spacing) + 1;
    f->pny = (int)floorf((float)f->ny * f->h * f->p_inv_spacing) + 1;

    f->capacity = particle_target(config, f->radius);
    size_t cells_f = (size_t)f->num_cells * sizeof(float);
    size_t parts = (size_t)(f->capacity > 0 ? f->capacity : 1);
    f->u = zalloc(f, cells_f);
    f->v = zalloc(f, cells_f);
    f->du = zalloc(f, cells_f);
    f->dv = zalloc(f, cells_f);
    f->prev_u = zalloc(f, cells_f);
    f->prev_v = zalloc(f, cells_f);
    f->s = zalloc(f, cells_f);
    f->s_static = zalloc(f, cells_f);
    f->density = zalloc(f, cells_f);
    f->cell_type = zalloc(f, (size_t)f->num_cells);
    f->fluid_list = zalloc(f, (size_t)f->num_cells * sizeof(uint16_t));
    f->inv_s_sum = zalloc(f, cells_f);
    f->pos = zalloc(f, parts * 2 * sizeof(float));
    f->vel = zalloc(f, parts * 2 * sizeof(float));
    f->cell_count = zalloc(f, ((size_t)f->pnx * f->pny + 1) * sizeof(int32_t));
    f->cell_ids = zalloc(f, parts * sizeof(uint16_t));
    f->wall_step = 0.5f * f->h;
    f->inv_wall_step = 1.0f / f->wall_step;
    f->wnx = (int)ceilf(config->width * f->inv_wall_step) + 1;
    f->wny = (int)ceilf(config->height * f->inv_wall_step) + 1;
    f->wall = zalloc(f, (size_t)f->wnx * f->wny);

    if (!f->u || !f->v || !f->du || !f->dv || !f->prev_u || !f->prev_v || !f->s || !f->s_static ||
        !f->density || !f->cell_type || !f->fluid_list || !f->inv_s_sum || !f->pos || !f->vel ||
        !f->cell_count || !f->cell_ids || !f->wall || f->num_cells > 65535 || f->capacity > 65535) {
        return NULL; // buffers are not freed: this only happens at boot with a bad config
    }

    for (int i = 0; i < f->wnx * f->wny; i++) {
        f->wall[i] = INT8_MAX;
    }
    f->rng = 0x9e3779b9u;
    build_static_mask(f);

    fluid_reset(f);
    return f;
}

void fluid_reset(fluid_t *f)
{
    const fluid_config_t *c = &f->cfg;
    float r = f->radius;
    float dx = 2.0f * r;
    float dy = 0.8660254f * dx;
    int target = f->capacity;

    int n = 0;
    int row = 0;
    for (float y = c->height - r - 0.5f; y > r && n < target; y -= dy, row++) {
        float x0 = r + 0.5f + ((row & 1) ? r : 0.0f);
        for (float x = x0; x < c->width - r && n < target; x += dx) {
            if (container_sdf(c, x, y, NULL, NULL) > -r ||
                (f->walls_active && wall_eval(f, x, y, NULL, NULL) < r)) {
                continue;
            }
            f->pos[2 * n] = x + f->h;
            f->pos[2 * n + 1] = y + f->h;
            f->vel[2 * n] = 0.0f;
            f->vel[2 * n + 1] = 0.0f;
            n++;
        }
    }
    f->num_particles = n;
    f->rest_density = 0.0f;

    memset(f->u, 0, (size_t)f->num_cells * sizeof(float));
    memset(f->v, 0, (size_t)f->num_cells * sizeof(float));
    f->stats.particles = n;
    f->stats.cells_x = f->nx;
    f->stats.cells_y = f->ny;
}

void fluid_set_obstacle(fluid_t *f, float x, float y, float radius, float vx, float vy, bool active)
{
    f->obstacle_active = active;
    f->obstacle_x = x + f->h;
    f->obstacle_y = y + f->h;
    f->obstacle_r = radius;
    f->obstacle_vx = vx;
    f->obstacle_vy = vy;
}

void fluid_set_walls(fluid_t *f, fluid_sdf_fn sdf, void *ctx)
{
    f->walls_active = sdf != NULL;
    for (int j = 0; j < f->wny; j++) {
        for (int i = 0; i < f->wnx; i++) {
            float d = sdf != NULL ? 4.0f * sdf(ctx, (float)i * f->wall_step, (float)j * f->wall_step) : 127.0f;
            d = clampf(d, -127.0f, 127.0f);
            f->wall[j * f->wnx + i] = (int8_t)(d < 0.0f ? d - 0.5f : d + 0.5f);
        }
    }
    build_static_mask(f);
    f->walls_version++;
    f->wall_grow_steps = 20;
}

uint32_t fluid_walls_version(const fluid_t *f)
{
    return f->walls_version;
}

float fluid_wall_sdf(const fluid_t *f, float x, float y)
{
    return f->walls_active ? wall_eval(f, x, y, NULL, NULL) : 1e6f;
}

static inline float rand01(uint32_t *x)
{
    *x = *x * 1664525u + 1013904223u; // LCG
    return (float)(*x >> 8) * (1.0f / 16777216.0f);
}

void fluid_splash(fluid_t *f, float dir_x, float dir_y, float speed)
{
    // Random pure noise would mostly be projected away by the pressure solve; a
    // coherent throw along dir, uneven per particle, breaks the surface instead.
    uint32_t x = f->rng;
    for (int i = 0; i < f->num_particles; i++) {
        float k = speed * (0.4f + 0.6f * rand01(&x));
        f->vel[2 * i] += dir_x * k + 0.3f * speed * (rand01(&x) - 0.5f);
        f->vel[2 * i + 1] += dir_y * k + 0.3f * speed * (rand01(&x) - 0.5f);
    }
    f->rng = x;
}

static void apply_obstacle_to_grid(fluid_t *f)
{
    int n = f->ny;
    memcpy(f->s, f->s_static, (size_t)f->num_cells * sizeof(float));
    if (!f->obstacle_active) {
        return;
    }
    float r2 = f->obstacle_r * f->obstacle_r;
    int i0 = clampi((int)((f->obstacle_x - f->obstacle_r) * f->inv_h) - 1, 1, f->nx - 2);
    int i1 = clampi((int)((f->obstacle_x + f->obstacle_r) * f->inv_h) + 1, 1, f->nx - 2);
    int j0 = clampi((int)((f->obstacle_y - f->obstacle_r) * f->inv_h) - 1, 1, f->ny - 2);
    int j1 = clampi((int)((f->obstacle_y + f->obstacle_r) * f->inv_h) + 1, 1, f->ny - 2);
    for (int i = i0; i <= i1; i++) {
        for (int j = j0; j <= j1; j++) {
            if (f->s_static[i * n + j] == 0.0f) {
                continue;
            }
            float dx = ((float)i + 0.5f) * f->h - f->obstacle_x;
            float dy = ((float)j + 0.5f) * f->h - f->obstacle_y;
            if (dx * dx + dy * dy < r2) {
                f->s[i * n + j] = 0.0f;
                f->u[i * n + j] = f->obstacle_vx;
                f->u[(i + 1) * n + j] = f->obstacle_vx;
                f->v[i * n + j] = f->obstacle_vy;
                f->v[i * n + j + 1] = f->obstacle_vy;
            }
        }
    }
}

// Work split between the caller and the optional helper (fluid_config_t.parallel).
// Every split below touches disjoint memory in its two parts.
static void run2(fluid_t *f, fluid_job_fn fn, void *ctx)
{
    if (f->cfg.parallel != NULL) {
        f->cfg.parallel(fn, ctx);
    } else {
        fn(ctx, 0);
        fn(ctx, 1);
    }
}

static inline void particle_range(const fluid_t *f, int part, int *i0, int *i1)
{
    int mid = f->num_particles / 2;
    *i0 = part == 0 ? 0 : mid;
    *i1 = part == 0 ? mid : f->num_particles;
}

typedef struct {
    fluid_t *f;
    float dt;
    float gx;
    float gy;
} integrate_job_t;

static void integrate_job(void *ctx, int part)
{
    const integrate_job_t *job = ctx;
    fluid_t *f = job->f;
    float dt = job->dt;
    float vmax = f->cfg.max_cells_per_step * f->h / dt;
    float vmax2 = vmax * vmax;
    float keep = 1.0f / (1.0f + f->cfg.damping * dt);
    float gdx = dt * job->gx;
    float gdy = dt * job->gy;
    float *restrict pos = f->pos;
    float *restrict vel = f->vel;
    int i0;
    int i1;
    particle_range(f, part, &i0, &i1);
    for (int i = i0; i < i1; i++) {
        float vx = vel[2 * i] * keep + gdx;
        float vy = vel[2 * i + 1] * keep + gdy;
        float v2 = vx * vx + vy * vy;
        if (v2 > vmax2) {
            float k = vmax * fluid_rsqrt(v2);
            vx *= k;
            vy *= k;
        }
        vel[2 * i] = vx;
        vel[2 * i + 1] = vy;
        pos[2 * i] += vx * dt;
        pos[2 * i + 1] += vy * dt;
    }
}

// Push-apart for base hash columns [x_begin, x_end). Each pair is visited once: a cell
// against itself (j > i) and its 4 "forward" neighbours (+x-1y, +x, +x+1y, +y), so a
// base column x only touches particles in columns x and x + 1.
static void separate_columns(fluid_t *f, int x_begin, int x_end)
{
    int pnx = f->pnx;
    int pny = f->pny;
    float *restrict pos = f->pos;
    const int32_t *restrict first = f->cell_count;
    const uint16_t *restrict ids = f->cell_ids;
    float min_dist = 2.0f * f->radius;
    float min_dist2 = min_dist * min_dist;

    for (int xi = x_begin; xi < x_end; xi++) {
        for (int yi = 0; yi < pny; yi++) {
            int c = xi * pny + yi;
            int a_begin = first[c];
            int a_end = first[c + 1];
            if (a_begin == a_end) {
                continue;
            }
            // Particle ranges of the 4 forward neighbours, resolved once per cell.
            int nb_begin[4];
            int nb_end[4];
            int nbs = 0;
            if (xi + 1 < pnx) {
                int right = c + pny;
                for (int dy = -1; dy <= 1; dy++) {
                    if (yi + dy >= 0 && yi + dy < pny) {
                        nb_begin[nbs] = first[right + dy];
                        nb_end[nbs++] = first[right + dy + 1];
                    }
                }
            }
            if (yi + 1 < pny) {
                nb_begin[nbs] = first[c + 1];
                nb_end[nbs++] = first[c + 2];
            }
            for (int ka = a_begin; ka < a_end; ka++) {
                int a = ids[ka];
                float ax = pos[2 * a];
                float ay = pos[2 * a + 1];
                // Same cell (later particles only), then the forward neighbours.
                for (int nb = -1; nb < nbs; nb++) {
                    int kb = nb < 0 ? ka + 1 : nb_begin[nb];
                    int b_end = nb < 0 ? a_end : nb_end[nb];
                    for (; kb < b_end; kb++) {
                        int b = ids[kb];
                        float dx = pos[2 * b] - ax;
                        float dy = pos[2 * b + 1] - ay;
                        float d2 = dx * dx + dy * dy;
                        if (d2 > min_dist2 || d2 == 0.0f) {
                            continue;
                        }
                        // Push apart by 0.6 of the overlap, s = 0.6 * (min_dist - d) / d,
                        // without sqrtf or a division. Pairs are visited once (the
                        // reference, 0.5, visits twice); 0.6 is the committed tuning.
                        float s = 0.6f * (min_dist * fluid_rsqrt(d2) - 1.0f);
                        dx *= s;
                        dy *= s;
                        ax -= dx;
                        ay -= dy;
                        pos[2 * b] += dx;
                        pos[2 * b + 1] += dy;
                    }
                }
                pos[2 * a] = ax;
                pos[2 * a + 1] = ay;
            }
        }
    }
}

// Parts own base columns [0, mid - 1) and [mid, pnx): they touch columns [0, mid) and
// [mid, pnx) respectively, which never overlap. Column mid - 1 runs afterwards.
static void separate_job(void *ctx, int part)
{
    fluid_t *f = ctx;
    int mid = f->pnx / 2;
    if (part == 0) {
        separate_columns(f, 0, mid - 1);
    } else {
        separate_columns(f, mid, f->pnx);
    }
}

static void push_particles_apart(fluid_t *f, int iters)
{
    int np = f->num_particles;
    int pnx = f->pnx;
    int pny = f->pny;
    int pcells = pnx * pny;
    float inv = f->p_inv_spacing;
    float *restrict pos = f->pos;
    int32_t *restrict first = f->cell_count;
    uint16_t *restrict ids = f->cell_ids;

    // Counting sort of particles into hash cells; first[c]..first[c+1] is cell c.
    memset(first, 0, ((size_t)pcells + 1) * sizeof(int32_t));
    for (int i = 0; i < np; i++) {
        int xi = clampi((int)(pos[2 * i] * inv), 0, pnx - 1);
        int yi = clampi((int)(pos[2 * i + 1] * inv), 0, pny - 1);
        first[xi * pny + yi]++;
    }
    int acc = 0;
    for (int c = 0; c < pcells; c++) {
        acc += first[c];
        first[c] = acc;
    }
    first[pcells] = acc;
    for (int i = 0; i < np; i++) {
        int xi = clampi((int)(pos[2 * i] * inv), 0, pnx - 1);
        int yi = clampi((int)(pos[2 * i + 1] * inv), 0, pny - 1);
        int c = xi * pny + yi;
        first[c]--;
        ids[first[c]] = (uint16_t)i;
    }

    int mid = pnx / 2;
    for (int it = 0; it < iters; it++) {
        run2(f, separate_job, f);
        separate_columns(f, mid - 1, mid);
    }
}

static void collide_job(void *ctx, int part)
{
    fluid_t *f = ctx;
    float h = f->h;
    float r = f->radius;
    float *restrict pos = f->pos;
    float *restrict vel = f->vel;
    float ob_r = f->obstacle_r + r;
    float ob_r2 = ob_r * ob_r;
    float cx = 0.5f * f->cfg.width + h;
    float cy = 0.5f * f->cfg.height + h;
    // Right after fluid_set_walls the push is limited, so a new wall shoves the fluid
    // aside over a few steps instead of flinging it; afterwards it is a plain wall.
    float max_push = f->wall_grow_steps > 0 ? 0.5f * h : 1e9f;
    int i0;
    int i1;
    particle_range(f, part, &i0, &i1);

    for (int i = i0; i < i1; i++) {
        float x = pos[2 * i];
        float y = pos[2 * i + 1];
        float vx = vel[2 * i];
        float vy = vel[2 * i + 1];

        if (!isfinite(x) || !isfinite(y) || !isfinite(vx) || !isfinite(vy)) {
            x = cx;
            y = cy;
            vx = 0.0f;
            vy = 0.0f;
        }

        if (f->obstacle_active) {
            float dx = x - f->obstacle_x;
            float dy = y - f->obstacle_y;
            float d2 = dx * dx + dy * dy;
            if (d2 < ob_r2) {
                float d = sqrtf(d2);
                float nx = d > 1e-4f ? dx / d : 0.0f;
                float ny = d > 1e-4f ? dy / d : -1.0f;
                x = f->obstacle_x + nx * ob_r;
                y = f->obstacle_y + ny * ob_r;
                vx = f->obstacle_vx;
                vy = f->obstacle_vy;
            }
        }

        if (f->walls_active) {
            float gx;
            float gy;
            float wd = wall_eval(f, x - h, y - h, &gx, &gy);
            if (wd < r) {
                // Out along the gradient.
                float g2 = gx * gx + gy * gy;
                float inv = g2 > 1e-6f ? fluid_rsqrt(g2) : 0.0f;
                float wx = gx * inv;
                float wy = g2 > 1e-6f ? gy * inv : -1.0f;
                float pen = r - wd;
                pen = pen > max_push ? max_push : pen;
                x += wx * pen;
                y += wy * pen;
                float vn = vx * wx + vy * wy;
                if (vn < 0.0f) {
                    vx -= vn * wx;
                    vy -= vn * wy;
                }
            }
        }

        float nx;
        float ny;
        float d = container_sdf(&f->cfg, x - h, y - h, &nx, &ny);
        if (d > -r) {
            float pen = d + r;
            x -= nx * pen;
            y -= ny * pen;
            float vn = vx * nx + vy * ny;
            if (vn > 0.0f) {
                vx -= vn * nx;
                vy -= vn * ny;
            }
        }

        pos[2 * i] = x;
        pos[2 * i + 1] = y;
        vel[2 * i] = vx;
        vel[2 * i + 1] = vy;
    }
}

// Per-component particle-to-grid splat, normalization and solid faces. u and v (and
// their weight arrays) are separate, so the two components run in parallel.
static void p2g_job(void *ctx, int comp)
{
    fluid_t *f = ctx;
    int n = f->ny;
    int nx = f->nx;
    float h = f->h;
    float h1 = f->inv_h;
    float h2 = 0.5f * h;
    float dx = comp == 0 ? 0.0f : h2;
    float dy = comp == 0 ? h2 : 0.0f;
    float *restrict fg = comp == 0 ? f->u : f->v;
    float *restrict dg = comp == 0 ? f->du : f->dv;
    const float *restrict prev = comp == 0 ? f->prev_u : f->prev_v;
    const uint8_t *restrict ct = f->cell_type;
    const float *restrict pos = f->pos;
    const float *restrict vel = f->vel;
    size_t cells_f = (size_t)f->num_cells * sizeof(float);

    memset(fg, 0, cells_f);
    memset(dg, 0, cells_f);
    for (int i = 0; i < f->num_particles; i++) {
        float x = clampf(pos[2 * i], h, (float)(nx - 1) * h);
        float y = clampf(pos[2 * i + 1], h, (float)(n - 1) * h);
        int x0 = (int)((x - dx) * h1);
        if (x0 > nx - 2) x0 = nx - 2;
        float tx = ((x - dx) - (float)x0 * h) * h1;
        int x1 = x0 + 1 > nx - 2 ? nx - 2 : x0 + 1;
        int y0 = (int)((y - dy) * h1);
        if (y0 > n - 2) y0 = n - 2;
        float ty = ((y - dy) - (float)y0 * h) * h1;
        int y1 = y0 + 1 > n - 2 ? n - 2 : y0 + 1;
        float sx = 1.0f - tx;
        float sy = 1.0f - ty;
        float d0 = sx * sy;
        float d1 = tx * sy;
        float d2 = tx * ty;
        float d3 = sx * ty;
        int nr0 = x0 * n + y0;
        int nr1 = x1 * n + y0;
        int nr2 = x1 * n + y1;
        int nr3 = x0 * n + y1;
        float pv = vel[2 * i + comp];
        fg[nr0] += pv * d0; dg[nr0] += d0;
        fg[nr1] += pv * d1; dg[nr1] += d1;
        fg[nr2] += pv * d2; dg[nr2] += d2;
        fg[nr3] += pv * d3; dg[nr3] += d3;
    }
    for (int i = 0; i < f->num_cells; i++) {
        if (dg[i] > 0.0f) {
            fg[i] *= fluid_recip(dg[i]);
        }
    }

    // Faces touching solid cells: 0 against static walls (container, fluid_set_walls),
    // else the previous value, which is the obstacle velocity. Keeping the previous
    // value at static walls too would freeze whatever velocity a face had when its
    // wall appeared (or the finger left it) and keep pumping it into the fluid.
    const float *restrict ss = f->s_static;
    int step = comp == 0 ? n : 1;
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < n; j++) {
            int c = i * n + j;
            bool has_prev = comp == 0 ? i > 0 : j > 0;
            if (ct[c] == FLUID_CELL_SOLID || (has_prev && ct[c - step] == FLUID_CELL_SOLID)) {
                bool wall = ss[c] == 0.0f || (has_prev && ss[c - step] == 0.0f);
                fg[c] = wall ? 0.0f : prev[c];
            }
        }
    }
}

static void particles_to_grid(fluid_t *f)
{
    int n = f->ny;
    int nx = f->nx;
    float h1 = f->inv_h;
    size_t cells_f = (size_t)f->num_cells * sizeof(float);

    memcpy(f->prev_u, f->u, cells_f);
    memcpy(f->prev_v, f->v, cells_f);

    for (int i = 0; i < f->num_cells; i++) {
        f->cell_type[i] = f->s[i] == 0.0f ? FLUID_CELL_SOLID : FLUID_CELL_AIR;
    }
    for (int i = 0; i < f->num_particles; i++) {
        int xi = clampi((int)(f->pos[2 * i] * h1), 0, nx - 1);
        int yi = clampi((int)(f->pos[2 * i + 1] * h1), 0, n - 1);
        int c = xi * n + yi;
        if (f->cell_type[c] == FLUID_CELL_AIR) {
            f->cell_type[c] = FLUID_CELL_FLUID;
        }
    }

    run2(f, p2g_job, f);
}
static void update_density(fluid_t *f)
{
    int n = f->ny;
    int nx = f->nx;
    float h = f->h;
    float h1 = f->inv_h;
    float h2 = 0.5f * h;
    float *restrict d = f->density;
    memset(d, 0, (size_t)f->num_cells * sizeof(float));

    for (int i = 0; i < f->num_particles; i++) {
        float x = clampf(f->pos[2 * i], h, (float)(nx - 1) * h);
        float y = clampf(f->pos[2 * i + 1], h, (float)(n - 1) * h);
        int x0 = (int)((x - h2) * h1);
        float tx = ((x - h2) - (float)x0 * h) * h1;
        int x1 = x0 + 1 > nx - 2 ? nx - 2 : x0 + 1;
        int y0 = (int)((y - h2) * h1);
        float ty = ((y - h2) - (float)y0 * h) * h1;
        int y1 = y0 + 1 > n - 2 ? n - 2 : y0 + 1;
        float sx = 1.0f - tx;
        float sy = 1.0f - ty;
        d[x0 * n + y0] += sx * sy;
        d[x1 * n + y0] += tx * sy;
        d[x1 * n + y1] += tx * ty;
        d[x0 * n + y1] += sx * ty;
    }

    if (f->rest_density == 0.0f) {
        float sum = 0.0f;
        int count = 0;
        for (int i = 0; i < f->num_cells; i++) {
            if (f->cell_type[i] == FLUID_CELL_FLUID) {
                sum += d[i];
                count++;
            }
        }
        if (count > 0) {
            f->rest_density = sum / (float)count;
        }
    }
}

typedef struct {
    fluid_t *f;
    int n_red;
    int iters;
    atomic_int arrived;    // two-party spin barrier between colour sweeps
    atomic_int generation;
} pressure_job_t;

// Gauss-Seidel over fluid_list[k0, k1).
static void pressure_sweep(fluid_t *f, int k0, int k1)
{
    int n = f->ny;
    const float *restrict s = f->s;
    const float *restrict bias = f->density;
    const uint16_t *restrict list = f->fluid_list;
    const float *restrict k_list = f->inv_s_sum;
    float *restrict u = f->u;
    float *restrict v = f->v;

    for (int k = k0; k < k1; k++) {
        int c = list[k];
        float div = u[c + n] - u[c] + v[c + 1] - v[c] - bias[c];
        float p = -div * k_list[k];
        u[c] -= s[c - n] * p;
        u[c + n] += s[c + n] * p;
        v[c] -= s[c - 1] * p;
        v[c + 1] += s[c + 1] * p;
    }
}

// Both parts wait here until the other one arrives. Only valid when the two parts
// really run concurrently (cfg.parallel set).
static void pressure_barrier(pressure_job_t *job)
{
    int gen = atomic_load(&job->generation);
    if (atomic_fetch_add(&job->arrived, 1) == 1) {
        atomic_store(&job->arrived, 0);
        atomic_fetch_add(&job->generation, 1);
    } else {
        while (atomic_load(&job->generation) == gen) {
        }
    }
}

// All iterations in one job: cells of one red-black colour never share a face, so each
// colour is split between the parts, with a barrier before the other colour starts.
// One fork per solve instead of two per iteration.
static void pressure_job(void *ctx, int part)
{
    pressure_job_t *job = ctx;
    fluid_t *f = job->f;
    int nf = f->num_fluid;
    int red_mid = job->n_red / 2;
    int black_mid = job->n_red + (nf - job->n_red) / 2;
    int r0 = part == 0 ? 0 : red_mid;
    int r1 = part == 0 ? red_mid : job->n_red;
    int b0 = part == 0 ? job->n_red : black_mid;
    int b1 = part == 0 ? black_mid : nf;

    for (int it = 0; it < job->iters; it++) {
        pressure_sweep(f, r0, r1);
        pressure_barrier(job);
        pressure_sweep(f, b0, b1);
        pressure_barrier(job);
    }
}

static void solve_incompressibility(fluid_t *f, int iters, float dt)
{
    int n = f->ny;
    const float *restrict s = f->s;
    size_t cells_f = (size_t)f->num_cells * sizeof(float);

    memcpy(f->prev_u, f->u, cells_f);
    memcpy(f->prev_v, f->v, cells_f);

    // Only interior FLUID cells with an open neighbour take part, red cells ((i + j)
    // even) first, then black. Per cell, precompute omega / (open neighbours) and the
    // drift-compensation term, which do not change during the iterations.
    //
    // Drift compensation subtracts k * (density - rest) from the divergence. The
    // reference uses k = 1 in metres (h = 0.03 m, dt = 1/60 s), about 0.5 h/dt; in
    // pixels k = 1 is ~300x weaker, and the fluid slowly compresses under its own
    // weight (worse at lower frame rates). Hence k = drift_k * h / dt. Compression up
    // to drift_slack above rest is ignored so normal density noise does not make the
    // resting surface shimmer.
    float omega = f->cfg.over_relaxation;
    float rest = f->rest_density;
    float drift = f->cfg.drift_k * f->h * fluid_recip(dt);
    float slack_rest = rest * (1.0f + f->cfg.drift_slack);
    float *restrict bias = f->density; // density is not needed after this point
    int nf = 0;
    int n_red = 0;
    for (int colour = 0; colour < 2; colour++) {
        for (int i = 1; i < f->nx - 1; i++) {
            for (int j = 1 + ((i + 1 + colour) & 1); j < n - 1; j += 2) {
                int c = i * n + j;
                if (f->cell_type[c] != FLUID_CELL_FLUID) {
                    continue;
                }
                float sum = s[c - n] + s[c + n] + s[c - 1] + s[c + 1];
                if (sum == 0.0f) {
                    continue;
                }
                f->fluid_list[nf] = (uint16_t)c;
                f->inv_s_sum[nf] = omega * fluid_recip(sum);
                float compression = rest > 0.0f ? bias[c] - slack_rest : 0.0f;
                bias[c] = compression > 0.0f ? drift * compression : 0.0f;
                nf++;
            }
        }
        if (colour == 0) {
            n_red = nf;
        }
    }
    f->num_fluid = nf;
    f->stats.fluid_cells = nf;

    if (f->cfg.parallel == NULL) {
        for (int it = 0; it < iters; it++) {
            pressure_sweep(f, 0, n_red);
            pressure_sweep(f, n_red, nf);
        }
        return;
    }
    pressure_job_t job = {.f = f, .n_red = n_red, .iters = iters};
    atomic_init(&job.arrived, 0);
    atomic_init(&job.generation, 0);
    f->cfg.parallel(pressure_job, &job);
}

// Per-component grid-to-particle transfer; each writes only its velocity component.
static void g2p_job(void *ctx, int comp)
{
    fluid_t *f = ctx;
    int n = f->ny;
    int nx = f->nx;
    float h = f->h;
    float h1 = f->inv_h;
    float h2 = 0.5f * h;
    float flip = f->cfg.flip_ratio;
    const uint8_t *restrict ct = f->cell_type;
    float dx = comp == 0 ? 0.0f : h2;
    float dy = comp == 0 ? h2 : 0.0f;
    const float *restrict fg = comp == 0 ? f->u : f->v;
    const float *restrict pf = comp == 0 ? f->prev_u : f->prev_v;
    const float *restrict pos = f->pos;
    float *restrict vel = f->vel;
    int offset = comp == 0 ? n : 1;

    for (int i = 0; i < f->num_particles; i++) {
        float x = clampf(pos[2 * i], h, (float)(nx - 1) * h);
        float y = clampf(pos[2 * i + 1], h, (float)(n - 1) * h);
        int x0 = (int)((x - dx) * h1);
        if (x0 > nx - 2) x0 = nx - 2;
        float tx = ((x - dx) - (float)x0 * h) * h1;
        int x1 = x0 + 1 > nx - 2 ? nx - 2 : x0 + 1;
        int y0 = (int)((y - dy) * h1);
        if (y0 > n - 2) y0 = n - 2;
        float ty = ((y - dy) - (float)y0 * h) * h1;
        int y1 = y0 + 1 > n - 2 ? n - 2 : y0 + 1;
        float sx = 1.0f - tx;
        float sy = 1.0f - ty;
        float d0 = sx * sy;
        float d1 = tx * sy;
        float d2 = tx * ty;
        float d3 = sx * ty;
        int nr0 = x0 * n + y0;
        int nr1 = x1 * n + y0;
        int nr2 = x1 * n + y1;
        int nr3 = x0 * n + y1;
        // A face is valid when either adjacent cell is not air.
        float v0 = (ct[nr0] != FLUID_CELL_AIR || ct[nr0 - offset] != FLUID_CELL_AIR) ? d0 : 0.0f;
        float v1 = (ct[nr1] != FLUID_CELL_AIR || ct[nr1 - offset] != FLUID_CELL_AIR) ? d1 : 0.0f;
        float v2 = (ct[nr2] != FLUID_CELL_AIR || ct[nr2 - offset] != FLUID_CELL_AIR) ? d2 : 0.0f;
        float v3 = (ct[nr3] != FLUID_CELL_AIR || ct[nr3 - offset] != FLUID_CELL_AIR) ? d3 : 0.0f;
        float d = v0 + v1 + v2 + v3;
        if (d > 0.0f) {
            float inv_d = fluid_recip(d);
            float pic = (v0 * fg[nr0] + v1 * fg[nr1] + v2 * fg[nr2] + v3 * fg[nr3]) * inv_d;
            float corr = (v0 * (fg[nr0] - pf[nr0]) + v1 * (fg[nr1] - pf[nr1]) +
                          v2 * (fg[nr2] - pf[nr2]) + v3 * (fg[nr3] - pf[nr3])) * inv_d;
            float cur = vel[2 * i + comp];
            vel[2 * i + comp] = (1.0f - flip) * pic + flip * (cur + corr);
        }
    }
}

void fluid_step(fluid_t *f, float gx, float gy, float dt, int substeps)
{
    if (substeps < 1) {
        substeps = 1;
    }
    float sdt = dt / (float)substeps;
    fluid_stats_t *st = &f->stats;
    st->us_integrate = st->us_separate = st->us_collide = 0;
    st->us_p2g = st->us_density = st->us_pressure = st->us_g2p = 0;
    int64_t t = f->cfg.clock_us ? f->cfg.clock_us() : 0;

    for (int k = 0; k < substeps; k++) {
        integrate_job_t integrate = {.f = f, .dt = sdt, .gx = gx, .gy = gy};
        run2(f, integrate_job, &integrate);
        st->us_integrate += elapsed(f, &t);
        if (f->cfg.separation_iters > 0) {
            push_particles_apart(f, f->cfg.separation_iters);
        }
        st->us_separate += elapsed(f, &t);
        run2(f, collide_job, f);
        if (f->wall_grow_steps > 0) {
            f->wall_grow_steps--;
        }
        st->us_collide += elapsed(f, &t);
        apply_obstacle_to_grid(f);
        particles_to_grid(f);
        st->us_p2g += elapsed(f, &t);
        update_density(f);
        st->us_density += elapsed(f, &t);
        solve_incompressibility(f, f->cfg.pressure_iters, sdt);
        st->us_pressure += elapsed(f, &t);
        run2(f, g2p_job, f);
        st->us_g2p += elapsed(f, &t);
    }
}

const fluid_stats_t *fluid_get_stats(const fluid_t *f)
{
    return &f->stats;
}

int fluid_particle_count(const fluid_t *f)
{
    return f->num_particles;
}

const float *fluid_particle_positions(const fluid_t *f)
{
    return f->pos;
}

const float *fluid_particle_velocities(const fluid_t *f)
{
    return f->vel;
}

float fluid_particle_radius(const fluid_t *f)
{
    return f->radius;
}

float fluid_screen_offset(const fluid_t *f)
{
    return f->h;
}

float fluid_rest_particles_per_px2(const fluid_t *f)
{
    float dx = 2.0f * f->radius;
    return 1.0f / (dx * 0.8660254f * dx);
}
