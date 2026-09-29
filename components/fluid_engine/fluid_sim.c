#include "fluid_internal.h"

#include <math.h>
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
        .pressure_iters = 30,
        .separation_iters = 1,
        .over_relaxation = 1.9f,
        .flip_ratio = 0.85f,
        .max_cells_per_step = 2.0f,
        .damping = 0.5f,
        .alloc = NULL,
        .clock_us = NULL,
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

    if (!f->u || !f->v || !f->du || !f->dv || !f->prev_u || !f->prev_v || !f->s || !f->s_static ||
        !f->density || !f->cell_type || !f->fluid_list || !f->inv_s_sum || !f->pos || !f->vel ||
        !f->cell_count || !f->cell_ids || f->num_cells > 65535 || f->capacity > 65535) {
        return NULL; // buffers are not freed: this only happens at boot with a bad config
    }

    // Container mask: a cell is open when its centre lies inside the rounded rect.
    for (int i = 0; i < f->nx; i++) {
        for (int j = 0; j < f->ny; j++) {
            float open = 0.0f;
            if (i > 0 && j > 0 && i < f->nx - 1 && j < f->ny - 1) {
                float cx = ((float)i + 0.5f) * f->h - f->h;
                float cy = ((float)j + 0.5f) * f->h - f->h;
                open = container_sdf(&f->cfg, cx, cy, NULL, NULL) < 0.0f ? 1.0f : 0.0f;
            }
            f->s_static[i * f->ny + j] = open;
            f->s[i * f->ny + j] = open;
        }
    }

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
            if (container_sdf(c, x, y, NULL, NULL) > -r) {
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

static void integrate_particles(fluid_t *f, float dt, float gx, float gy)
{
    float vmax = f->cfg.max_cells_per_step * f->h / dt;
    float vmax2 = vmax * vmax;
    float keep = 1.0f / (1.0f + f->cfg.damping * dt);
    float *restrict pos = f->pos;
    float *restrict vel = f->vel;
    for (int i = 0; i < f->num_particles; i++) {
        float vx = vel[2 * i] * keep + dt * gx;
        float vy = vel[2 * i + 1] * keep + dt * gy;
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

    float min_dist = 2.0f * f->radius;
    float min_dist2 = min_dist * min_dist;
    for (int it = 0; it < iters; it++) {
        // Each pair once: a cell against itself (j > i) and its 4 "forward" neighbours
        // (+x-1y, +x, +x+1y, +y). Same-iteration in-place updates, as in the reference.
        for (int xi = 0; xi < pnx; xi++) {
            for (int yi = 0; yi < pny; yi++) {
                int c = xi * pny + yi;
                int a_end = first[c + 1];
                for (int ka = first[c]; ka < a_end; ka++) {
                    int a = ids[ka];
                    float ax = pos[2 * a];
                    float ay = pos[2 * a + 1];
                    for (int n = 0; n < 5; n++) {
                        int nx_i = xi + (n == 0 || n == 4 ? 0 : 1);
                        int ny_i = yi + (n == 0 ? 0 : (n == 4 ? 1 : n - 2));
                        if (nx_i >= pnx || ny_i < 0 || ny_i >= pny) {
                            continue;
                        }
                        int nc = nx_i * pny + ny_i;
                        int kb = n == 0 ? ka + 1 : first[nc];
                        int b_end = first[nc + 1];
                        for (; kb < b_end; kb++) {
                            int b = ids[kb];
                            float dx = pos[2 * b] - ax;
                            float dy = pos[2 * b + 1] - ay;
                            float d2 = dx * dx + dy * dy;
                            if (d2 > min_dist2 || d2 == 0.0f) {
                                continue;
                            }
                            // s = 0.5 * (min_dist - d) / d without sqrtf or a division. Pairs
                            // are visited once (the reference visits twice), which packs the
                            // rest state ~8% denser; a larger factor (0.7) matches the
                            // reference volume but leaves the surface shimmering at rest.
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
}

static void handle_collisions(fluid_t *f)
{
    float h = f->h;
    float r = f->radius;
    float *restrict pos = f->pos;
    float *restrict vel = f->vel;
    float ob_r = f->obstacle_r + r;
    float ob_r2 = ob_r * ob_r;
    float cx = 0.5f * f->cfg.width + h;
    float cy = 0.5f * f->cfg.height + h;

    for (int i = 0; i < f->num_particles; i++) {
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

static void particles_to_grid(fluid_t *f)
{
    int n = f->ny;
    int nx = f->nx;
    float h = f->h;
    float h1 = f->inv_h;
    float h2 = 0.5f * h;
    size_t cells_f = (size_t)f->num_cells * sizeof(float);

    memcpy(f->prev_u, f->u, cells_f);
    memcpy(f->prev_v, f->v, cells_f);
    memset(f->du, 0, cells_f);
    memset(f->dv, 0, cells_f);
    memset(f->u, 0, cells_f);
    memset(f->v, 0, cells_f);

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

    for (int comp = 0; comp < 2; comp++) {
        float dx = comp == 0 ? 0.0f : h2;
        float dy = comp == 0 ? h2 : 0.0f;
        float *restrict fg = comp == 0 ? f->u : f->v;
        float *restrict dg = comp == 0 ? f->du : f->dv;
        for (int i = 0; i < f->num_particles; i++) {
            float x = clampf(f->pos[2 * i], h, (float)(nx - 1) * h);
            float y = clampf(f->pos[2 * i + 1], h, (float)(n - 1) * h);
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
            float pv = f->vel[2 * i + comp];
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
    }

    // Faces touching solid cells keep their previous (wall / obstacle) velocity.
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < n; j++) {
            int c = i * n + j;
            bool solid = f->cell_type[c] == FLUID_CELL_SOLID;
            if (solid || (i > 0 && f->cell_type[c - n] == FLUID_CELL_SOLID)) {
                f->u[c] = f->prev_u[c];
            }
            if (solid || (j > 0 && f->cell_type[c - 1] == FLUID_CELL_SOLID)) {
                f->v[c] = f->prev_v[c];
            }
        }
    }
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

static void solve_incompressibility(fluid_t *f, int iters)
{
    int n = f->ny;
    const float *restrict s = f->s;
    float *restrict u = f->u;
    float *restrict v = f->v;
    size_t cells_f = (size_t)f->num_cells * sizeof(float);

    memcpy(f->prev_u, u, cells_f);
    memcpy(f->prev_v, v, cells_f);

    // Only interior FLUID cells with an open neighbour take part. Per cell, precompute
    // omega / (open neighbours) and the drift-compensation term (compression above rest
    // density, k = 1), which do not change during the iterations.
    float omega = f->cfg.over_relaxation;
    float rest = f->rest_density;
    float *restrict bias = f->density; // density is not needed after this point
    int nf = 0;
    for (int i = 1; i < f->nx - 1; i++) {
        for (int j = 1; j < n - 1; j++) {
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
            float compression = rest > 0.0f ? bias[c] - rest : 0.0f;
            bias[c] = compression > 0.0f ? compression : 0.0f;
            nf++;
        }
    }
    f->num_fluid = nf;
    f->stats.fluid_cells = nf;

    const uint16_t *restrict list = f->fluid_list;
    const float *restrict k_list = f->inv_s_sum;
    for (int it = 0; it < iters; it++) {
        for (int k = 0; k < nf; k++) {
            int c = list[k];
            float div = u[c + n] - u[c] + v[c + 1] - v[c] - bias[c];
            float p = -div * k_list[k];
            u[c] -= s[c - n] * p;
            u[c + n] += s[c + n] * p;
            v[c] -= s[c - 1] * p;
            v[c + 1] += s[c + 1] * p;
        }
    }
}

static void grid_to_particles(fluid_t *f)
{
    int n = f->ny;
    int nx = f->nx;
    float h = f->h;
    float h1 = f->inv_h;
    float h2 = 0.5f * h;
    float flip = f->cfg.flip_ratio;
    const uint8_t *restrict ct = f->cell_type;

    for (int comp = 0; comp < 2; comp++) {
        float dx = comp == 0 ? 0.0f : h2;
        float dy = comp == 0 ? h2 : 0.0f;
        const float *restrict fg = comp == 0 ? f->u : f->v;
        const float *restrict pf = comp == 0 ? f->prev_u : f->prev_v;
        int offset = comp == 0 ? n : 1;
        for (int i = 0; i < f->num_particles; i++) {
            float x = clampf(f->pos[2 * i], h, (float)(nx - 1) * h);
            float y = clampf(f->pos[2 * i + 1], h, (float)(n - 1) * h);
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
                float cur = f->vel[2 * i + comp];
                f->vel[2 * i + comp] = (1.0f - flip) * pic + flip * (cur + corr);
            }
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
        integrate_particles(f, sdt, gx, gy);
        st->us_integrate += elapsed(f, &t);
        if (f->cfg.separation_iters > 0) {
            push_particles_apart(f, f->cfg.separation_iters);
        }
        st->us_separate += elapsed(f, &t);
        handle_collisions(f);
        st->us_collide += elapsed(f, &t);
        apply_obstacle_to_grid(f);
        particles_to_grid(f);
        st->us_p2g += elapsed(f, &t);
        update_density(f);
        st->us_density += elapsed(f, &t);
        solve_incompressibility(f, f->cfg.pressure_iters);
        st->us_pressure += elapsed(f, &t);
        grid_to_particles(f);
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
