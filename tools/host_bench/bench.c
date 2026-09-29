// Host benchmark and sanity checks for fluid_engine (no ESP-IDF needed).
// Runs a scripted scenario (settle, rotate, shake, zero-g, finger sweep), checks the
// particles stay valid, measures per-stage cost and the bytes each style would send
// to the panel, and dumps PPM frames. See run.sh.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "fluid_raster.h"
#include "fluid_render.h"
#include "fluid_sim.h"

#define W 410
#define H 502
#define BAND 16
#define FPS 60
#define QSPI_BYTES_PER_S 20.0e6 // 40 MHz x 4 lines

static int64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

typedef struct {
    fluid_raster_t raster;
    fluid_render_t render;
    uint8_t *level;
    uint8_t *foam;
    uint16_t *band;
    uint16_t *frame; // full frame, for PPM dumps
    double us_raster;
    double us_render;
    double bytes;
    double bytes_max;
    double bytes_tail; // last second, fluid settling
    int frames;
} style_ctx_t;

static void write_ppm(const char *path, const uint16_t *px)
{
    FILE *fp = fopen(path, "wb");
    if (fp == NULL) {
        perror(path);
        return;
    }
    fprintf(fp, "P6\n%d %d\n255\n", W, H);
    for (int i = 0; i < W * H; i++) {
        uint16_t c = px[i];
        unsigned char rgb[3] = {
            (unsigned char)(((c >> 11) & 31) * 255 / 31),
            (unsigned char)(((c >> 5) & 63) * 255 / 63),
            (unsigned char)((c & 31) * 255 / 31),
        };
        fwrite(rgb, 1, 3, fp);
    }
    fclose(fp);
}

// Same band loop the device runs; returns bytes that would go to the panel.
static long draw_frame(style_ctx_t *s)
{
    long bytes = 0;
    fluid_render_prepare(&s->render, s->level, s->foam);
    for (int y = 0; y < H; y += BAND) {
        int lines = H - y < BAND ? H - y : BAND;
        int x0;
        int x1;
        if (!fluid_render_band_dirty(&s->render, y, lines, &x0, &x1)) {
            continue;
        }
        if ((x0 & 1) || !(x1 & 1) || (y & 1) || (lines & 1)) {
            fprintf(stderr, "FAIL: misaligned band y=%d lines=%d x=%d..%d\n", y, lines, x0, x1);
            exit(1);
        }
        int w = x1 - x0 + 1;
        fluid_render_band(&s->render, y, lines, x0, x1, s->band);
        for (int l = 0; l < lines; l++) {
            memcpy(s->frame + (y + l) * W + x0, s->band + l * w, (size_t)w * sizeof(uint16_t));
        }
        bytes += (long)w * lines * 2;
    }
    fluid_render_commit(&s->render);
    return bytes;
}

// Gravity script, in g (screen axes), and the finger obstacle.
static void scenario(double t, float *gx, float *gy, int *finger, float *fx, float *fy, float *fvx)
{
    *finger = 0;
    *fx = *fy = *fvx = 0.0f;
    if (t < 3.0) {
        *gx = 0.0f; *gy = 1.0f;                                   // settle
    } else if (t < 7.0) {
        double a = (t - 3.0) / 4.0 * 2.0 * M_PI;                  // full turn
        *gx = (float)sin(a); *gy = (float)cos(a);
    } else if (t < 8.0) {
        *gx = (float)(1.5 * sin((t - 7.0) * 2.0 * M_PI * 5.0)); // shake
        *gy = 1.0f;
    } else if (t < 10.0) {
        *gx = 0.0f; *gy = 0.0f;                                   // flat on a table
    } else if (t < 12.0) {
        *gx = -1.0f; *gy = 0.0f;                                  // on its side
    } else if (t < 14.0) {
        *gx = 0.0f; *gy = 1.0f;                                   // finger sweep
        double u = (t - 12.0) / 2.0;
        *finger = 1;
        *fvx = (float)(W * 0.8 / 2.0);
        *fx = (float)(W * 0.1 + W * 0.8 * u);
        *fy = (float)(H * 0.78);
    } else {
        *gx = 0.0f; *gy = 1.0f;                                   // settle again
    }
}

int main(int argc, char **argv)
{
    fluid_config_t cfg = fluid_default_config();
    int substeps = 1;
    float gravity = 1800.0f;
    double seconds = 22.0;
    const char *out = "out";

    for (int i = 1; i + 1 < argc; i += 2) {
        if (!strcmp(argv[i], "--cell")) cfg.cell_size = (float)atof(argv[i + 1]);
        else if (!strcmp(argv[i], "--fill")) cfg.fill_fraction = (float)atof(argv[i + 1]);
        else if (!strcmp(argv[i], "--iters")) cfg.pressure_iters = atoi(argv[i + 1]);
        else if (!strcmp(argv[i], "--sep")) cfg.separation_iters = atoi(argv[i + 1]);
        else if (!strcmp(argv[i], "--substeps")) substeps = atoi(argv[i + 1]);
        else if (!strcmp(argv[i], "--gravity")) gravity = (float)atof(argv[i + 1]);
        else if (!strcmp(argv[i], "--flip")) cfg.flip_ratio = (float)atof(argv[i + 1]);
        else if (!strcmp(argv[i], "--damping")) cfg.damping = (float)atof(argv[i + 1]);
        else if (!strcmp(argv[i], "--seconds")) seconds = atof(argv[i + 1]);
        else if (!strcmp(argv[i], "--max-particles")) cfg.max_particles = atoi(argv[i + 1]);
        else if (!strcmp(argv[i], "--out")) out = argv[i + 1];
        else {
            fprintf(stderr, "unknown option %s\n", argv[i]);
            return 2;
        }
    }
    cfg.clock_us = now_us;
    mkdir(out, 0755);

    fluid_t *f = fluid_create(&cfg);
    if (f == NULL) {
        fprintf(stderr, "FAIL: fluid_create\n");
        return 1;
    }
    const fluid_stats_t *st = fluid_get_stats(f);
    int np0 = fluid_particle_count(f);
    printf("cell=%.0fpx grid=%dx%d particles=%d r=%.2fpx iters=%d sep=%d substeps=%d flip=%.2f damping=%.2f g=%.0fpx/s2\n",
           cfg.cell_size, st->cells_x, st->cells_y, np0, fluid_particle_radius(f), cfg.pressure_iters,
           cfg.separation_iters, substeps, cfg.flip_ratio, cfg.damping, gravity);

    style_ctx_t styles[FLUID_STYLE_COUNT];
    for (int s = 0; s < FLUID_STYLE_COUNT; s++) {
        style_ctx_t *c = &styles[s];
        memset(c, 0, sizeof(*c));
        int p = fluid_style_pitch((fluid_style_t)s);
        if (!fluid_raster_init(&c->raster, W, H, p, NULL)) return 1;
        c->raster.blur = s != FLUID_STYLE_LED;
        int cells = c->raster.cols * c->raster.rows;
        if (!fluid_render_init(&c->render, W, H, cells, p, false, NULL)) return 1;
        fluid_render_configure(&c->render, (fluid_style_t)s, 0, &c->raster);
        c->level = calloc((size_t)cells, 1);
        c->foam = calloc((size_t)cells, 1);
        c->band = calloc((size_t)W * BAND, sizeof(uint16_t));
        c->frame = calloc((size_t)W * H, sizeof(uint16_t));
    }

    const double dump_at[] = {2.9, 4.0, 5.0, 7.5, 9.5, 11.9, 13.0, 16.9};
    const int ndump = (int)(sizeof(dump_at) / sizeof(dump_at[0]));
    int next_dump = 0;
    double sum_stage[7] = {0};
    double sum_step = 0.0;
    double max_step = 0.0;
    int frames = (int)(seconds * FPS);
    float dt = 1.0f / FPS;
    int fail = 0;

    for (int k = 0; k < frames; k++) {
        double t = (double)k / FPS;
        float gx;
        float gy;
        int finger;
        float fx;
        float fy;
        float fvx;
        scenario(t, &gx, &gy, &finger, &fx, &fy, &fvx);
        fluid_set_obstacle(f, fx, fy, 28.0f, fvx, 0.0f, finger != 0);

        int64_t t0 = now_us();
        fluid_step(f, gx * gravity, gy * gravity, dt, substeps);
        double us = (double)(now_us() - t0);
        sum_step += us;
        if (us > max_step) max_step = us;
        sum_stage[0] += st->us_integrate;
        sum_stage[1] += st->us_separate;
        sum_stage[2] += st->us_collide;
        sum_stage[3] += st->us_p2g;
        sum_stage[4] += st->us_density;
        sum_stage[5] += st->us_pressure;
        sum_stage[6] += st->us_g2p;

        // Validity: count, finite, inside the container.
        int np = fluid_particle_count(f);
        const float *pos = fluid_particle_positions(f);
        const float *vel = fluid_particle_velocities(f);
        float off = fluid_screen_offset(f);
        int bad = 0;
        double ke = 0.0;
        for (int i = 0; i < np; i++) {
            float x = pos[2 * i] - off;
            float y = pos[2 * i + 1] - off;
            if (!isfinite(x) || !isfinite(y) || !isfinite(vel[2 * i]) || !isfinite(vel[2 * i + 1]) ||
                fluid_container_sdf(f, x, y) > 0.5f) {
                bad++;
            }
            ke += 0.5 * ((double)vel[2 * i] * vel[2 * i] + (double)vel[2 * i + 1] * vel[2 * i + 1]);
        }
        if (np != np0 || bad > 0) {
            fprintf(stderr, "FAIL t=%.2f: particles=%d/%d invalid=%d\n", t, np, np0, bad);
            fail = 1;
            break;
        }
        if (k % FPS == FPS - 1) {
            printf("t=%5.1fs mean_speed=%7.1f px/s fluid_cells=%d\n", t, sqrt(2.0 * ke / np), st->fluid_cells);
        }

        bool dump = next_dump < ndump && t >= dump_at[next_dump];
        for (int s = 0; s < FLUID_STYLE_COUNT; s++) {
            style_ctx_t *c = &styles[s];
            int64_t r0 = now_us();
            fluid_raster_run(&c->raster, f, c->level, c->foam);
            int64_t r1 = now_us();
            long bytes = draw_frame(c);
            int64_t r2 = now_us();
            if (k > 0) { // frame 0 is the forced full redraw
                c->us_raster += (double)(r1 - r0);
                c->us_render += (double)(r2 - r1);
                c->bytes += (double)bytes;
                if (bytes > c->bytes_max) c->bytes_max = (double)bytes;
                c->frames++;
                if (k >= frames - FPS) c->bytes_tail += (double)bytes;
            }
            if (dump) {
                char path[512];
                snprintf(path, sizeof(path), "%s/%s_%04.1fs.ppm", out, fluid_style_name((fluid_style_t)s), t);
                write_ppm(path, c->frame);
            }
        }
        if (dump) next_dump++;
    }

    const char *names[7] = {"integrate", "separate", "collide", "p2g", "density", "pressure", "g2p"};
    printf("\nsim per frame (host us): avg=%.0f max=%.0f\n", sum_step / frames, max_step);
    for (int i = 0; i < 7; i++) {
        printf("  %-10s %7.0f us  %4.1f%%\n", names[i], sum_stage[i] / frames, 100.0 * sum_stage[i] / sum_step);
    }
    printf("\nstyle    pitch grid     raster_us render_us  avg_KB  max_KB  est_flush_ms avg/max  rest_KB\n");
    for (int s = 0; s < FLUID_STYLE_COUNT; s++) {
        style_ctx_t *c = &styles[s];
        int n = c->frames > 0 ? c->frames : 1;
        printf("%-8s %5d %3dx%-4d %9.0f %9.0f %7.1f %7.1f   %5.1f / %5.1f  %7.1f\n", fluid_style_name((fluid_style_t)s),
               c->raster.pitch, c->raster.cols, c->raster.rows, c->us_raster / n, c->us_render / n,
               c->bytes / n / 1024.0, c->bytes_max / 1024.0, c->bytes / n / QSPI_BYTES_PER_S * 1000.0,
               c->bytes_max / QSPI_BYTES_PER_S * 1000.0, c->bytes_tail / FPS / 1024.0);
    }
    printf("\n%s\n", fail ? "RESULT: FAIL" : "RESULT: OK");
    return fail;
}
