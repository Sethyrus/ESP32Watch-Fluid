#include "fluid_clock.h"

#include <math.h>

#define DIGIT_W 112.0f
#define DIGIT_H 184.0f
#define STROKE_R 11.0f // half the stroke width
#define GAP_X 40.0f    // between the two digits of a row
#define GAP_Y 40.0f    // between the rows
// Segment ends are pulled back from the corners so neighbouring strokes leave a gap
// of ~10 px, wide enough for particles (and pressure cells) to pass.
#define INSET_H 26.0f // horizontal segments, from the vertical stroke centre line
#define INSET_V 19.0f // vertical segments, from the horizontal stroke centre line

// Segment centre lines in digit coordinates: x0, y0, x1, y1 (axis aligned).
static const float s_segments[7][4] = {
    {STROKE_R + INSET_H, STROKE_R, DIGIT_W - STROKE_R - INSET_H, STROKE_R},                      // a
    {DIGIT_W - STROKE_R, STROKE_R + INSET_V, DIGIT_W - STROKE_R, DIGIT_H / 2 - INSET_V},          // b
    {DIGIT_W - STROKE_R, DIGIT_H / 2 + INSET_V, DIGIT_W - STROKE_R, DIGIT_H - STROKE_R - INSET_V}, // c
    {STROKE_R + INSET_H, DIGIT_H - STROKE_R, DIGIT_W - STROKE_R - INSET_H, DIGIT_H - STROKE_R},  // d
    {STROKE_R, DIGIT_H / 2 + INSET_V, STROKE_R, DIGIT_H - STROKE_R - INSET_V},                    // e
    {STROKE_R, STROKE_R + INSET_V, STROKE_R, DIGIT_H / 2 - INSET_V},                              // f
    {STROKE_R + INSET_H, DIGIT_H / 2, DIGIT_W - STROKE_R - INSET_H, DIGIT_H / 2},                // g
};

static const unsigned char s_digit_segments[10] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f,
};

void fluid_clock_init(fluid_clock_t *c, float screen_w, float screen_h)
{
    float x0 = 0.5f * (screen_w - 2.0f * DIGIT_W - GAP_X);
    float y0 = 0.5f * (screen_h - 2.0f * DIGIT_H - GAP_Y);
    for (int k = 0; k < 4; k++) {
        c->x[k] = x0 + (float)(k & 1) * (DIGIT_W + GAP_X);
        c->y[k] = y0 + (float)(k >> 1) * (DIGIT_H + GAP_Y);
        c->segments[k] = 0;
    }
}

void fluid_clock_set(fluid_clock_t *c, int hours, int minutes)
{
    int d[4] = {hours / 10 % 10, hours % 10, minutes / 10 % 10, minutes % 10};
    for (int k = 0; k < 4; k++) {
        c->segments[k] = s_digit_segments[d[k]];
    }
}

static inline float max3(float a, float b, float c)
{
    float m = a > b ? a : b;
    return m > c ? m : c;
}

float fluid_clock_sdf(void *ctx, float x, float y)
{
    const fluid_clock_t *c = ctx;
    float best2 = 1e12f;
    for (int k = 0; k < 4; k++) {
        float lx = x - c->x[k];
        float ly = y - c->y[k];
        // Skip digits whose box is already farther than the best stroke.
        float bx = max3(-lx, lx - DIGIT_W, 0.0f);
        float by = max3(-ly, ly - DIGIT_H, 0.0f);
        if (bx * bx + by * by >= best2) {
            continue;
        }
        for (int s = 0; s < 7; s++) {
            if (!(c->segments[k] & (1u << s))) {
                continue;
            }
            const float *g = s_segments[s];
            float dx = max3(g[0] - lx, lx - g[2], 0.0f);
            float dy = max3(g[1] - ly, ly - g[3], 0.0f);
            float d2 = dx * dx + dy * dy;
            if (d2 < best2) {
                best2 = d2;
            }
        }
    }
    return sqrtf(best2) - STROKE_R;
}
