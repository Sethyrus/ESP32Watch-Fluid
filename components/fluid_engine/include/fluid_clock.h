#pragma once

// Clock face made of walls: HH on top, MM below, as 7-segment digits drawn with
// rounded strokes. Segments stop short of each other so the fluid flows in and out of
// the digits. Pass fluid_clock_sdf with the clock as ctx to fluid_set_walls().

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x[4]; // digit top-left corners, screen px
    float y[4];
    unsigned char segments[4]; // bit 0..6 = segments a..g
} fluid_clock_t;

void fluid_clock_init(fluid_clock_t *clock, float screen_w, float screen_h);

// 24 h; hours keep the leading zero.
void fluid_clock_set(fluid_clock_t *clock, int hours, int minutes);

// Signed distance in screen px (negative inside a stroke).
float fluid_clock_sdf(void *clock, float x, float y);

#ifdef __cplusplus
}
#endif
