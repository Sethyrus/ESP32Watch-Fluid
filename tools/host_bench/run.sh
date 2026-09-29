#!/bin/sh
# Builds and runs the host benchmark. Extra arguments go to the bench, e.g.:
#   tools/host_bench/run.sh --cell 8 --substeps 2 --out /tmp/fluid_frames
#   CFLAGS_EXTRA=-fsanitize=thread tools/host_bench/run.sh --threads --seconds 4
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ENGINE="$HERE/../../components/fluid_engine"
BUILD="${BUILD_DIR:-$HERE/build}"
mkdir -p "$BUILD"
cc -std=c11 -O2 ${CFLAGS_EXTRA:-} -Wall -Wextra -Werror -D_POSIX_C_SOURCE=200809L \
    -I"$ENGINE/include" -I"$ENGINE" \
    "$HERE/bench.c" "$ENGINE/fluid_sim.c" "$ENGINE/fluid_raster.c" "$ENGINE/fluid_render.c" \
    -lm -lpthread -o "$BUILD/fluid_bench"
cd "$BUILD"
exec ./fluid_bench "$@"
