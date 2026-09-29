# Pure C engine (simulation, raster, render); also built on the host by tools/host_bench.
idf_component_register(
    SRCS
        "fluid_sim.c"
        "fluid_raster.c"
        "fluid_render.c"
        "fluid_clock.c"
    INCLUDE_DIRS "include"
)
