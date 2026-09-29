# AGENTS.md

## Project Shape
- ESP-IDF C firmware `ESP32WatchFluid`: a real-time FLIP/PIC fluid simulation driven by the IMU, drawn straight to the AMOLED with no LVGL at runtime. The entrypoint is `app_main()` in `main/main.c`, which only calls `fluid_app_start()`.
- Target hardware is Waveshare `ESP32-S3-Touch-AMOLED-2.06`: ESP32-S3R8, AMOLED 410x502 QSPI, FT3168 touch, QMI8658 IMU, AXP2101 PMU.
- Baseline stack is `ESP-IDF 5.5.4 + waveshare/esp32_s3_touch_amoled_2_06` BSP. LVGL 9.3.0 stays pinned because the BSP pulls it in, but it is never started. Do not migrate to ESP-IDF 6.x unless explicitly requested.
- Shared board services (`imu_service.h`, `watch_buttons.h`) come from `watch_board` in https://github.com/Sethyrus/ESP32Watch-core, pinned by tag in `main/idf_component.yml`. Fix hardware bugs there, not in local copies. Hardware docs live in that repo's `docs/`.
- Components:
  - `components/fluid_engine/` is pure C with no ESP-IDF includes: `fluid_sim` (FLIP), `fluid_raster` (particles to LED grid) and `fluid_render` (RGB565 bands plus dirty detection). Keep it that way so `tools/host_bench` keeps building on the host.
  - `components/fluid_app/` holds everything ESP-specific: panel, tasks, IMU, touch, BOOT and Kconfig.
- Design, budget, measurements and next steps: `docs/FLUID_DESIGN.md`. Update its measurements table when tuning on hardware.
- Durable config lives in `sdkconfig.defaults`, `partitions.csv`, component manifests and `dependencies.lock`. `sdkconfig`, `build/` and `managed_components/` are generated.

## Commands
- Source ESP-IDF: `source "$HOME/.espressif/v5.5.4/esp-idf/export.sh"`.
- First setup or fresh config: `idf.py set-target esp32s3`.
- Build (primary verification): `idf.py build`.
- Flash and monitor: `idf.py -p <PORT> flash monitor`. The macOS port looks like `/dev/tty.usbmodem*` and changes with the USB socket; `idf.py` auto-detects it if `-p` is omitted.
- Host benchmark and checks for the engine: `tools/host_bench/run.sh [--cell N --iters N --substeps N --sep N --flip F --damping F --gravity G --fill F --out DIR]`. It fails, with a non-zero exit, on lost, NaN or escaped particles and on misaligned bands. Run it after any change to `fluid_engine`.
- There are no other test, lint or format targets; do not invent npm, PlatformIO or pytest commands.

## Runtime Notes
- **Display.**
  - `bsp_display_new()` provides the panel (same pattern as ESP32Watch-Doom). Bands are 16 lines, double-buffered in internal DMA RAM, big-endian RGB565.
  - The panel needs an even x/y start and an odd inclusive end for every window. `fluid_render_band_dirty()` guarantees this; keep it if you change banding.
  - QSPI runs at 40 MHz (fixed in the BSP), about 20 MB/s, so a full frame takes about 20.6 ms. Only dirty bands are sent.
- **Tasks.**
  - `fluid_sim` runs on core 1 at a fixed 60 Hz. It owns all I2C: IMU and touch.
  - `fluid_render` runs on core 0 and takes the latest frame from a triple buffer.
  - `imu_service` is not thread-safe. Only call it from `fluid_sim`.
- **Gravity.** It is absolute: `imu_service_calibrate()` is never called on purpose. The watch lying flat means zero gravity.
- **Buttons.** BOOT short press cycles the style, BOOT long press cycles the palette. PWR is unused, because this is the app's root screen (core button convention). Holding PWR for ~6 s powers the board off.
- **Memory.** Simulation, raster and render buffers go to internal RAM, falling back to PSRAM with a warning; frame buffers go to PSRAM. Watch `internal_free` in the stats log when raising particle counts.
- **Stats.** Both tasks log every 5 s:
  - `sim:` step, separation, pressure, raster, overruns, heap;
  - `render:` fps, frame time, KB sent.

## Working Rules
- Do not commit, tag or push. The owner reviews and commits.
