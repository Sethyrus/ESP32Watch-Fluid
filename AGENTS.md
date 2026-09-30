# AGENTS.md

## Project Shape
- ESP-IDF C firmware `ESP32WatchFluid`: a real-time FLIP/PIC fluid simulation driven by the IMU, drawn straight to the AMOLED; LVGL only runs for the settings menu. It doubles as a clock: the time digits are walls the fluid flows around. The entrypoint is `app_main()` in `main/main.c`, which only calls `fluid_app_start()`.
- Target hardware is Waveshare `ESP32-S3-Touch-AMOLED-2.06`: ESP32-S3R8, AMOLED 410x502 QSPI, FT3168 touch, QMI8658 IMU, AXP2101 PMU.
- Baseline stack is `ESP-IDF 5.5.4 + waveshare/esp32_s3_touch_amoled_2_06` BSP. LVGL 9.3.0 is pinned. It only runs for the PWR menu (`fluid_menu.c`), inside the render task, with no esp_lvgl_port. Do not migrate to ESP-IDF 6.x unless explicitly requested.
- Shared board services (`imu_service.h`, `watch_buttons.h`, `watch_rtc.h`, `watch_nvs.h`, `watch_launcher.h`) come from `watch_board` in https://github.com/Sethyrus/ESP32Watch-core, pinned by tag in `main/idf_component.yml`. Fix hardware bugs there, not in local copies. Hardware docs live in that repo's `docs/`.
- Components:
  - `components/fluid_engine/` is pure C with no ESP-IDF includes: `fluid_sim` (FLIP plus walls), `fluid_clock` (7-segment digits as an SDF), `fluid_raster` (particles to LED grid) and `fluid_render` (RGB565 bands plus dirty detection). Keep it that way so `tools/host_bench` keeps building on the host.
  - `components/fluid_app/` holds everything ESP-specific: panel, tasks, IMU, touch, buttons, NVS settings, the LVGL menu (`fluid_menu.c`, allocator in `lv_mem_core_psram.c`) and Kconfig.
  - The RTC (`watch_rtc.h`) and NVS init (`watch_nvs.h`) come from core `watch_board` (>= v0.3.0). Settings use the `fluid` NVS namespace only.
- Design, budget, measurements and next steps: `docs/FLUID_DESIGN.md`. Update its measurements table when tuning on hardware.
- Durable config lives in `sdkconfig.defaults`, `partitions.csv`, component manifests and `dependencies.lock`. `sdkconfig`, `build/` and `managed_components/` are generated.

## Commands
- Source ESP-IDF: `source "$HOME/.espressif/v5.5.4/esp-idf/export.sh"`.
- First setup or fresh config: `idf.py set-target esp32s3`.
- Build (primary verification): `idf.py build`.
- Flash and monitor: `idf.py -p <PORT> flash monitor`. The macOS port looks like `/dev/tty.usbmodem*` and changes with the USB socket; `idf.py` auto-detects it if `-p` is omitted.
- Host benchmark and checks for the engine: `tools/host_bench/run.sh [--cell N --iters N --substeps N --sep N --flip F --damping F --gravity G --fill F --fps N --drift K --slack S --clock HHMM --splash T --palette N --threads --out DIR]`.
  - It fails, with a non-zero exit, on lost, NaN or escaped particles, on particles left inside walls and on misaligned bands.
  - Run it after any change to `fluid_engine`, with and without `--clock`.
  - Parallel jobs: `CFLAGS_EXTRA=-fsanitize=thread BUILD_DIR=/tmp/tsan tools/host_bench/run.sh --threads`.
- There are no other test, lint or format targets; do not invent npm, PlatformIO or pytest commands.

## Runtime Notes
- **Display.**
  - `bsp_display_new()` provides the panel (same pattern as ESP32Watch-Doom). Bands are 16 lines, double-buffered in internal DMA RAM, big-endian RGB565.
  - The panel needs an even x/y start and an odd inclusive end for every window. `fluid_render_band_dirty()` guarantees this; keep it if you change banding.
  - QSPI runs at 40 MHz (fixed in the BSP), about 20 MB/s, so a full frame takes about 20.6 ms. Only dirty bands are sent.
- **Tasks.**
  - `fluid_sim` (core 1, priority 5) targets 60 Hz, and `dt` is the real frame time clamped to [1/120, 1/30] s. It owns the IMU, touch, BOOT and PWR.
  - `fluid_helper` (core 0, priority 6) runs the second half of each engine job (`fluid_config_t.parallel`). Jobs must touch disjoint memory in their two parts; check with the TSan bench.
  - `fluid_render` (core 0, priority 7, above the helper so the bus never idles) takes the latest frame from a triple buffer.
  - `imu_service` is not thread-safe. Only call it from `fluid_sim`.
- **Menu.**
  - PWR sets `s_menu_open`. The sim task stops stepping and only polls the buttons; the render task runs `fluid_menu_run()`.
  - While the menu is open, touch belongs to LVGL, and only the render task touches the panel.
  - Settings (`fluid_settings_t`) change hands through that flag and are saved to NVS (namespace `fluid`).
- **Performance rule.** On the S3, float `/` and `sqrtf` are software calls. Never use them per particle or per cell in `fluid_engine`; use `fluid_recip` and `fluid_rsqrt` from `fluid_internal.h`.
- **Gravity.** It is absolute: `imu_service_calibrate()` is never called on purpose. The watch lying flat means zero gravity.
- **Buttons.**
  - PWR short press opens and closes the menu (core convention: PWR = back/menu).
  - BOOT short press cycles the style, and BOOT long press cycles the palette (shortcuts). Inside the menu, BOOT closes it.
  - Holding PWR for ~6 s powers the board off.
- Launcher mode (core `watch_launcher.h`): `app_main` calls `watch_launcher_boot_once()` first. The menu shows `Salir al launcher` only when `watch_launcher_is_available()`; `run_menu()` saves the settings and then calls `watch_launcher_exit()`. `partitions.csv` is a copy of the shared table in ESP32Watch-Launcher; do not change it here alone.
- **Walls and clock.**
  - `fluid_set_walls()` samples the SDF once, which costs ~24 ms on the S3, so call it only when the minute changes.
  - Faces against static solids are zeroed in P2G; do not go back to keeping the previous value there (it pumps fluid).
- **Memory.** Simulation, raster and render buffers go to internal RAM, falling back to PSRAM with a warning. Frame buffers, the menu background and all LVGL allocations go to PSRAM. `internal_free` is ~18 KB with the clock, so watch it when adding internal buffers or raising particle counts.
- **Stats.** Both tasks log every 5 s:
  - `sim:` step, separation, pressure, raster, overruns, heap;
  - `render:` fps, frame time, KB sent.

## Working Rules
- Do not commit, tag or push. The owner reviews and commits.
