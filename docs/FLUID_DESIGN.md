# Diseno: simulacion de fluido

## Objetivo

Un fluido tipo "colgante de matriz LED" que se mueve con la inclinacion real del reloj. Buscamos la mejor relacion entre fps, resolucion y fidelidad que permita la placa:
- 60 fps de objetivo;
- aspecto vivo, con salpicaduras;
- sin perder volumen;
- en reposo, quieto.

## Decisiones cerradas

| Tema | Decision | Por que |
| --- | --- | --- |
| Metodo | FLIP/PIC (Mueller, Ten Minute Physics #18) | La rejilla da incompresibilidad barata y las particulas transportan el fluido sin perder volumen, con salpicaduras. Es lo mismo que usan los colgantes de mitxela en un M4 a 100 MHz. SPH es mas caro (vecinos para todo) y una rejilla sola (Stam) disipa volumen. |
| Gravedad | Absoluta, sin calibracion de postura | Un liquido cae hacia el "abajo" real. `imu_service` sin `imu_service_calibrate()` tiene bias 0, asi que devuelve la gravedad proyectada en la pantalla, en g, mas la aceleracion lineal (al sacudir, salpica). Plano sobre la mesa, gravedad aprox. 0 y el fluido flota. |
| Pantalla | Directa al panel (`bsp_display_new`), sin LVGL | El limite es el bus. Hace falta control de que se envia y cuando. |
| Envio | Solo las franjas que cambian | El fondo negro y el fluido en reposo no se reenvian. |
| Estilos | LED (pitch 8), Pixel (pitch 5), Liquid (pitch 5, suave) | La misma simulacion con tres looks, que se cambian con BOOT. |
| Contenedor | Rectangulo redondeado, R = 114 px | Es la forma visible del AMOLED (R9.2 mm, core `HARDWARE.md`). Asi el fluido no se esconde en esquinas que no se ven. |

## Presupuesto

### Bus de pantalla (el limite duro)

- El QSPI va a 40 MHz, fijo en `SH8601_PANEL_IO_QSPI_CONFIG` del BSP. Son unos 20 MB/s.
- Un frame completo son 410 x 502 x 2 = 411.640 B, unos **20,6 ms**. Con frame completo el techo es de **~48 fps**.
- Por eso el render solo envia las franjas de 16 lineas que cambian, recortadas al rango x cambiado (inicio par y fin impar, que el panel exige).
- La **histeresis** de nivel (10/255) evita que el temblor de las particulas en reposo reenvie la superficie en cada frame.

### CPU

- Hay dos nucleos a 240 MHz con FPU de precision simple.
- **core 1:** entrada, `fluid_step` y `fluid_raster_run`, a 60 Hz fijos (`dt` = 1/60).
- **core 0:** `fluid_render_*` y el DMA. El pipeline es un triple buffer: la simulacion prepara el frame k+1 mientras se envia el k, y ninguna tarea espera a la otra.
- El coste por etapa se ve en el benchmark host y en el log `sim:`. En el host, la separacion de particulas (hash grid 3x3) es alrededor del 55 % y la presion alrededor del 27 %.

### Memoria (celda de 10 px)

| Buffer | Tamano | Donde |
| --- | --- | --- |
| Rejilla | 43 x 53 celdas x 11 arrays aprox. | ~100 KB internos |
| Particulas | ~2.500 x 16 B, mas hash | ~60 KB internos |
| Raster LED (51x62) y fino (82x100) | ~57 KB | internos |
| Render: codigos, teselas, LUT | ~35 KB | internos |
| Bandas DMA | 2 x 13 KB | internos (DMA) |
| Frames (triple buffer) | 3 x 2 x 8.200 B | PSRAM |

Todo se reserva al arrancar, y no hay `malloc` en el bucle.

## Pipeline por frame

1. **Entrada** (core 1):
   - IMU: `imu_service_read`, una rafaga de 6 bytes;
   - tacto: FT3168;
   - BOOT: debounce de 30 ms; corta por debajo de 700 ms y larga a partir de 700 ms.
2. **`fluid_step`**, por cada substep:
   1. integrar, con amortiguacion de 0,5/s y limite de 2 celdas por substep;
   2. separar particulas (1 iteracion);
   3. colisiones: SDF del contenedor y obstaculo del dedo;
   4. P2G;
   5. densidad;
   6. presion: Gauss-Seidel con SOR de 1,9 y compensacion de deriva, solo sobre la lista de celdas fluidas;
   7. G2P, con FLIP 0,85.
3. **`fluid_raster_run`**:
   - splat bilineal de las particulas a la rejilla del estilo;
   - desenfoque 1-2-1 en los estilos finos;
   - normalizacion por la densidad de reposo;
   - filtro temporal;
   - la espuma sale de la velocidad media de cada celda.
4. **Render** (core 0):
   - `prepare`: histeresis y codigos visuales;
   - por franja: `band_dirty`, `band` y `esp_lcd_panel_draw_bitmap`;
   - al final, `commit`.
   - **Estilos LED y Pixel:** teselas precalculadas de 64 codigos (16 niveles x 4 de espuma).
   - **Liquid:** interpolacion bilineal entera del campo, con una LUT de 4 x 256 (borde antialias y sombreado por profundidad).

## Parametros (Kconfig: *Fluid simulation*)

| Opcion | Por defecto | Efecto |
| --- | --- | --- |
| `FLUID_CELL_PX` | 10 | Es el mando principal. Celda de 8 px: unas 3.900 particulas y ~1,6x de CPU. Celda de 12 px: unas 1.700 particulas. |
| `FLUID_FILL_PERCENT` | 40 | Cuanto fluido hay. |
| `FLUID_SUBSTEPS` | 1 | Estabilidad frente a CPU. Con 2 se duplica el coste. |
| `FLUID_PRESSURE_ITERS` | 30 | Incompresibilidad. Con 20 la presion cuesta ~35 % menos. |
| `FLUID_GRAVITY_PX_PER_G` | 1800 | Escala del "tanque virtual" (~0,3 m). |
| `FLUID_BRIGHTNESS` | 80 | Brillo del panel, comando 0x51. |

El resto (FLIP, amortiguacion y separacion) esta en `fluid_default_config()` y en `fluid_raster_init()`.

## Medidas

### Host (Apple Silicon, referencia relativa, no del reloj)

Configuracion: `tools/host_bench/run.sh` con los valores por defecto. Celda de 10 px, 2.497 particulas, 30 iteraciones, 1 substep, separacion 1, FLIP 0,85 y amortiguacion 0,5.

| Estilo | Rejilla | KB/frame medio | KB/frame max | Flush estimado medio/max | KB en reposo |
| --- | --- | --- | --- | --- | --- |
| LED | 51x62 | 59 | 215 | 3,0 / 11,0 ms | 42 |
| Pixel | 82x100 | 68 | 226 | 3,5 / 11,6 ms | 51 |
| Liquid | 82x100 | 100 | 253 | 5,1 / 12,9 ms | 87 |

Incluso el peor frame cabe en 16,7 ms de bus, asi que el limite real lo pondra la CPU del reloj.

Barrido en el host, en us por frame: separacion 2 a 1 ahorra ~30 %; substeps 2 a 1 ahorra ~45 %; celda 8 px cuesta ~1,6x y celda 12 px ~0,7x.

### Reloj (pendiente)

Rellenar con el log `sim:` / `render:` tras flashear:

| Celda | Particulas | Iteraciones | Substeps | step_avg (us) | render fps (LED / Pixel / Liquid) | internal_free | Notas |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 10 | ~2.500 | 30 | 1 | | | | |
| 8 | ~3.900 | 30 | 1 | | | | |
| 12 | ~1.700 | 30 | 1 | | | | |

**Criterio:** el mejor tier con 50 fps o mas chapoteando y 60 en reposo.
- Si `step_avg` pasa de unos 14 ms, primero bajar iteraciones (20) y despues subir la celda.
- Si sobra CPU, bajar la celda.

## Riesgos y mitigaciones

| Riesgo | Mitigacion |
| --- | --- |
| La simulacion no cabe en 16,7 ms en el S3 | Kconfig por tiers. Mover la separacion o la presion al core 0 si el render va sobrado. |
| Tearing (el panel refresca mientras llega una franja) | No hay TE conectado en el BSP. Probar TE en GPIO13 (comando 0x35) si se nota. |
| Poca memoria interna al bajar la celda | Asignacion con fallback a PSRAM y aviso en el log. Vigilar `internal_free`. |
| Temblor en reposo | Amortiguacion 0,5/s, histeresis de nivel y filtro temporal en el raster. |
| Fallo del IMU | Gravedad fija hacia abajo y contador `imu_err` en el log. |

## Siguientes pasos (fuera de la primera version)

- **Calibracion de bias**, plana sobre la mesa y guardada en NVS. Hace falta una API en core (`imu_service_set_bias`) y un tag nuevo. Solo corrige el offset del sensor, no la postura.
- **QSPI a 80 MHz** con configuracion IO propia en vez de `bsp_display_new`. Duplicaria el techo del bus si el panel lo aguanta.
- **Sincronizar con TE** contra el tearing.
- **Atenuar el brillo o dormir por inactividad**: fluido quieto y sin tacto durante N s.
- **Giroscopio:** inercia al girar el reloj en su plano.
- **Colores por particula** (mezcla de dos liquidos).
