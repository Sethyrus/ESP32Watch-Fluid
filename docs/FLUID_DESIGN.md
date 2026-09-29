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
| Pantalla | Directa al panel (`bsp_display_new`); LVGL solo en el menu | El limite es el bus. Hace falta control de que se envia y cuando. El menu (PWR) pausa el fluido y ejecuta LVGL en la tarea de render, con los mismos buffers DMA, asi que nunca compiten por el panel. |
| Reloj | Digitos de 7 segmentos como paredes del fluido | La hora se lee por contraste, y el fluido la rodea. Los segmentos no se tocan en las uniones, para que el fluido entre y salga de los huecos del 0, del 8, etc. |
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
- **core 1:** entrada, `fluid_step` y `fluid_raster_run`.
  - El ritmo objetivo es 60 Hz, pero `dt` es el tiempo real entre frames, limitado a [1/120, 1/30] s. Asi el fluido va a velocidad fisica aunque haya menos fps.
- **core 0:** `fluid_render_*` y el DMA, y ademas la tarea auxiliar que ejecuta la segunda mitad de cada etapa del motor (`fluid_config_t.parallel`):
  - integrar, colisiones, P2G y G2P: por mitades de particulas o por componente;
  - separacion: por columnas del hash;
  - presion: Gauss-Seidel red-black, cada color repartido, con una barrera de espera activa.
  - Prioridades: render 7 > auxiliar 6 > simulacion 5. El render vuelve a llenar el bus en cuanto acaba una franja.
- **Triple buffer:** la simulacion prepara el frame k+1 mientras se envia el k, y ninguna tarea espera a la otra.
- **En el S3 la division float y `sqrtf` son llamadas de software** (`__divsf3`, `sqrtf`; ni `-ffast-math` las evita) y cuestan mas de 100 ciclos. En los bucles calientes se usan `fluid_recip` y `fluid_rsqrt` (`fluid_internal.h`). **Regla: nada de `/` ni `sqrtf` por particula ni por celda.**
- Mover el motor a IRAM no mejoro nada: el coste es de calculo (nucleo in-order), no de cache.
- El coste por etapa se ve en el benchmark host y en el log `sim:`.

### Memoria (celda de 10 px)

| Buffer | Tamano | Donde |
| --- | --- | --- |
| Rejilla | 43 x 53 celdas x 11 arrays aprox. | ~100 KB internos |
| Particulas | ~2.500 x 16 B, mas hash | ~60 KB internos |
| Raster LED (51x62) y fino (82x100) | ~57 KB | internos |
| Render: codigos, teselas, LUT | ~35 KB | internos |
| Bandas DMA | 2 x 13 KB | internos (DMA) |
| Frames (triple buffer) | 3 x 2 x 8.200 B | PSRAM |
| Paredes: campo de distancia int8 (h/2) + mascaras por estilo | ~8,6 KB + ~20 KB | internos |
| Menu: fondo (captura atenuada) y objetos LVGL | ~400 KB + lo que pida LVGL | PSRAM (`lv_mem_core_psram.c`) |

Todo se reserva al arrancar (el fondo del menu, al abrirlo por primera vez), y no hay `malloc` en el bucle del fluido. Con el reloj, `internal_free` ronda los 18 KB: poco margen.

## Pipeline por frame

1. **Entrada** (core 1):
   - IMU: `imu_service_read`, una rafaga de 6 bytes;
   - tacto: FT3168;
   - BOOT: debounce de 30 ms; corta por debajo de 700 ms y larga a partir de 700 ms.
2. **`fluid_step`**, por cada substep:
   1. integrar, con amortiguacion de 0,5/s y limite de 2 celdas por substep;
   2. separar particulas (1 iteracion);
   3. colisiones: SDF del contenedor, paredes (campo muestreado) y obstaculo del dedo;
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
   - **Estilos LED y Pixel:** teselas precalculadas de 65 codigos (16 niveles x 4 de espuma, mas la pared).
   - **Liquid:** interpolacion bilineal entera del campo, con una LUT de 4 x 256 (borde antialias y sombreado por profundidad). Las paredes usan su propia mascara, y un cambio en ella redibuja la pantalla entera (una vez por minuto).

## Paredes y reloj

- **API:** `fluid_set_walls(fluid, sdf, ctx)` recibe una distancia con signo en px.
  - Se muestrea una vez en un campo int8 cada h/2 (1/4 px de resolucion, saturado a +-31 px); la funcion no se guarda.
  - Las celdas cuyo centro cae dentro de una pared pasan a solidas en `s_static`.
  - Las colisiones empujan por el gradiente del campo.
- **Paredes nuevas:** durante 20 pasos tras cada cambio, el empuje se limita a 0,5 celdas por paso. El digito nuevo aparta el fluido en vez de lanzarlo.
- **Caras solidas en P2G:** contra paredes estaticas (contenedor y paredes) valen 0; contra el dedo conservan su velocidad.
  - Antes conservaban el valor anterior tambien en las estaticas. Una pared que aparecia sobre fluido en movimiento, o el dedo al irse junto al borde, dejaba esa velocidad fija para siempre, bombeando fluido.
- **Digitos (`fluid_clock`):**
  - 7 segmentos alineados con los ejes, de 112x184 px y trazo de 22 px;
  - HH arriba y MM abajo;
  - huecos de ~10 px en las uniones.
  - La SDF usa distancias al cuadrado y una sola `sqrtf` por muestra.
- **Coste en el reloj:**
  - regenerar las paredes tarda ~24 ms una vez por minuto (se pierde aproximadamente un frame);
  - las colisiones cuestan ~2,1 ms por frame con paredes;
  - aun asi el fps sube, porque los digitos ocupan volumen y quedan menos celdas fluidas que resolver.
- **Hora:** RTC PCF85063 (`components/watch_rtc`).
  - Al arrancar se copia al reloj del sistema, que se usa con `time()`, sin mas I2C.
  - Si el RTC perdio la hora, se escribe la de compilacion de `watch_rtc.c`.
  - Se ajusta desde el menu.

## Easter egg

- **Deteccion:** tres picos de aceleracion en el plano de la pantalla, por encima de 1,8 g, en menos de 1,5 s.
  - `imu_service` suaviza las muestras, asi que el umbral es menor que el de los picos reales. Cada pico se registra en el log para poder ajustarlo.
- **Efecto:** `fluid_splash()` lanza el fluido en contra de la gravedad (estimacion lenta), de forma desigual por particula.
  - Ruido puro no sirve: la proyeccion de presion lo anula casi entero.
- **Recompensa:** desbloquea la paleta "Arcoiris", guardada en NVS.
  - El tono sigue la velocidad (codigo de espuma, con `foam_speed` de 450 px/s): violeta en reposo, y hacia azul, verde, amarillo y rojo al moverse.
  - No cuesta nada en tiempo de ejecucion: sale en las mismas teselas y LUT.

## Parametros (Kconfig: *Fluid simulation*)

| Opcion | Por defecto | Efecto |
| --- | --- | --- |
| `FLUID_CELL_PX` | 10 | Es el mando principal. Celda de 8 px: unas 3.900 particulas y ~1,6x de CPU. Celda de 12 px: unas 1.700 particulas. |
| `FLUID_FILL_PERCENT` | 40 | Cuanto fluido hay. |
| `FLUID_SUBSTEPS` | 1 | Estabilidad frente a CPU. Con 2 se duplica el coste. |
| `FLUID_PRESSURE_ITERS` | 20 | Incompresibilidad. En el host, 20 se ve igual que 30. |
| `FLUID_GRAVITY_PX_PER_G` | 1800 | Escala del "tanque virtual" (~0,3 m). |
| `FLUID_BRIGHTNESS` | 80 | Brillo inicial del panel (comando 0x51). Despues manda el menu, guardado en NVS. |
| `FLUID_CLOCK_DEFAULT` | y | Digitos del reloj activados al primer arranque. |
| `FLUID_RTC_SET_FROM_BUILD` | n | Escribir la hora de compilacion en el RTC en cada arranque. |

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

### Reloj (ESP32-S3, estilo LED, celda 10 px, 2.497 particulas, 1 substep)

Del log `sim:` / `render:`:

| Version | Iteraciones | step_avg | fps | fluid_cells | internal_free | Notas |
| --- | --- | --- | --- | --- | --- | --- |
| Primera (un nucleo, `dt` fijo 1/60) | 30 | ~36,5 ms | 24 | ~860 | | El fluido iba a camara lenta (~40 % de su velocidad). |
| Dos nucleos, red-black, `dt` real | 20 | ~20 ms | 39-44 | ~520 | | Comprimido: la compensacion de deriva estaba en unidades erroneas. |
| Deriva corregida | 20 | ~22 ms | 36-37 | ~700 | | Volumen correcto en vertical. |
| Con reloj (digitos) | 20 | ~19,8 ms | 41 | ~705 | ~18 KB (min 13,5 KB) | sep 6,2 / prs 3,0 / g2p 3,8 / p2g 3,0 / col 2,1 ms; raster 2,6 ms. |

Por que el S3 tarda ~50 veces lo del host: el nucleo es in-order, con latencias de FPU y division por software. Es calculo, no memoria.

## Riesgos y mitigaciones

| Riesgo | Mitigacion |
| --- | --- |
| La simulacion no cabe en 16,7 ms en el S3 | Kconfig por tiers. Mover la separacion o la presion al core 0 si el render va sobrado. |
| Tearing (el panel refresca mientras llega una franja) | No hay TE conectado en el BSP. Probar TE en GPIO13 (comando 0x35) si se nota. |
| Poca memoria interna al bajar la celda | Asignacion con fallback a PSRAM y aviso en el log. Vigilar `internal_free`. LVGL asigna en PSRAM. |
| Temblor en reposo | Amortiguacion 0,5/s, histeresis de nivel y filtro temporal en el raster. |
| Fallo del IMU | Gravedad fija hacia abajo y contador `imu_err` en el log. |

## Siguientes pasos (fuera de la primera version)

- **Calibracion de bias**, plana sobre la mesa y guardada en NVS. Hace falta una API en core (`imu_service_set_bias`) y un tag nuevo. Solo corrige el offset del sensor, no la postura.
- **QSPI a 80 MHz** con configuracion IO propia en vez de `bsp_display_new`. Duplicaria el techo del bus si el panel lo aguanta.
- **Sincronizar con TE** contra el tearing.
- **Atenuar el brillo o dormir por inactividad**: fluido quieto y sin tacto durante N s.
- **Paredes sin tiron:** muestrear las paredes nuevas repartido en varios frames, para evitar los ~24 ms de cada cambio de minuto.
- **`watch_rtc` a core**, cuando otra app lo necesite.
- **Giroscopio:** inercia al girar el reloj en su plano.
- **Colores por particula** (mezcla de dos liquidos).
