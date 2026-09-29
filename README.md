# ESP32Watch-Fluid

Simulacion de fluido para la Waveshare **ESP32-S3-Touch-AMOLED-2.06**, al estilo de los colgantes de "matriz LED" con liquido. Es un FLIP/PIC en tiempo real que sigue la gravedad real medida por el IMU. Se pinta directamente al AMOLED, sin LVGL, y solo se envian las zonas que cambian.

## Controles

| Accion | Control |
| --- | --- |
| Mover el fluido | Inclinar o girar el reloj: cae hacia el "abajo" real. Al sacudirlo, salpica. |
| Remover | Tocar y arrastrar el dedo sobre la pantalla |
| Cambiar estilo (LED, Pixel, Liquid) | Pulsacion corta de `BOOT` |
| Cambiar paleta (agua, lava, toxic) | Pulsacion larga de `BOOT` (~0,7 s) |

- No hay calibracion: el reloj plano sobre la mesa deja el fluido flotando, sin gravedad, y en vertical cae al fondo.
- `PWR` no hace nada, porque es la pantalla raiz de la app. No mantenerlo unos 6 s: apaga la placa.

## Compilar y flashear

Requiere `ESP-IDF 5.5.4` (ver [SETUP](https://github.com/Sethyrus/ESP32Watch-core/blob/main/docs/SETUP.md)).

```sh
source "$HOME/.espressif/v5.5.4/esp-idf/export.sh"
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor   # p. ej. /dev/tty.usbmodem1101; sin -p lo autodetecta
```

La calidad y el coste se ajustan en `idf.py menuconfig` > *Fluid simulation*: tamano de celda, relleno, substeps, iteraciones, gravedad y brillo. Los valores que se queden deben ir a `sdkconfig.defaults`.

### Benchmark en el ordenador

El motor es C puro y tambien compila en el host. El benchmark tiene cuatro funciones:
- ejecuta un guion fijo: reposo, giro completo, sacudida, gravedad cero, de lado, dedo y reposo;
- comprueba que no se pierden particulas, que no hay NaN y que ninguna sale del contenedor;
- mide el coste de cada etapa y los bytes que cada estilo enviaria al panel;
- vuelca frames PPM para ver el resultado.

```sh
tools/host_bench/run.sh                           # valores por defecto
tools/host_bench/run.sh --cell 8 --iters 20 --out frames
```

## Estado

Primera version:
- funcionan la simulacion, los tres estilos, las paletas, el tacto y BOOT;
- en el host pasan las comprobaciones;
- las medidas en el reloj estan pendientes (ver [docs/FLUID_DESIGN.md](docs/FLUID_DESIGN.md)).

## Estructura

| Ruta | Contenido |
| --- | --- |
| `main/main.c` | Arranque. |
| `components/fluid_engine/` | Motor en C puro: simulacion FLIP (`fluid_sim`), rejilla LED (`fluid_raster`) y render por franjas con deteccion de cambios (`fluid_render`). |
| `components/fluid_app/` | Parte ESP32: pantalla sin LVGL, tareas (simulacion en el core 1, render y DMA en el core 0), IMU, tacto, BOOT y Kconfig. |
| `tools/host_bench/` | Benchmark y comprobaciones en el ordenador. |
| `docs/FLUID_DESIGN.md` | Analisis, decisiones, presupuesto de rendimiento, medidas y siguientes pasos. |

El IMU y los botones vienen del componente `watch_board` de [ESP32Watch-core](https://github.com/Sethyrus/ESP32Watch-core), donde tambien esta la documentacion de hardware.

## Creditos y licencia

- El metodo FLIP sigue el tutorial [Ten Minute Physics #18](https://matthias-research.github.io/pages/tenMinutePhysics/18-flip.html) de Matthias Mueller (MIT). Es una reimplementacion en C adaptada a esta pantalla.
- Inspirado en el [Fluid Simulation Pendant](https://mitxela.com/projects/fluid-pendant) de mitxela.

MIT. Ver [LICENSE](LICENSE).
