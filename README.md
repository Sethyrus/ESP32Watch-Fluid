# ESP32Watch-Fluid

Simulacion de fluido para la Waveshare **ESP32-S3-Touch-AMOLED-2.06**, al estilo de los colgantes de "matriz LED" con liquido. Es un FLIP/PIC en tiempo real que sigue la gravedad real medida por el IMU. Se pinta directamente al AMOLED y solo se envian las zonas que cambian; LVGL solo se usa para el menu de ajustes.

Tambien es un reloj: los digitos de la hora son paredes, y el fluido fluye a su alrededor.

## Controles

| Accion | Control |
| --- | --- |
| Mover el fluido | Inclinar o girar el reloj: cae hacia el "abajo" real. Al sacudirlo, salpica. |
| Remover | Tocar y arrastrar el dedo sobre la pantalla |
| Abrir y cerrar el menu | Pulsacion corta de `PWR` |
| Cambiar estilo (LED, Pixel, Liquido) | Pulsacion corta de `BOOT` (atajo) |
| Cambiar paleta (Agua, Lava, Toxico) | Pulsacion larga de `BOOT`, ~0,7 s (atajo) |

**Menu (`PWR`):**
- Opciones: estilo, paleta, reloj si/no, ajustar hora, brillo, reiniciar el fluido y continuar.
- Arrancada desde el launcher, ademas `Salir al launcher` (guarda antes los ajustes).
- Se cierra con `Continuar`, `PWR` o `BOOT`.
- Mientras esta abierto, el fluido queda congelado y atenuado detras.
- Los ajustes se guardan en NVS y se conservan al reiniciar.

**Notas:**
- **Sin calibracion:** el reloj plano sobre la mesa deja el fluido flotando, sin gravedad, y en vertical cae al fondo.
- **Hora:** la guarda el RTC. Si la pierde (sin bateria), se pone la hora de compilacion; se ajusta desde el menu.
- **`PWR`:** no mantenerlo unos 6 s, porque apaga la placa.
- **Secreto:** hay un easter egg. Pista: agitalo con ganas.

## Compilar y flashear

Requiere `ESP-IDF 5.5.4` (ver [SETUP](https://github.com/Sethyrus/ESP32Watch-core/blob/main/docs/SETUP.md)).

```sh
source "$HOME/.espressif/v5.5.4/esp-idf/export.sh"
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor   # p. ej. /dev/tty.usbmodem1101; sin -p lo autodetecta
```

Para tenerla junto a las demas apps y elegirla desde un menu de arranque, grabarla con [ESP32Watch-Launcher](https://github.com/Sethyrus/ESP32Watch-Launcher) (`./flash_all.sh`). `partitions.csv` es la tabla comun del launcher; en standalone la app ocupa `factory`.

La calidad y el coste se ajustan en `idf.py menuconfig` > *Fluid simulation*: tamano de celda, relleno, substeps, iteraciones, gravedad, brillo inicial, reloj por defecto y RTC. Los valores que se queden deben ir a `sdkconfig.defaults`.

### Benchmark en el ordenador

El motor es C puro y tambien compila en el host. El benchmark tiene cuatro funciones:
- ejecuta un guion fijo: reposo, giro completo, sacudida, gravedad cero, de lado, dedo y reposo;
- comprueba que no se pierden particulas, que no hay NaN y que ninguna sale del contenedor;
- mide el coste de cada etapa y los bytes que cada estilo enviaria al panel;
- vuelca frames PPM para ver el resultado.

```sh
tools/host_bench/run.sh                           # valores por defecto
tools/host_bench/run.sh --cell 8 --iters 20 --out frames
tools/host_bench/run.sh --clock 1259 --splash 18 --palette 3   # reloj, salpicadura, arcoiris
```

## Estado

- **Funciona:** la simulacion, los tres estilos, las paletas, el tacto, BOOT, el menu, el reloj y el RTC.
- **Comprobaciones del host:** pasan, tambien con dos hilos y con TSan.
- **En el reloj:** ~41 fps con el reloj activado (ver medidas en [docs/FLUID_DESIGN.md](docs/FLUID_DESIGN.md)).

## Estructura

| Ruta | Contenido |
| --- | --- |
| `main/main.c` | Arranque. |
| `components/fluid_engine/` | Motor en C puro: simulacion FLIP con paredes (`fluid_sim`), digitos del reloj (`fluid_clock`), rejilla LED (`fluid_raster`) y render por franjas con deteccion de cambios (`fluid_render`). |
| `components/fluid_app/` | Parte ESP32: pantalla, tareas (simulacion en el core 1, render y DMA en el core 0), IMU, tacto, botones, NVS, menu LVGL (`fluid_menu`) y Kconfig. |
| `tools/host_bench/` | Benchmark y comprobaciones en el ordenador. |
| `docs/FLUID_DESIGN.md` | Analisis, decisiones, presupuesto de rendimiento, medidas y siguientes pasos. |

El IMU, los botones, el RTC y la init de NVS vienen del componente `watch_board` de [ESP32Watch-core](https://github.com/Sethyrus/ESP32Watch-core), donde tambien esta la documentacion de hardware.

## Creditos y licencia

- El metodo FLIP sigue el tutorial [Ten Minute Physics #18](https://matthias-research.github.io/pages/tenMinutePhysics/18-flip.html) de Matthias Mueller (MIT). Es una reimplementacion en C adaptada a esta pantalla.
- Inspirado en el [Fluid Simulation Pendant](https://mitxela.com/projects/fluid-pendant) de mitxela.

MIT. Ver [LICENSE](LICENSE).
