# Raspberry Pi 4B — visión y decisión

La Raspberry Pi es el cerebro: mira por la cámara USB, decide qué hacer, y le
manda órdenes al ESP32-S3 por USB. **El ESP32 no piensa, solo ejecuta; la Pi
no toca hardware, solo decide.** El contrato entre ambos está en
[`docs/protocolo-serial.md`](../docs/protocolo-serial.md).

## Qué hace la cámara, y qué NO

Esto es lo primero que hay que entender, porque casi todo lo demás se deduce
de acá:

| | Resuelto por |
|---|---|
| Encontrar la **bandera del oponente** | La cámara + el modelo de Edge Impulse. Es lo **único** que hace la cámara. |
| Soltar la **llave** en la zona neutra | El sensor de **color** del ESP32 al ver amarillo. Nunca dependió de la cámara. |
| Saber que llegó a la **zona propia** | El sensor de **color** al ver el rojo/azul del equipo. |
| No **salirse de la pista** | Los sensores de **reflectancia**, que separan borde negro de fondo gris. |
| Cuándo **cerrar la pinza** | El **ToF** del ESP32: mide de verdad, no estima. |

Por eso cambiar el detector de bandera no rompe el resto de la misión.

> **El entrenamiento del modelo se hizo FUERA de este repo**, en Edge Impulse
> Studio (cuenta de Montse). Acá no hay scripts de captura ni de
> entrenamiento: se borraron porque nunca llegaron a usarse. Lo único que vive
> acá es el `.eim` ya exportado y el código que lo ejecuta. El historial
> completo de ese trabajo está en
> [`docs/handoff-vision-edge-impulse.md`](../docs/handoff-vision-edge-impulse.md).

## Cómo está organizado

```
raspberry-pi/
├── src/athena/
│   ├── protocol.py         contrato binario con el ESP32 (¡espejo de main.cpp!)
│   ├── link.py             enlace serial: reconexión y descubrimiento de puerto
│   ├── camera.py           captura en hilo aparte, descartando frames viejos
│   ├── ei_flag_detector.py el modelo de Edge Impulse -> cajas de banderas
│   ├── centering.py        línea central + zona muerta -> giro proporcional
│   ├── decision.py         máquina de estados de la misión
│   ├── config.py           parámetros ajustables
│   └── types.py            tipos comunes de percepción
├── scripts/
│   ├── run_rover.py            EL PROGRAMA DE COMPETENCIA
│   └── run_flag_tracker_ei.py  herramienta para calibrar el seguimiento a ojo
├── models/                 athena_ei_banderas.eim (el modelo entrenado)
├── config/                 rover.json de ESTA Pi (no se versiona)
├── deploy/                 arranque automático con systemd
└── tests/                  61 tests, ninguno necesita hardware
```

## El sistema: Raspberry Pi OS de 64 bits

**Los 64 bits no son opcionales.** El modelo `.eim` no es un archivo de datos:
es un **ejecutable ELF compilado para AArch64** (así se exporta de Edge
Impulse). En un Raspberry Pi OS de 32 bits no corre.

Y falla de forma engañosa: `requirements.txt` marca `edge_impulse_linux` con
`platform_machine == "aarch64"`, así que en 32 bits **pip lo salta en
silencio** y el error que ves es "falta el SDK" — te manda a instalar algo que
tampoco iba a funcionar. Por eso `ei_flag_detector.py` comprueba la
arquitectura y lo dice claro.

En Raspberry Pi Imager, elegí **"Raspberry Pi OS (64-bit)"**. Para verificarlo
en una Pi ya instalada:

```bash
uname -m        # tiene que decir aarch64. Si dice armv7l, es de 32 bits.
```

## Instalación

```bash
sudo apt install python3-opencv python3-numpy python3-serial
sudo apt install libatlas-base-dev libportaudio0 libportaudio2 libportaudiocpp0 portaudio19-dev
pip install --break-system-packages edge_impulse_linux
chmod +x models/athena_ei_banderas.eim
sudo usermod -aG dialout $USER     # después: cerrar sesión y volver a entrar
```

Cuatro cosas de esa lista que valen una explicación:

- **`apt` para OpenCV y NumPy**, no `pip`: los paquetes del sistema vienen
  compilados con las optimizaciones NEON del ARM. Los de `pip` son ruedas
  genéricas y corren bastante más lento.
- **`--break-system-packages`** hace falta desde Bookworm: Raspberry Pi OS
  marca su Python como *externally managed* (PEP 668) y sin ese flag `pip`
  se niega a instalar nada.
- **`chmod +x` en el `.eim`**: es un binario y al clonar el repo llega sin
  permiso de ejecución. Es el error más fácil de cometer en una Pi nueva.
- **El grupo `dialout`** es el que permite abrir `/dev/ttyACM*` y
  `/dev/ttyUSB*`. En Bookworm ya no existe el usuario `pi` por defecto —cada
  quien crea el suyo al instalar— y un usuario nuevo puede quedar fuera. Sin
  esto el robot arranca, no da error visible y **se queda mudo**: `link.py`
  detecta ese caso concreto y lo avisa en el log, pero es más rápido no
  tropezarse.

## Uso

```bash
# Ver que todo funciona, sin mover motores
python3 scripts/run_rover.py --equipo rojo --simular --ver

# En competencia
python3 scripts/run_rover.py --equipo rojo
```

Con `--simular` los motores no se mueven, pero **la señal de bandera sí se
manda**: es una luz, no un movimiento, así que se puede verificar el LED sobre
la mesa sin que el robot ruede.

Para ajustar a ojo la zona muerta y la ganancia de giro sin correr la misión
completa cada vez:

```bash
python3 scripts/run_flag_tracker_ei.py --equipo rojo --ver
```

> No uses ese script en competencia: no deposita la llave, y arrancarlo solo
> pierde la ronda de inmediato según el reglamento.

## El puerto serial

`config.py` trae `serial_port = "auto"`, que prueba en orden `/dev/ttyACM0`,
`/dev/ttyACM1`, `/dev/ttyUSB0`, `/dev/ttyUSB1` y se queda con el primero que
abra. No es capricho: el ESP32 aparece como **ttyACM** por su USB nativo y
como **ttyUSB** por el puerto de programación, y en la práctica cambió entre
sesiones de trabajo. Si querés forzar uno, ponelo explícito en
`config/rover.json`.

El enlace también se **reconecta solo**: cuando el ESP32 se reinicia, su
dispositivo USB desaparece y vuelve a aparecer. Un programa que abra el puerto
una sola vez al arrancar se queda mudo para siempre después del primer reset —
y eso pasa cada vez que se reprograma el firmware entre rondas.

---

## Cómo se decide qué mandarle al ESP32

`decision.py` es una máquina de estados que sigue el orden del reglamento, con
las prioridades muy claras:

1. **No salirse de la pista.** Sacar dos ruedas pierde la ronda de inmediato.
   La evasión del borde negro pisa cualquier otra decisión, aunque la bandera
   esté justo delante.
2. **No adelantar la secuencia.** Buscar la bandera antes de depositar la llave
   también descalifica. La máquina no puede pasar a buscar hasta que la llave
   esté depositada, y hay un test que lo verifica.
3. **Cumplir la misión.**

```
INICIO → BUSCAR_ZONA_NEUTRA → DEPOSITAR_LLAVE → BUSCAR_BANDERA
       → APROXIMAR_BANDERA → AGARRAR_BANDERA → RETORNAR_A_ZONA
       → ENTREGAR → TERMINADO
```

`DecisionMaker.step()` es una **función pura** de sus entradas: no toca el
puerto serial ni guarda estado escondido. Por eso toda la lógica de misión se
prueba sin robot, sin cámara y sin ESP32.

### Dos controles de giro, a propósito

- `decision._perseguir` corrige sobre el **ángulo en grados**, derivado del
  modelo pinhole. Lo usa toda la máquina de estados y es lo que prueban los
  tests.
- `centering.calcular_giro` corrige sobre el **error normalizado** (−1..1)
  medido directamente contra la caja que devolvió el modelo, con una zona
  muerta central.

En la fase de perseguir la bandera, `run_rover.py` reemplaza el primero por el
segundo: es más directo trabajar sobre la caja real que sobre un ángulo
derivado de una focal que todavía no está calibrada. El de grados se conserva
porque es el que la máquina de estados usa para decidir si está *centrada*.

### Señalizar la bandera

Cuando el modelo ve la bandera contraria, `decision.py` marca
`Commands.bandera_a_la_vista` y `run_rover.py` manda `CMD_FLAG_SIGNAL` al
ESP32, que hace destellar el LED alternando blanco con el color de equipo (así
ninguna de las dos señales tapa a la otra). Es el reto de demostración
"detectar la bandera del oponente y señalizar su detección".

Se marca **en cuanto se ve**, sin esperar a ninguna fase. Lo que descalifica
según el reglamento es *ir a buscar* la bandera antes de depositar la llave, no
verla de pasada — y de ir o no ir se encarga la máquina de estados, no esa
señal. El comando se manda **solo cuando el estado cambia**, no en cada cuadro.

---

## Tests

```bash
python3 -m pytest tests/ -q
```

61 tests, ninguno necesita hardware. Los más importantes son los de contrato:
abren el `main.cpp` **real** de los dos firmwares, extraen cada tipo de
paquete, cada largo y cada enum compartido, y los comparan con Python **en las
dos direcciones**. Agregar algo en un solo lado rompe el test.

Eso importa porque ya pasó de verdad: `CMD_FLAG_SIGNAL` existió varios commits
solo en el firmware y la Pi nunca supo mandarlo, así que el LED nunca
señalizó nada. El test de entonces enumeraba los paquetes a mano y no lo notó.

---

## Lo que falta

- **El retorno a la zona propia.** El robot avanza recto y solo reconoce su
  zona cuando ya pisa la línea de color. Funciona si quedó orientado hacia su
  lado, pero **no se recupera si quedó girado**. Necesita odometría (encoders)
  o reconocer visualmente la línea. Es el hueco más grande de la lógica de
  misión, y está marcado como TODO en el código.
- **Calibrar la focal** de la cámara (`GeometryConfig.focal_px` es hoy una
  estimación de webcam genérica). Afecta la distancia estimada, no el cierre
  de la pinza — de eso se encarga el ToF.
