# Protocolo serial ESP32-S3 ↔ Raspberry Pi

Este es **el idioma común** entre las dos mitades del robot: la que ve
(Raspberry Pi, cámara) y la que actúa (ESP32-S3, motores y sensores). Es el
documento de referencia; las dos implementaciones tienen que coincidir con lo
que dice acá.

| Lado | Archivo |
|---|---|
| ESP32-S3 (C++) | [`firmware-esp32/src/main.cpp`](../firmware-esp32/src/main.cpp), sección `[3]` |
| Raspberry Pi (Python) | [`raspberry-pi/src/athena/protocol.py`](../raspberry-pi/src/athena/protocol.py) |
| Guardia automática | [`raspberry-pi/tests/test_protocol.py`](../raspberry-pi/tests/test_protocol.py) |

> **La guardia no es decorativa.** Los tests abren el `main.cpp` real, extraen
> cada tipo de paquete, cada largo y cada enum compartido, y los comparan con
> Python **en las dos direcciones**. Agregar algo en un solo lado rompe el test.
> Eso importa porque ya pasó: `CMD_FLAG_SIGNAL` existió varios commits solo en
> el firmware, y la Raspberry Pi nunca supo mandarlo, así que el LED jamás
> señalizó la bandera. El test de entonces enumeraba los paquetes a mano y no
> lo notó. El de ahora sí.

## Formato de trama

```
[0xAA] [TYPE] [LEN] [PAYLOAD ... LEN bytes] [CHECKSUM]
```

- `0xAA` — byte de inicio.
- `TYPE` — 1 byte, ver las tablas de abajo.
- `LEN` — 1 byte, tamaño del payload (máximo 32).
- `CHECKSUM` — 1 byte: XOR de `TYPE`, `LEN` y todos los bytes del payload.

Todo entero de varios bytes va en **little-endian**. Los booleanos viajan
**empacados en bits** de un solo byte.

### Por qué byte a byte y no un `struct` con `memcpy`

Es la trampa clásica: el compilador de C++ inserta *padding* invisible entre
los campos de un struct según sus reglas de alineación, y ese padding no tiene
por qué coincidir con lo que Python asuma del otro lado. El síntoma sería
telemetría con valores absurdos en un sitio donde nadie mira. Serializando a
mano, el contrato queda fijo y verificable.

## Raspberry Pi → ESP32 (comandos)

| Código | Nombre | Len | Payload |
|:---:|---|:---:|---|
| `0x01` | `CMD_MOTOR` | 3 | `[0]` modo (0=STOP, 1=DRIVE) · `[1]` izquierda `i8` · `[2]` derecha `i8`, en % (−100..100) |
| `0x02` | `CMD_GRIPPER` | 1 | `[0]` acción (0=OPEN, 1=CLOSE_LLAVE, 2=CLOSE_BANDERA) |
| `0x03` | `CMD_LED` | 1 | `[0]` equipo (0=NONE, 1=RED, 2=BLUE) |
| `0x04` | `CMD_FLAG_SIGNAL` | 1 | `[0]` 1 = la cámara ve AHORA la bandera contraria, 0 = no |

**`CMD_GRIPPER` tiene tres acciones, no cinco.** El robot tiene **un solo
servo**: agarra o suelta, no sube ni baja nada. Hay dos cierres distintos
porque la llave (cubo) y el asta cilíndrica de la bandera tienen grosores
distintos, y los ángulos están calibrados por separado en
[`pruebas-platformio/06-calibracion-gripper/`](../pruebas-platformio/06-calibracion-gripper/).

**`CMD_FLAG_SIGNAL` es el puente del reto de demostración** "detectar la
bandera del oponente y señalizar su detección". El ESP32 no tiene cámara: sin
este comando no hay forma de que sepa que hay algo que señalizar. La Raspberry
Pi lo manda **solo cuando el estado cambia**, no en cada cuadro; el firmware lo
guarda en una cola de un elemento sobrescribible, así que si el enlace se cae,
el LED vuelve solo al color de equipo fijo en vez de quedarse destellando.

## ESP32 → Raspberry Pi (telemetría)

| Código | Nombre | Len | Payload |
|:---:|---|:---:|---|
| `0x10` | `TLM_COLOR` | 7 | `[0..3]` timestamp_ms `u32` · `[4]` color delantero · `[5]` color trasero · `[6]` flags: bit0 delantero válido, bit1 trasero válido |
| `0x11` | `TLM_REFLECT` | 9 | `[0..3]` timestamp_ms `u32` · `[4..5]` izq. crudo `u16` · `[6..7]` der. crudo `u16` · `[8]` flags: bit0 izq. sobre línea, bit1 der. sobre línea |
| `0x12` | `TLM_HEALTH` | 5 | `[0..3]` timestamp_ms `u32` · `[4]` bitmask de tareas colgadas |
| `0x13` | `TLM_TOF` | 7 | `[0..3]` timestamp_ms `u32` · `[4..5]` distancia_mm `u16` · `[6]` flags: bit0 válido |

### Valores de `ColorLabel` (en `TLM_COLOR`)

| Valor | Etiqueta | Qué es en la pista |
|:---:|---|---|
| 0 | `UNKNOWN` | sin lectura confiable |
| 1 | `BLACK` | cinta del borde — salirse pierde la ronda |
| 2 | `YELLOW` | zona neutra, donde va la llave |
| 3 | `RED` | línea de la zona roja |
| 4 | `BLUE` | línea de la zona azul |
| 5 | `FLOOR` | tapete gris, el fondo normal de la pista |

### Bitmask de `TLM_HEALTH`

Bit *i* encendido = la tarea *i* del ESP32 dejó de dar señales de vida. El
orden es el del `enum class TaskId` del firmware:

`0` serial_comm · `1` motor_control · `2` gripper_control · `3` color_sensor ·
`4` reflectance · `5` led_status · `6` tof_sensor

El ESP32 **reporta, no se reinicia**. En plena ronda un reinicio significa
motores parados, pinza suelta (se cae la bandera) y varios segundos de
arranque: perder la ronda. La Pi recibe el aviso, lo registra y sigue
compitiendo con lo que quede vivo.

## Qué sensor resuelve qué

Esto es lo que da sentido a la tabla de arriba. Cada reto de la demostración
se apoya en un sensor concreto, y ninguno hace el trabajo de otro:

| Reto | Sensor | Cómo llega a la lógica |
|---|---|---|
| Distinguir el borde negro del fondo gris | **Reflectancia (QTR ×2)** | `TLM_REFLECT` → prioridad absoluta en `decision.py`: si ve borde, retrocede pase lo que pase |
| Identificar las zonas de color (negro, amarillo, rojo, azul) | **Sensores de color (TCS34725 ×2)** | `TLM_COLOR` → dispara soltar la llave (amarillo) y terminar la misión (color propio) |
| Saber cuándo cerrar la pinza sobre la bandera | **ToF delantero (VL53L1X)** | `TLM_TOF` → medición física real, más confiable de cerca que estimar por tamaño en la imagen |
| Detectar la bandera del oponente y señalizarla | **Cámara USB + modelo de Edge Impulse** | Solo la Pi la ve → `CMD_FLAG_SIGNAL` → el LED del ESP32 destella |
| Cargar y depositar la llave | **Un servo de gripper** | `CMD_GRIPPER` con `CLOSE_LLAVE` / `OPEN` |

## Robustez del enlace

Las dos implementaciones son máquinas de estados incrementales, no lectores de
tramas completas: un puerto serie entrega los bytes partidos donde le da la
gana.

- **Un byte suelto o un checksum malo cuesta esa trama y nada más.** El parser
  vuelve a buscar el byte de inicio y sigue. Nunca se queda trabado.
- **Cada lado valida el byte de tipo contra lo que puede recibir**
  (`IsReceivableType` en C++, `RECEIVABLE_TYPES` en Python). Sin eso, un `0xAA`
  de basura justo antes de una trama válida se tragaría esa trama.
- **La Pi nunca acepta comandos** y **el ESP32 nunca acepta telemetría**: cada
  dirección solo reconoce lo que le corresponde.
- **Los motores tienen failsafe local:** si `MotorTask` pasa 500 ms sin un
  comando válido, frena por su cuenta. La lógica está *dentro* de esa tarea a
  propósito: así el robot frena aunque el problema esté en la tarea de
  comunicación, en el cable USB o en la propia Raspberry Pi.

## Cómo agregar un paquete nuevo

1. Agregarlo al `namespace Proto` de `firmware-esp32/src/main.cpp` (código,
   `LEN_`, y la lista de `IsReceivableType` si es un comando).
2. Manejarlo en `SerialDispatch` (comando) o emitirlo desde `SerialTask`
   (telemetría).
3. Agregar el mismo código y largo a `protocol.py`, más su `encode_*` o su
   rama en `_decode_payload`.
4. Exponerlo en `link.py` como un método `send_*`.
5. Correr `python3 -m pytest tests/ -q` desde `raspberry-pi/`. Si te saltaste
   un paso, el test te lo dice por nombre.
6. Actualizar la tabla de este documento.
