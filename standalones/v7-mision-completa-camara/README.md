# Firmware ESP32-S3 — v7-mision-completa-camara

Copia exacta de [`v6-mision-completa`](../v6-mision-completa/) — **no se
tocó nada** de motores, color, ToF, gripper, ni ninguno de los tiempos —
más UNA cosa nueva: durante el segundo giro tras soltar la caja (el de
"recentrarse" hacia donde va a estar la bandera), la Raspberry Pi puede
avisarle al ESP32 que ya ve la bandera contraria, y el giro se corta ahí
mismo en vez de completar siempre el tiempo fijo.

Si la Raspberry Pi no está conectada, o no manda nada, este firmware se
comporta exactamente igual que v6 — el giro simplemente agota su tope de
tiempo de siempre. La Pi nunca puede impedir que la misión avance, solo
puede hacer que un giro termine antes.

## Qué es "la señal de cámara"

Un protocolo deliberadamente mínimo, sin relación con
[`docs/protocolo-serial.md`](../../docs/protocolo-serial.md) (el de
`firmware-esp32/`, con telemetría completa) ni con nada de
`raspberry-pi/src/athena/`: la Pi manda el byte `'V'` (ASCII) cada vez que
su detector ve la bandera del equipo contrario. Sin framing, sin
checksum, sin structs — cualquier otro byte se ignora. El ESP32 exige que
la señal esté **fresca de forma sostenida** (`kFrescoBanderaCamaraMs` /
`kSostenBanderaCamaraMs` en `Mission::`, ver `src/main.cpp`) antes de
confiar y cortar el giro, para no reaccionar a un byte suelto de ruido.

**Antes** del giro de recentrado (retroceso, primer giro de esquive,
avance) la señal puede estar llegando igual, pero el firmware **no la
usa para nada** — a propósito, pedido explícito: nada de la cámara debe
tocar los motores antes de esa fase.

## Conexión física

- **CAM_LINK = `Serial`, el puerto USB nativo del ESP32-S3.** La
  Raspberry Pi va conectada ahí, no al puerto UART.
- **DEBUG_LINK = `Serial0`, el puerto UART** — el mismo que se usa para
  flashear. `pio device monitor` funciona sin cambiar de cable.
- Estos son los MISMOS dos puertos que usa `firmware-esp32/` (el
  firmware real de competencia): nativo para la Pi, UART para
  flashear/depurar — así que si ya sabes cuál es cuál de esa parte del
  proyecto, es el mismo acá.

## Qué correr en la Raspberry Pi

Un script nuevo y deliberadamente simple, sin relación con
`run_rover.py` (que trae la máquina de estados completa y no aplica
acá): solo cámara + modelo de Edge Impulse + mandar `'V'` por serial.
Ver [`raspberry-pi/scripts/avisar_bandera_v7.py`](../../raspberry-pi/scripts/avisar_bandera_v7.py).

```bash
cd raspberry-pi
python3 scripts/avisar_bandera_v7.py --equipo rojo   # o azul -- debe coincidir con el switch físico del robot
```

## Uso

```bash
pio run -t upload --upload-port COM11   # o el puerto UART que corresponda
pio device monitor --port COM11         # log de depuración
```

El robot espera con el switch en el centro. Muévelo a ROJO o AZUL para
armarlo, elegir equipo y arrancar. Ten la caja/llave puesta bajo el
gripper desde antes de energizar — igual que v6.

## Si algo no anda

Todo lo que **no** sea "el segundo giro no se corta antes de tiempo" es
un problema de v6, no de esta variante — revisa primero ahí, ya que este
archivo es una copia byte a byte salvo por lo descrito arriba.
