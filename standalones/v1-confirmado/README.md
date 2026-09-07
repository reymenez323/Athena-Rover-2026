# Firmware ESP32-S3 AUTÓNOMO — v1-confirmado

Primera variante del firmware standalone (sin Raspberry Pi, sin cámara)
**confirmada funcionando de punta a punta en banco**: asegura la llave,
avanza hasta la zona amarilla, se detiene por completo, retrocede y la
deposita. Ver [`../v2-merodeo/`](../v2-merodeo/) para la variante que además
convierte la evasión de borde en una maniobra de retroceso + giro 180° +
merodeo.

**CONSERVAR TAL CUAL** — este archivo no se modifica salvo pedido explícito;
es el punto de referencia al que volver si una variante nueva rompe algo.

## Secuencia de misión

0. `ARRANQUE`: cuenta regresiva de `Mission::kStartupDelayMs` (3 s) para que
   el operador coloque la llave en la pinza abierta y ubique el robot.
1. `ASEGURAR_LLAVE`: cierra la pinza sobre la llave.
2. `BUSCAR_ZONA_NEUTRA`: avanza recto (sensor de color **trasero**) hasta
   pisar la zona amarilla. Si el borde negro se detecta en cualquier
   momento, se **retrocede al instante mientras siga detectándose** —
   evasión reactiva simple, sin maniobra de giro.
3. `DETENER_ZONA_NEUTRA`: full stop (`kFullStopMs`, 400 ms) antes de
   invertir el sentido de marcha.
4. `RETROCEDER_A_ZONA_NEUTRA`: retrocede `kRetrocesoZonaNeutraMs` (700 ms)
   para que el frente (y la llave) quede dentro de la zona segura.
5. `DEPOSITAR_LLAVE`: abre la pinza. Misión terminada.

No hay fase de búsqueda de bandera: el bus I2C nº0 (TCS34725 delantero +
VL53L1X) se retiró por completo — sin él no queda ningún sensor capaz de
detectar "hay algo delante". Ver el aviso grande al principio de
`src/main.cpp`.

## Hardware

- 2x L298N (4 motores de tracción)
- 1x PCA9685 (servo del gripper), bus I2C nº1 (`Wire1`)
- 1x TCS34725 (color, **trasero únicamente**), bus I2C nº1
- 2x QTRX-HD-01A (reflectancia, borde negro)
- LED RGB (indicador de zona de piso: amarillo/rojo/azul fijo, morado sobre
  el borde negro, apagado en piso neutro)
- Switch de 3 posiciones (selección de equipo)

Conexiones completas en
[`../../hardware/conexiones-esp32-s3.md`](../../hardware/conexiones-esp32-s3.md).

## Compilar y subir

```bash
pio run              # compilar
pio run -t upload    # subir al ESP32-S3
pio device monitor   # ver los logs de misión (fases, sensores, watchdog)
```

## Qué falta calibrar antes de una demo real

- `kDarkThreshold` de los QTR (reflectancia) — nunca se calibró contra el
  piso/luz reales; ver el log `[Reflect]` en consola.
- Los umbrales de `ClassifyColor()` para el TCS34725 trasero.
