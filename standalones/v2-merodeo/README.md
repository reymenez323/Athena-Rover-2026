# Firmware ESP32-S3 AUTÓNOMO — v2-merodeo

Parte de [`../v1-confirmado/`](../v1-confirmado/) (misma secuencia base, ya
confirmada en banco) y le cambia únicamente el manejo del borde negro
durante `BUSCAR_ZONA_NEUTRA`.

## Diferencia con v1-confirmado

En vez de retroceder al instante mientras el borde siga detectándose,
ejecuta una maniobra fija en cuatro tiempos y luego retoma la búsqueda:

1. `EVADIR_BORDE_DETENER_ANTES_RETROCESO`: detiene las cuatro ruedas (200 ms).
2. `EVADIR_BORDE_RETROCESO`: retrocede `kEvasionRetrocesoMs` (1500 ms).
3. `EVADIR_BORDE_DETENER_ANTES_GIRO`: vuelve a detener las cuatro ruedas
   (1000 ms).
4. `EVADIR_BORDE_GIRO`: gira ~180° sobre su propio eje durante
   `kEvasionGiroMs` (2000 ms), con ambos lados al 100 % y en sentidos opuestos.
5. Vuelve a `BUSCAR_ZONA_NEUTRA` y sigue merodeando hasta encontrar la zona
   amarilla.

Los tiempos de retroceso y giro recuperan la calibracion realizada sobre el
chasis real en `pruebas-platformio/01-mantente-en-cuadro` (commit `b435181`).

La detección del borde negro depende **exclusivamente de los QTR**. Las
lecturas `BLACK` del TCS34725 se ignoran tanto para la maniobra como para la
indicación morada del LED, evitando que un falso positivo del sensor de color
se confunda con un borde real. El TCS34725 se conserva para reconocer las
zonas amarilla, roja y azul.

El resto de la secuencia (asegurar la llave, detenerse al ver amarillo,
retroceder, depositar) es idéntico a `v1-confirmado` — no se tocó ese
mecanismo, ya confirmado funcionando en banco.

## Secuencia de misión

0. `ARRANQUE` → 1. `ASEGURAR_LLAVE` → 2. `BUSCAR_ZONA_NEUTRA` (con evasión
   de borde en dos fases, ver arriba) → 3. `DETENER_ZONA_NEUTRA` (full stop
   400 ms) → 4. `RETROCEDER_A_ZONA_NEUTRA` (700 ms) → 5. `DEPOSITAR_LLAVE`.

## Hardware

Igual que [`../v1-confirmado/README.md`](../v1-confirmado/README.md#hardware).

## Compilar y subir

```bash
pio run              # compilar
pio run -t upload    # subir al ESP32-S3
pio device monitor   # ver los logs de misión (fases, sensores, watchdog)
```

## Qué falta calibrar antes de una demo real

- `kDarkThreshold` de los QTR (reflectancia).
- Los umbrales de `ClassifyColor()` para el TCS34725 trasero.
- `kEvasionRetrocesoMs` / `kEvasionGiroMs` — cuánto retrocede y gira de
  verdad depende del peso del robot y la fricción de la pista; ajustar en
  banco si el giro queda corto/largo de 180°.
