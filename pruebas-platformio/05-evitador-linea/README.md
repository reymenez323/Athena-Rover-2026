# Prueba 05 — Evitador de línea (QTR con prioridad + color de respaldo)

Robot que se queda **siempre** dentro del rectángulo delimitado por cinta
negra. En cuanto detecta el borde: retrocede ~1.5 s en línea recta
(cronometrado) y después gira sobre su propio eje hacia un lado elegido
**al azar** (izquierda o derecha, 50/50) — reevaluando los sensores en
cada vuelta de `loop()` durante el giro, hasta que confirma que ya no hay
negro debajo. El giro en sí sigue sin temporizador fijo: dura lo que el
sensor diga.

## Dos tipos de sensor, con prioridad distinta (a propósito)

1. **QTR — con prioridad.** Los DOS (izquierdo GPIO1, derecho GPIO2),
   reflectancia, `analogRead()` en <1 ms. Decide SOLO: confirmado rápido
   (~20 ms), tanto para empezar a girar como para volver a avanzar. El
   color no necesita estar de acuerdo para nada de lo que decide el QTR.
2. **Color — de respaldo.** Los DOS TCS34725 (delantero I2C0, trasero
   I2C1), clasificados con la MISMA lógica de umbrales que
   [`../../calibracion-color/detector-tcs/`](../../calibracion-color/detector-tcs/)
   (`Umbral::`/`Clasificar()`, recalibrados contra 970 muestras reales —
   ver ese archivo para el detalle y las limitaciones, sobre todo con
   NEGRO). Menor prioridad que el QTR de tres formas:
   - Más lento (se muestrea cada 100 ms, no cada vuelta de `loop()`).
   - Exige MÁS lecturas seguidas para confiar (`kConfirmacionesColor` >
     `kConfirmacionesQtr`) — tarda más en convencerse.
   - Solo puede **iniciar** un giro si el QTR todavía no lo hizo (red de
     seguridad ante un QTR que falle o esté mal calibrado). **Nunca**
     decide cuándo volver a avanzar — esa decisión es 100% del QTR.

**Solo importa NEGRO.** Cualquier otra clasificación de color (AMARILLO,
ROJO, AZUL, GRIS) se ignora por completo para la máquina de estados — no
dispara nada, no evita nada. Sí se sigue mostrando en el LED RGB y la
consola, como diagnóstico.

> Antes esta prueba usaba un clasificador K-NN entrenado con el mismo
> dataset (ver `../../calibracion-color/generar_dataset_knn.py`, que sigue
> disponible si hace falta esa precisión extra más adelante). Se cambió a
> los umbrales de `detector-tcs/` para que las dos herramientas se
> comporten igual — el costo conocido es que el K-NN identificaba NEGRO
> bastante mejor (ver la comparación en `calibracion-color/README.md`).

## Máquina de estados

| Estado | Qué hace | Cómo sale |
|---|---|---|
| `DRIVING` | Avanza recto. | QTR confirma negro (rápido) **o** color confirma negro por su cuenta (lento, solo si el QTR no lo hizo ya) → `REVERSING`. |
| `REVERSING` | Retrocede en línea recta durante `kReverseMs` (~1.5 s), cronometrado, sin mirar los sensores. | Al cumplirse el tiempo, sortea un lado al azar (50/50) → `TURNING`. |
| `TURNING` | Gira en el sitio hacia el lado sorteado (un lado adelante, el otro atrás — **nunca** avanza ni retrocede). | SOLO cuando el QTR confirma que ya no hay negro → `DRIVING`. El color no participa en esta decisión. |

`kReverseMs` es el ÚNICO temporizador fijo del archivo — retroceder no
tiene una señal de sensor que diga "ya retrocediste lo suficiente", así
que es de lazo abierto (igual que en
[`01-mantente-en-cuadro`](../01-mantente-en-cuadro/README.md#el-giro)). El
giro que sigue no tiene ese problema: dura lo que el sensor diga, no un
número cronometrado.

El giro usa el máximo duty (`kTurnSpeed = 100`): un pivote en el sitio con
4 ruedas motrices necesita mucho torque (las 4 raspan contra el piso).

**El lado del giro (izquierda o derecha) se sortea al azar** con
`esp_random()` (generador de números aleatorios por hardware del
ESP32-S3, no una semilla fija) cada vez que termina la reversa — no
depende de por dónde entró la cinta ni de qué sensor disparó la detección.

## Compilar y subir

```bash
pio run -t upload -t monitor
```

El monitor usa el puerto **UART** del DevKit (`Serial0`/`DEBUG_LINK`) —
ver `platformio.ini`.

## Cableado

Los pines completos de `hardware/conexiones-esp32-s3.md`, en los sensores
que toca esta prueba: los 2 QTR (GPIO1/2) + control de emisores (GPIO42),
los 2 TCS34725 por I2C0 (GPIO8/9) e I2C1 (GPIO47/48) + sus LED (GPIO18/21),
los 2 L298N, el LED de equipo (GPIO40/41) y el LED RGB (GPIO39/38/3). No
requiere el PCA9685 — en todo lo demás es un subconjunto completo del
firmware final.

## Qué mirar si no funciona

- **Nunca detecta el borde**: abrir el monitor, ver la línea impresa cada
  200 ms (`izq=... der=... qtr=... del=... tra=... color_respaldo=...
  estado=...`). Si `del=?` y `tra=?` todo el tiempo, ningún TCS34725 está
  respondiendo — el robot sigue funcionando solo con el QTR (que tiene
  prioridad), pero pierde la red de seguridad del color.
- **El color nunca ayuda a detectar el borde**: normal en parte — NEGRO es
  la clase que peor detecta este clasificador de umbrales (~58% de
  acierto, ver `calibracion-color/detector-tcs/src/main.cpp`). El QTR
  sigue siendo quien debe cargar con la detección la mayoría del tiempo;
  el color es un respaldo, no el plan principal.
- **El giro no gira** (mismo problema que se vio en `01`): si a
  `kTurnSpeed=100` las ruedas no se mueven, ya no es de software — revisar
  tracción, batería de motores, o roce mecánico.
