# Firmware ESP32-S3 AUTÓNOMO — v4-merodeo-color

Prueba de merodeo puro: avanza en línea recta, y al detectar el borde negro
con el QTR se detiene **de inmediato** y gira un ángulo aleatorio entre 60°
y 180° (dirección también aleatoria) para seguir dentro de la pista, sin
fin. **No agarra nada ni se detiene en ningún color** — a diferencia de
[`../v3-color-delantero/`](../v3-color-delantero/), aquí no hay gripper ni
misión con objetivo: es solo para validar merodeo + color sensado a la vez.

Parte de `v3-color-delantero` (sensores de color delantero+trasero,
prioridad al delantero, LED en vivo — sin tocar esos parámetros) y de
[`../v2-merodeo/`](../v2-merodeo/) (la técnica de rechazo de luz ambiente
del QTR, ya calibrada en banco).

## Comportamiento

- **LED RGB**: muestra en vivo el color que ve el sensor activo (delantero
  con prioridad, trasero de respaldo) — blanco=GRIS, amarillo=AMARILLO,
  rojo=ROJO, azul=AZUL, apagado=NEGRO o sin lectura. Exactamente igual que
  `v3-color-delantero`, sin cambios.
- **Merodeo**: avanza recto. Al ver el borde negro (QTR **derecho** —el
  izquierdo sigue roto, ver abajo), para de inmediato y gira. El ángulo
  (60°–180°) y la dirección (izquierda/derecha) se sortean cada vez, así
  que el patrón de merodeo no se repite siempre igual.
- Las dos lógicas son **independientes**: el color solo se muestra, nunca
  frena ni redirige al robot. Solo el QTR decide cuándo girar.

## QTR izquierdo: sigue roto

Confirmado en banco anoche (ver `firmware-esp32/src/main.cpp` y
`../v2-merodeo/`): el sensor izquierdo se queda pegado en 4095 sin importar
superficie ni estado del emisor. Se fuerza `left_on_line=false` — toda la
detección de borde depende del **derecho**, con la técnica de rechazo de
luz ambiente ya calibrada (`kBordeRestadoUmbral=40`).

## Sin maniobra de retroceso (a diferencia de v2-merodeo)

`v2-merodeo` hace un baile de 4 fases (parar, retroceder, parar, girar
180° siempre a la derecha) antes de retomar. Pedido explícito para esta
prueba: más simple — **para y gira nada más**, con ángulo y dirección
aleatorios en vez de siempre 180° a la derecha. La duración del giro se
calcula con la misma tasa calibrada anoche en `v2-merodeo`
(`kEvasionGiroMs=2000 ms` para 180°).

Se conservan dos seguridades de `v2-merodeo` que no costaban nada agregar
y ya resolvieron problemas reales en banco:
- **Debounce de 50 ms** sobre el borde (ruido de un instante no dispara el giro).
- **Gracia de 1 s** al retomar el merodeo (para no volver a dispararse
  contra el mismo borde del que recién se alejó).

## Sentido de giro de los motores

Mismo hallazgo que `v3-color-delantero`: la convención vieja
(`speed < 0 = avanza`, de `v1-confirmado`/`v2-merodeo`) ya no coincide con
este robot. Se usa `speed > 0 = avanza`, confirmado en banco hoy.

## Hardware necesario

- 2x L298N (4 motores)
- 2x TCS34725 — delantero (I2C bus 0) + trasero (I2C bus 1), **solo para
  el LED**, nada de gripper ni PCA9685 en esta variante
- 2x QTRX-HD-01A (solo el derecho funciona)
- LED RGB
- Switch de 3 posiciones (solo como traba de seguridad)

**NADA** de gripper/PCA9685/ToF activo en esta variante.

## Uso

```bash
pio run -t upload
pio device monitor
```

El robot espera con el switch en el centro (LED blanco tenue parpadeando).
Muévelo a cualquier posición para armarlo y que arranque la cuenta
regresiva de 3 s antes de empezar a merodear.
