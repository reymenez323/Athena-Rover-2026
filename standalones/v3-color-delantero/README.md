# Firmware ESP32-S3 AUTÓNOMO — v3-color-delantero

Prueba acotada para validar el sensor de color **DELANTERO**, recién
recalibrado (ver [`calibracion/color/`](../../calibracion/color/)), en una
misión real y no solo en banco suelto. Parte de
[`../v1-confirmado/`](../v1-confirmado/) pero con el sensor delantero de
vuelta (v1/v2 lo habían retirado) como prioridad, y **sin QTR ni ToF** —
alcance acotado a propósito para esta prueba.

## Secuencia de misión

0. `ARRANQUE`: cuenta regresiva de 3 s para cargar la llave y ubicar el robot.
1. `ASEGURAR_LLAVE`: cierra la pinza sobre la llave.
2. `BUSCAR_ZONA_NEUTRA`: avanza **en línea recta** (sin QTR, sin evasión de
   borde) hasta que el color activo (ver abajo) sea AMARILLO.
3. `DETENER_ZONA_NEUTRA`: full stop (400 ms) antes de soltar — sin
   retroceso, ver el porqué más abajo.
4. `DEPOSITAR_LLAVE`: abre la pinza. Misión terminada.

Durante **todo** el recorrido (no solo mientras busca la zona), el LED RGB
muestra en vivo qué color está viendo el sensor activo:

| Color detectado | LED |
|---|---|
| GRIS / piso | Blanco |
| AMARILLO | Amarillo |
| ROJO | Rojo |
| AZUL | Azul |
| NEGRO (o sin lectura de ningún sensor) | Apagado |

## Por qué el sensor DELANTERO y no solo el trasero (como v1/v2)

v1-confirmado/v2-merodeo retiraron el bus I2C nº0 (TCS34725 delantero +
VL53L1X) porque nunca dieron una conexión física confiable en banco. El 28
de agosto se calibraron 9 umbrales contra 970 muestras del delantero con
~14% de error; probado el 2026-09-07 bajo luz de oficina + sol directo,
AMARILLO clasificaba GRIS la mayoría de las veces — no porque el sensor
estuviera dañado (ROJO y AZUL seguían leyendo sólido), sino porque
`AMARILLO_G_MIN` (0.350) quedó justo en medio del rango real de g/c del
delantero bajo esa luz (0.328–0.357). Se recapturó AMARILLO (480 muestras,
8 puntos) y se reajustó ESE único umbral a 0.200 — ver
[`../../calibracion/color/detector-tcs/src/main.cpp`](../../calibracion/color/detector-tcs/src/main.cpp)
para el detalle completo, con matriz de confusión.

**Esta variante existe para confirmar que ese ajuste sirve de verdad en una
misión real**, no solo en banco suelto — y por eso reintroduce el bus I2C
nº0 que v1/v2 habían retirado.

## Prioridad: delantero sobre trasero

Se leen los dos sensores cada ciclo. Para el LED y para decidir "¿ya llegué
a la zona amarilla?", manda el **delantero** si dio lectura válida ese
ciclo; solo si no respondió se cae al trasero. El trasero usa el MISMO
juego de umbrales — nunca se caracterizó por separado, así que su
clasificación aquí es un respaldo de mejor esfuerzo, no algo validado.

## Sin maniobra de retroceso

v1-confirmado necesitaba retroceder tras detectar amarillo porque su único
sensor (trasero) ya había pasado la zona en el instante en que la veía — la
llave, al frente, quedaba más allá. Con el sensor **delantero** como
prioridad, el sensor que manda está literalmente donde está la llave: en
cuanto ve amarillo, el frente del robot ya está sobre la zona. Solo hace
falta un full stop antes de soltar, no un retroceso.

## Alcance deliberadamente acotado

Pedido explícito para esta prueba: **solo sensores de color + gripper +
motores/drivers para avanzar recto**. Nada de QTR (sin `ReflectanceTask`,
sin evasión de borde) ni de ToF (el VL53L1X, si sigue físicamente
conectado, se mantiene en reset todo el tiempo). Si esto funciona bien, el
siguiente paso natural es sumar los QTR — ver
[`../v1-confirmado/`](../v1-confirmado/)/[`../v2-merodeo/`](../v2-merodeo/)
para cómo se hizo antes.

## Switch de equipo: solo traba de seguridad

Esta prueba no depende del equipo elegido para nada (no busca ninguna
bandera), pero el switch físico de 3 posiciones ya está cableado y sigue
actuando como traba de seguridad: nada se mueve mientras esté en el centro.
Cualquiera de los dos tiros arma el robot (no importa cuál).

## Hardware necesario

- 2x L298N (4 motores)
- 1x PCA9685 (I2C, bus 1) — 1 servo del gripper
- 2x TCS34725 — delantero (I2C bus 0) + trasero (I2C bus 1)
- LED RGB
- Switch de 3 posiciones (solo como traba de seguridad)

**NADA** de QTR ni ToF activo para esta prueba.

## Uso

```bash
pio run -t upload
pio device monitor
```

El robot espera con el switch en el centro (LED blanco tenue parpadeando).
Muévelo a cualquier posición para armarlo y que arranque la cuenta
regresiva de 3 s.
