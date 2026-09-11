# Firmware ESP32-S3 AUTÓNOMO — v6-mision-completa

Integra en una sola secuencia lo ya validado por separado:
[v3-color-delantero](../v3-color-delantero/) (caja + sensor de color
delantero) y [v5-agarrar-bandera](../v5-agarrar-bandera/) (ToF + gripper).
Sin Raspberry Pi, sin cámara, sin QTR — el robot todavía no puede doblar
sobre las ruedas, así que **todo el recorrido es en línea recta**.

## Secuencia completa

0. **Switch de equipo** (ROJO/AZUL) — arma el robot y define cuál es la
   "zona enemiga" más adelante (el color opuesto al elegido).
1. **Asegura la caja/llave** — gripper a 128° (`kClawClosedLlaveDeg`, ver
   [pruebas-platformio/06-calibracion-gripper/](../../pruebas-platformio/06-calibracion-gripper/)).
   Se asume ya puesta bajo el gripper al arrancar, igual que v3.
2. Avanza recto hasta ver **AMARILLO** con el sensor de color delantero
   (zona de depósito) — full stop, y suelta la caja.
3. **Espera media** (6 s, ajustable) — pensada para que muevas la caja
   fuera del camino y pongas la **bandera** al frente del robot, a la
   vista del ToF.
4. Se acerca a la bandera guiado por el ToF — el mismo algoritmo de
   avance grueso → parar → medir → paso chico → confirmar rango 3 veces
   seguidas → cerrar gripper (65°) de v5-agarrar-bandera, con los mismos
   valores ya afinados en banco. **Sin giro** al agarrar — pedido
   explícito, el robot sigue derecho.
5. Avanza recto hasta ver el color de la **zona enemiga** (opuesto al
   equipo elegido en el switch) con el sensor delantero — full stop,
   espera, y suelta la bandera.
6. Terminado.

## Por qué el sensor de color va en el bus 1 (recableado)

Esta es la primera variante que necesita el ToF y el TCS34725 delantero
**a la vez** (v3 solo tenía color, v5 solo tenía ToF). El 2026-09-08 se
confirmó que los dos NO pueden compartir el bus 0x29 — ver
[`calibracion/tof/README.md`](../../calibracion/tof/README.md). El fix
fue separar los buses físicamente:

- **TCS34725 delantero**: bus I2C 1 (GPIO47/48), junto al PCA9685.
- **TCS34725 trasero**: desconectado (no se usa en esta variante).
- **VL53L1X (ToF)**: solo en el bus I2C 0 (GPIO8/9), sin reasignación.

Si en tu robot el TCS34725 delantero todavía está en el bus 0, hace falta
recablearlo al bus 1 antes de usar este firmware — si no, el ToF no va a
inicializar (mismo síntoma que se vio esa noche).

## El LED — qué significa cada color

Pediste que decidiera qué mostrar: hay dos tipos de información en juego
(qué color ve el sensor delantero, y en qué paso de la maniobra de
agarre de la bandera está el robot) y un solo LED. Así quedó repartido:

**Mientras el robot busca una zona por color** (buscando la zona amarilla
al principio, y buscando la zona enemiga al final) — el LED muestra el
**color que está viendo en vivo**, igual que hacía v3. Sirve para
confirmar a simple vista que el sensor está viendo lo que esperas antes
de que el robot decida algo:

| Color del LED | Significa |
|---|---|
| ⚪ Blanco tenue | GRIS/piso — no hay zona debajo todavía |
| 🟡 Amarillo | Ve la zona amarilla (depósito de la caja) |
| 🔴 Rojo | Ve rojo (zona enemiga, si tu equipo es azul) |
| 🔵 Azul | Ve azul (zona enemiga, si tu equipo es rojo) |
| ⚫ Apagado | Ve negro, o el sensor no tiene lectura válida |

**Durante el resto de la maniobra** (acercándose a la bandera con el ToF,
agarrándola, deteniéndose a soltar) — el LED muestra en qué paso está,
igual que hacía v5:

| Color del LED | Significa |
|---|---|
| ⚪ Blanco tenue fijo | Acercamiento grueso a la bandera (ToF) |
| 🟡 Amarillo parpadeando | Parado, midiendo/dando pasos chicos para afinar la distancia |
| 🟢 Verde fijo | Bandera agarrada / avanzando hacia la zona enemiga / deteniéndose a soltarla / terminado |
| 🔴 Rojo parpadeando | Falla: se agotaron los pasos de ajuste sin confirmar el rango de agarre |

Si alguna de las dos capas de información no te sirve o quieres otra
combinación (por ejemplo, mezclar ambas todo el tiempo), decímelo y lo
ajusto — quedó separado en `EstadoVisible` (ver `LedTask` en
`src/main.cpp`) para que sea fácil de cambiar.

## Tiempos usados (todos ajustables en `Mission::` en `src/main.cpp`)

| Constante | Valor | Para qué |
|---|---|---|
| `kStartupDelayMs` | 3 s | Ubicar el robot y la caja tras elegir equipo |
| `kGripperSettleCajaMs` | 400 ms | El servo llega a 128° sobre la caja |
| `kDelayTrasAsegurarCajaMs` | 600 ms | Margen extra para que el agarre quede firme antes de avanzar |
| `kFullStopAntesDeSoltarCajaMs` | 400 ms | Full stop antes de soltar la caja en la zona amarilla |
| `kGripperSettleAperturaCajaMs` | 400 ms | El servo llega a 0° (abierto) |
| `kEsperaReacomodoMs` | 6 s | Tiempo para mover la caja y poner la bandera |
| `kEsperaTrasAgarrarBanderaMs` | 600 ms | Respiro tras cerrar el gripper sobre la bandera, antes de seguir derecho |
| `kFullStopZonaEnemigaMs` | 700 ms | Full stop + espera antes de soltar la bandera |
| `kGripperSettleSueltaBanderaMs` | 400 ms | El servo llega a 0° (abierto) |

Los tiempos del acercamiento ToF (`kDistanciaAproximacionMm`,
`kRangoAgarreMinMm/MaxMm`, `kVelocidadPaso`, `kPasoDuracionMs`, etc.) son
los mismos ya afinados en banco por v5-agarrar-bandera — no se tocaron.

## El ToF no sirve para la caja

Confirmado por el equipo: el ToF está montado a una altura que nunca va a
poder sensar la caja. Por eso la caja se agarra al arrancar (se asume ya
puesta) y se suelta por **color** (zona amarilla), nunca por distancia —
el ToF solo entra en juego para la bandera, más adelante, cuando ya no
hay caja de por medio.

## Hardware necesario

- 2x L298N (4 motores)
- 1x VL53L1X — bus I2C 0 (GPIO8/9), solo
- 1x TCS34725 delantero — bus I2C 1 (GPIO47/48), junto al PCA9685
- 1x PCA9685 + el servo del gripper — bus I2C 1 (GPIO47/48)
- LED RGB
- Switch de 3 posiciones (elige equipo Y arma el robot)

**NADA** de QTR, cámara, ni TCS34725 trasero (desconectado).

## Uso

```bash
pio run -t upload
pio device monitor
```

El robot espera con el switch en el centro. Muévelo a ROJO o AZUL para
armarlo, elegir equipo y arrancar la cuenta regresiva de 3 s. Ten la
caja/llave puesta bajo el gripper desde antes de energizar.
