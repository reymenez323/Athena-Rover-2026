# 09 — ToF + cámara + gripper, aislado

Diagnóstico puntual: en la primera corrida completa de
[`standalones/v7-mision-completa-camara/`](../../standalones/v7-mision-completa-camara/)
el gripper nunca cerró sobre la bandera. Este banco quita de en medio TODO
lo demás (motores, color, las fases previas de la misión) para poder
probar únicamente ToF + PCA9685 (gripper) + la señal de cámara de la
Raspberry Pi, acercando la bandera a mano.

## Qué hace

Corre solo, sin comandos: mide distancia con el ToF todo el tiempo, y en
cuanto la Raspberry Pi confirma (byte `'V'` por `CAM_LINK`, mismo
protocolo que `v7`) que ve la bandera contraria **y** el ToF lleva 3
lecturas seguidas en rango de agarre (56-60mm), **cierra el gripper
solo**. Al salir de rango se rearma para poder repetir la prueba sin
reiniciar nada.

## Cómo usarlo para diagnosticar

| Lo que ves | Lo que significa |
|---|---|
| Cierra bien (solo o con `'C'` a mano) | El ToF/PCA9685 están bien — el problema de `v7` es que la misión nunca llegó de verdad a la fase de agarre. Revisa el log de `v7` por UART para ver en qué fase se quedó. |
| No cierra ni a mano con `'C'`, pero dice "PCA9685 listo" | Mecánico: el servo no tiene fuerza, algo lo traba, o el cableado de señal/alimentación del servo está mal. |
| Dice "PCA9685 no responde" | Bus I2C 1 — cableado SDA/SCL o el PCA9685 sin alimentación real. |
| Dice "VL53L1X no responde" | Bus I2C 0 — mismo tipo de problema, del lado del ToF. |

**Prueba de consumo eléctrico:** corre esto primero con el robot quieto
(motores conectados pero sin girar). Si funciona bien así, y en `v7`
fallaba con los motores en movimiento, es que los motores bajan el
voltaje del riel compartido lo suficiente como para que el ToF o el
PCA9685 fallen — no un cable suelto.

## Comandos (por el puerto UART, `DEBUG_LINK`)

| Comando | Efecto |
|---|---|
| `O` | Abre el gripper a mano |
| `C` | Cierra el gripper a mano (mismo ángulo que el cierre automático) |
| `R` | Rearma el cierre automático sin tener que abrir primero |

## Qué correr en la Raspberry Pi

El mismo script de siempre, sin cambios:

```bash
cd raspberry-pi
python3 scripts/avisar_bandera_v7.py --equipo rojo   # o azul
```

## Conexiones

Igual que `v7-mision-completa-camara/`: `CAM_LINK` = puerto USB nativo
(Raspberry Pi), `DEBUG_LINK` = puerto UART (flasheo y consola).
