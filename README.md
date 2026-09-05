# Athena Rover 2026 — Retos del Rover H07

Proyecto universitario del Instituto Tecnológico de Santo Domingo (INTEC) —
Ingeniería Mecatrónica.

**Equipo:** Reymildo (modelado, firmware ESP32, integración) · Montse (visión,
cámara y modelo de clasificación).

Robot autónomo para el reto **"Retos del Rover – H07"**: dos robots se
enfrentan en una pista; cada uno debe depositar una llave en la zona neutra y
después identificar, capturar y trasladar la bandera del oponente hasta su
propia zona segura.

---

## Cómo está partido el robot

El robot son dos mitades que se hablan por un cable USB. Cada una hace lo que
la otra no puede:

```
        ┌──────────────────────────┐        ┌─────────────────────────────┐
        │      RASPBERRY PI 4B     │        │         ESP32-S3            │
        │  "la que ve y decide"    │        │   "la que siente y actúa"   │
        ├──────────────────────────┤        ├─────────────────────────────┤
        │ Cámara USB               │        │ 4 motores (2x L298N)        │
        │ Modelo Edge Impulse      │  USB   │ 1 servo de gripper (PCA9685)│
        │   -> ¿dónde está la      │ <────> │ 2 sensores de color         │
        │      bandera rival?      │ serial │ 2 sensores de reflectancia  │
        │ Máquina de estados de    │ binario│ 1 ToF delantero             │
        │   la misión              │        │ LED RGB de equipo           │
        └──────────────────────────┘        └─────────────────────────────┘
```

**El ESP32 no piensa, y la Pi no toca hardware.** Lo único que los une es el
protocolo binario documentado en
[`docs/protocolo-serial.md`](docs/protocolo-serial.md), verificado por tests
que leen el firmware real y lo comparan con el Python.

### Qué sensor resuelve qué reto

Ninguno hace el trabajo de otro. Esto es lo que hay que tener claro antes de
tocar cualquier código:

| Reto de la demostración | Quién lo resuelve |
|---|---|
| Distinguir el **borde negro** del **fondo gris** de la pista | Los 2 sensores de **reflectancia** (QTR) |
| Identificar las **zonas de color** (negro, amarillo, rojo, azul) | Los 2 sensores de **color** (TCS34725) |
| Saber cuándo **cerrar la pinza** sobre la bandera cilíndrica | El **ToF** (VL53L1X) montado delante del gripper |
| **Detectar la bandera del oponente y señalizar** su detección | La **cámara** de la Pi → `CMD_FLAG_SIGNAL` → destella el LED del ESP32 |
| **Cargar y depositar la llave** | El **servo** del gripper (uno solo: agarra o suelta) |

---

## Estructura del repositorio

```
Athena-Rover-2026/
│
├── firmware-esp32/             Firmware de vuelo (CON Raspberry Pi).
│                               Todo en un main.cpp, 8 tareas FreeRTOS aisladas,
│                               cero librerías externas salvo el driver del ToF.
│
├── firmware-esp32-standalone/  Firmware de DEMOSTRACIÓN (SIN Raspberry Pi).
│                               La misión entera corre dentro del ESP32.
│                               Sin cámara no distingue bandera roja de azul:
│                               usa el ToF como sustituto. Documentado, no es
│                               un descuido.
│
├── raspberry-pi/               Visión y lógica de misión (Python).
│   ├── src/athena/              paquete principal
│   ├── scripts/                 el bucle del robot + herramienta de calibrado
│   ├── models/                  el .eim entrenado en Edge Impulse
│   ├── deploy/                  arranque automático con systemd
│   └── tests/                   61 tests, ninguno necesita hardware
│
├── pruebas-platformio/         Un proyecto chiquito por comportamiento, para
│                               validar con hardware real antes de integrar.
│
├── calibracion/                Herramientas de banco: caracterizar un sensor
│   ├── color/                   ANTES de fijar sus umbrales en el firmware.
│   └── reflectancia/            Incluye los datos crudos ya capturados.
│
├── hardware/                   Cómo cablear todo. Léelo antes de conectar nada.
└── docs/                       Reglas del reto, protocolo y decisiones de diseño.
```

---

## Por dónde empezar

| Si querés… | Leé |
|---|---|
| **Cablear el robot** | [`hardware/conexiones-esp32-s3.md`](hardware/conexiones-esp32-s3.md) — incluye los 5 errores que queman hardware |
| **Entender cómo se hablan las dos mitades** | [`docs/protocolo-serial.md`](docs/protocolo-serial.md) |
| **Tocar el firmware** | [`firmware-esp32/README.md`](firmware-esp32/README.md) |
| **Correr el robot en la Pi** | [`raspberry-pi/README.md`](raspberry-pi/README.md) |
| **Hacer la demo sin Raspberry Pi** | [`firmware-esp32-standalone/README.md`](firmware-esp32-standalone/README.md) |
| **Probar un subsistema suelto** | [`pruebas-platformio/README.md`](pruebas-platformio/README.md) |
| **Calibrar un sensor** | [`calibracion/README.md`](calibracion/README.md) |
| **Saber las reglas y las medidas reales** | [`docs/reglas-reto-rover-2025.md`](docs/reglas-reto-rover-2025.md) |
| **Entender el *porqué* del firmware** | [`docs/arquitectura-firmware-esp32.md`](docs/arquitectura-firmware-esp32.md) |
| **Retomar el trabajo de visión** | [`docs/handoff-vision-edge-impulse.md`](docs/handoff-vision-edge-impulse.md) |

---

## Reglas de la competencia (resumen)

Detalle completo en [`docs/reglas-reto-rover-2025.md`](docs/reglas-reto-rover-2025.md).

- Robot ≤ 30 × 20 × 20 cm, identificado con LED (rojo/azul).
- Cada ronda: depositar la llave (cubo ~20×20×20 mm) en la zona neutra amarilla
  → **después** buscar y capturar la bandera enemiga (cilindro de 5 cm de
  diámetro × 15 cm de alto) → llevarla a la zona segura propia.
- Pista real (según plano acotado, no el documento de reglas): **170 × 83.5 cm**,
  zona neutra central de 27 × 29 cm.
- Máximo 10 minutos por ronda. Pierde de inmediato quien saque 2 ruedas de la
  pista o busque la bandera antes de depositar la llave.

Esas dos formas de perder están escritas como **prioridad 1 y 2** en
`raspberry-pi/src/athena/decision.py`, y hay tests que lo verifican.

---

## Estado actual

**Listo:**

- [x] Firmware ESP32-S3 con tareas FreeRTOS aisladas por subsistema
- [x] Drivers propios de PCA9685 y TCS34725 (sin librerías externas)
- [x] Protocolo binario ESP32 ↔ Pi, verificado byte a byte y **exhaustivamente**
      por tests que leen el firmware real
- [x] Detección de la bandera rival con modelo FOMO entrenado en Edge Impulse
- [x] Señalización de la bandera por LED, de punta a punta (cámara → LED)
- [x] Máquina de estados de la misión, con las reglas de descalificación probadas
- [x] Firmware autónomo para demostrar sin Raspberry Pi, con la misión completa habilitada
- [x] Umbrales de color recalibrados contra 970 muestras reales

**Pendiente antes de competir:**

- [ ] Calibrar la focal de la cámara (`GeometryConfig.focal_px` es una estimación)
- [ ] Calibrar el sensor de color **trasero** (hoy comparte los umbrales del delantero)
- [ ] Resolver el retorno a la zona propia: hoy el robot avanza recto y solo
      reconoce su zona al pisarla. Necesita odometría o referencia visual. **Es
      el hueco más grande que queda.**

## Cómo colaborar

Ver [`CONTRIBUTING.md`](CONTRIBUTING.md).
