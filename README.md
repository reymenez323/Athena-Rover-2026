# Athena Rover 2026 — Retos del Rover H07

Proyecto universitario del Instituto Tecnológico de Santo Domingo (INTEC) — Ingeniería Mecatrónica.

**Equipo:** Reymildo · Montse

Robot autónomo diseñado para competir en el reto **"Retos del Rover – H07"**: dos robots se enfrentan en una pista, cada uno debe depositar una llave en la zona neutra, luego identificar, capturar y trasladar la bandera del oponente hasta su propia zona segura.

## Arquitectura del robot

- **Tracción:** 4 motores.
- **Gripper** delantero accionado por servomotor, con un segundo servo para subir/bajar el gripper.
- **Sensores de color:** uno delantero y uno trasero, para detectar las líneas de zona en el piso (negro, amarillo, rojo, azul).
- **Sensores de reflectancia:** 2 delanteros (izquierda y derecha).
- **Cámara web USB** para detectar la bandera del oponente y la llave.
- **Cómputo:** Raspberry Pi (visión e inteligencia de alto nivel) ↔ comunicación serial ↔ **ESP32-S3** (control de actuadores y lectura de sensores de bajo nivel, firmware basado en **FreeRTOS** con una tarea independiente por subsistema — ver [`docs/arquitectura-firmware-esp32.md`](docs/arquitectura-firmware-esp32.md)).

## Estructura del repositorio

```
Athena-Rover-2026/
├── firmware-esp32/     Firmware del ESP32-S3: TODO en un solo main.cpp,
│                       7 tareas FreeRTOS independientes, cero librerías externas
├── raspberry-pi/       Visión (cámara USB + CNN INT8) y lógica de misión
│   ├── src/athena/      paquete principal
│   ├── data/            dataset para entrenar el modelo
│   ├── scripts/         captura, entrenamiento, benchmark, bucle principal
│   └── tests/           28 tests, ninguno necesita hardware
├── hardware/           Conexiones, esquemáticos, CAD
└── docs/               Reglas del reto y decisiones de diseño
```

## Reglas de la competencia (resumen)

Ver [`docs/reglas-reto-rover-2025.md`](docs/reglas-reto-rover-2025.md) para el detalle completo.

- Robot ≤ 30 x 20 x 20 cm, identificado con LED (rojo/azul).
- Cada ronda: depositar llave (cubo ~20x20x20 mm) en la zona neutra amarilla → luego buscar y capturar la bandera enemiga (cilindro 5 cm diám. x 15 cm alto) → llevarla a la zona segura propia.
- Pista real (medidas confirmadas por el equipo, según plano acotado): **170 cm x 83.5 cm**, zona neutra central de 27 x 29 cm.
- Ronda: máx. 10 minutos; pierde quien saque 2 ruedas de la pista o busque la bandera antes de depositar la llave.

## Documentos clave

| Documento | Para qué |
|-----------|----------|
| [`hardware/conexiones-esp32-s3.md`](hardware/conexiones-esp32-s3.md) | **Cómo cablear todo.** Incluye los 5 errores que queman hardware. Léelo antes de conectar nada. |
| [`firmware-esp32/README.md`](firmware-esp32/README.md) | Las 7 tareas de FreeRTOS y cómo se aíslan entre sí |
| [`raspberry-pi/README.md`](raspberry-pi/README.md) | El pipeline de visión y por qué está diseñado así |
| [`docs/reglas-reto-rover-2025.md`](docs/reglas-reto-rover-2025.md) | Reglas del reto y medidas reales de la pista |
| [`docs/arquitectura-firmware-esp32.md`](docs/arquitectura-firmware-esp32.md) | El *porqué* de las decisiones de diseño del firmware |

## Estado actual

- [x] Firmware ESP32-S3 con 7 tareas FreeRTOS aisladas
- [x] Drivers propios de PCA9685 y TCS34725 (sin librerías externas)
- [x] Protocolo binario ESP32 ↔ RPi, verificado byte a byte por tests
- [x] Pipeline de visión en 2 etapas optimizado para la Pi 4B
- [x] Máquina de estados de la misión, con las reglas de descalificación probadas
- [ ] Calibrar sensores de color y reflectancia sobre la pista real
- [ ] Capturar el dataset y entrenar el modelo
- [ ] Calibrar la focal de la cámara
- [ ] Resolver el retorno a la zona propia (necesita odometría o referencia visual)

## Cómo colaborar

Ver [`CONTRIBUTING.md`](CONTRIBUTING.md).
