# Rover 2025 — Retos del Rover H07

Proyecto universitario del Instituto Tecnológico de Santo Domingo (INTEC) — Ingeniería Mecatrónica.

**Equipo:** Reymildo · Montse

Robot autónomo diseñado para competir en el reto **"Retos del Rover – H07"**: dos robots se enfrentan en una pista, cada uno debe depositar una llave en la zona neutra, luego identificar, capturar y trasladar la bandera del oponente hasta su propia zona segura.

## Arquitectura del robot

- **Tracción:** 4 motores.
- **Gripper** delantero accionado por servomotor, con un segundo servo para subir/bajar el gripper.
- **Sensores de color:** uno delantero y uno trasero, para detectar las líneas de zona en el piso (negro, amarillo, rojo, azul).
- **Sensores de reflectancia:** 2 delanteros (izquierda y derecha).
- **Cámara web USB** para detectar la bandera del oponente y la llave.
- **Cómputo:** Raspberry Pi (visión e inteligencia de alto nivel) ↔ comunicación serial ↔ **ESP32-S3** (control de actuadores y lectura de sensores de bajo nivel).

## Estructura del repositorio

```
rover-2025/
├── firmware-esp32/     # Firmware del ESP32-S3 (control de motores, servos, sensores)
├── raspberry-pi/       # Código de alto nivel: visión (cámara), lógica de misión, comunicación serial
├── hardware/           # Diseños CAD, esquemáticos electrónicos, diagramas de cableado
├── docs/                # Documentación del proyecto, reglas del reto, notas de diseño
└── README.md
```

## Reglas de la competencia (resumen)

Ver [`docs/reglas-reto-rover-2025.md`](docs/reglas-reto-rover-2025.md) para el detalle completo.

- Robot ≤ 30 x 20 x 20 cm, identificado con LED (rojo/azul).
- Cada ronda: depositar llave (cubo ~20x20x20 mm) en la zona neutra amarilla → luego buscar y capturar la bandera enemiga (cilindro 5 cm diám. x 15 cm alto) → llevarla a la zona segura propia.
- Pista real (medidas confirmadas por el equipo, según plano acotado): **170 cm x 83.5 cm**, zona neutra central de 27 x 29 cm.
- Ronda: máx. 10 minutos; pierde quien saque 2 ruedas de la pista o busque la bandera antes de depositar la llave.

## Cómo colaborar

Ver [`CONTRIBUTING.md`](CONTRIBUTING.md).
