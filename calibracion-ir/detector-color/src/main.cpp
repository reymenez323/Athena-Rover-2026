// ===========================================================================
//  Detector NEGRO/GRIS — banco de calibración (ESP32-S3)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  Lee los sensores A y B (QTRX-HD-01A analógicos) y clasifica NEGRO vs.
//  GRIS. Imprime el resultado por consola cada 200 ms y enciende el LED
//  RGB: rojo para negro, verde para gris (igual que siempre).
//
//  SOLO decide el sensor B — no es un umbral por capricho, es lo que dio
//  menos errores probando contra los datos reales. Se probaron A solo, B
//  solo, y varias combinaciones lineales de ambos (A+B, A-B, promedios,
//  A + w·B para w entre -3 y 3) sobre las 740 muestras de
//  ../data_logs/IR_NEGRO_2026-08-28_07-42-21.csv e
//  IR_GRIS_2026-08-28_07-44-29.csv. El ganador, por lejos, fue B solo:
//
//      Umbral         Errores totales   Detalle
//      A solo (3700)      80 / 740      66 negro→gris, 14 gris→negro
//      A + 0.8·B          69 / 740      58 negro→gris, 11 gris→negro
//      B solo (2922)       40 / 740      40 negro→gris,  0 gris→negro
//
//  Con B, el rango de NEGRO fue 2804–2940 y el de GRIS 2457–2922 — se
//  solapan solo en una franja angosta (2804–2922). Ahí es normal que
//  algunas muestras de negro salgan como gris (nunca al revés, con este
//  umbral) — es ruido real de esa franja, no un umbral mal elegido. Si
//  hace falta más margen, el próximo paso NO es otro umbral: es fijar la
//  atenuación del ADC explícitamente (analogSetPinAttenuation, como hace
//  firmware-esp32/) en vez de dejar la que trae por defecto — ver
//  ../README.md.
//
//  El sensor A se sigue leyendo e imprimiendo (para tenerlo a la vista),
//  pero ya NO participa en la decisión.
//
//  Pines: cableado físico actual confirmado por el equipo — sensores en
//  GPIO1/GPIO2, control compartido en GPIO42 (los mismos GPIO que
//  QTR_LEFT_OUT/QTR_RIGHT_OUT/QTR_EMITTER_CTRL del diseño de vuelo, ver
//  hardware/conexiones-esp32-s3.md) — más el LED RGB en los mismos GPIO
//  que usa el diseño final (firmware-esp32/,
//  pruebas-platformio/02-cuadro-color-rgb/). El módulo IR genérico no está
//  en uso (no está cableado), así que este sketch no lo lee.
//
//  Como con cualquier umbral fijo sacado de un log puntual: si cambia la
//  luz del lugar o se reposiciona el sensor, verifícalo de nuevo contra la
//  superficie real. La calibración en vivo de pruebas-platformio/01 y 02
//  (recalibra en cada arranque) sigue siendo la que manda para el robot de
//  verdad — esto es solo una herramienta de banco.
//
// ===========================================================================

#include <Arduino.h>
#include <QTRSensors.h>

// =====================================================
// PINES
// =====================================================

const uint8_t QTR_A_SENSOR = 1;   // = QTR_LEFT_OUT en hardware/conexiones-esp32-s3.md — se lee pero no decide
const uint8_t QTR_A_CTRL   = 42;  // = QTR_EMITTER_CTRL en hardware/conexiones-esp32-s3.md
const uint8_t QTR_B_SENSOR = 2;   // = QTR_RIGHT_OUT en hardware/conexiones-esp32-s3.md — el que decide
const uint8_t QTR_B_CTRL   = QTR_A_CTRL;  // mismo pin físico que QTR_A_CTRL

const uint8_t RGB_R = 39;
const uint8_t RGB_G = 38;
const uint8_t RGB_B = 3;

// =====================================================
// UMBRAL — ver el análisis completo en el encabezado
// =====================================================

constexpr uint16_t UMBRAL_NEGRO_GRIS = 2922;  // sensor B > esto => NEGRO, si no => GRIS

// =====================================================
// QTR OBJECTS
// =====================================================

QTRSensors qtrA;
QTRSensors qtrB;

uint16_t valuesA[1];
uint16_t valuesB[1];

// =====================================================
// LED RGB
// =====================================================
//
// POLARIDAD SIN CONFIRMAR con el LED físico (mismo TODO que en
// firmware-esp32/ y pruebas-platformio/02-cuadro-color-rgb/): se asume
// cátodo común (duty alto = canal más brillante). Si al probarlo los
// colores salen invertidos, cambiar kCommonAnode a true — no hace falta
// tocar el resto del código.

constexpr bool kCommonAnode = false;

namespace Pwm {
    constexpr uint32_t RGB_FREQ_HZ    = 5000;  // fuera del rango audible
    constexpr uint8_t  RGB_RESOLUTION = 8;     // duty 0..255
}

// La API de LEDC cambió entre el core 2.x y el 3.x de Arduino-ESP32.
void PwmAttach(uint8_t pin, uint8_t channel, uint32_t freqHz, uint8_t resolution) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;
    ledcAttach(pin, freqHz, resolution);
#else
    ledcSetup(channel, freqHz, resolution);
    ledcAttachPin(pin, channel);
#endif
}

void PwmWrite(uint8_t pin, uint8_t channel, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;
    ledcWrite(pin, duty);
#else
    (void)pin;
    ledcWrite(channel, duty);
#endif
}

namespace RgbLed {
    constexpr uint8_t CH_R = 0;
    constexpr uint8_t CH_G = 1;
    constexpr uint8_t CH_B = 2;

    void Setup() {
        PwmAttach(RGB_R, CH_R, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(RGB_G, CH_G, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(RGB_B, CH_B, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
    }

    void SetRaw(uint8_t r, uint8_t g, uint8_t b) {
        if (kCommonAnode) { r = 255 - r; g = 255 - g; b = 255 - b; }
        PwmWrite(RGB_R, CH_R, r);
        PwmWrite(RGB_G, CH_G, g);
        PwmWrite(RGB_B, CH_B, b);
    }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nDetector NEGRO/GRIS - banco de calibracion");

    analogReadResolution(12);

    qtrA.setTypeAnalog();
    const uint8_t pinsA[] = { QTR_A_SENSOR };
    qtrA.setSensorPins(pinsA, 1);
    qtrA.setSamplesPerSensor(8);
    qtrA.setEmitterPin(QTR_A_CTRL);

    qtrB.setTypeAnalog();
    const uint8_t pinsB[] = { QTR_B_SENSOR };
    qtrB.setSensorPins(pinsB, 1);
    qtrB.setSamplesPerSensor(8);
    qtrB.setEmitterPin(QTR_B_CTRL);

    RgbLed::Setup();
    RgbLed::SetRaw(0, 0, 0);

    Serial.println("Listo.\n");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    qtrA.read(valuesA);
    qtrB.read(valuesB);

    const bool esNegro = valuesB[0] > UMBRAL_NEGRO_GRIS;

    RgbLed::SetRaw(esNegro ? 255 : 0, esNegro ? 0 : 255, 0);

    Serial.print(esNegro ? "NEGRO" : "GRIS ");
    Serial.print("  B=");
    Serial.print(valuesB[0]);
    Serial.print("  A=");
    Serial.print(valuesA[0]);
    Serial.println("  (A no participa en la decision)");

    delay(200);
}
