// ===========================================================================
//  Detector de color — banco de calibración (ESP32-S3)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  Lee el sensor B (QTRX-HD-01A analógico) y clasifica NEGRO vs. GRIS con un
//  umbral fijo, calculado a partir de datos reales capturados con
//  ../calibrar_ir.py (ver ../data_logs/). Imprime el resultado por consola
//  cada 200 ms y enciende el LED RGB: rojo para negro, verde para gris.
//
//  Por qué solo el sensor B decide (por ahora): el umbral de abajo se
//  calculó con datos donde el sensor A no daba señal (estaba en GPIO35,
//  que en el ESP32-S3 no tiene hardware de ADC — analogRead() ahí siempre
//  tira "Pin 35 is not ADC pin!" y devuelve 0). Ya se recableó, así que A
//  se sigue leyendo e imprimiendo, pero no participa en la clasificación
//  todavía.
//
//  Pines: cableado físico actual confirmado por el equipo — sensores en
//  GPIO1/GPIO2, control compartido en GPIO42 (los mismos GPIO que
//  QTR_LEFT_OUT/QTR_RIGHT_OUT/QTR_EMITTER_CTRL del diseño de vuelo, ver
//  hardware/conexiones-esp32-s3.md) — más el LED RGB en los mismos GPIO
//  que usa el diseño final (firmware-esp32/,
//  pruebas-platformio/02-cuadro-color-rgb/).
//
//  Sobre el umbral (145): calculado con 740 muestras del sensor B leído en
//  GPIO4 (NEGRO promedió 165.2, desviación 13.1; GRIS promedió 124.4,
//  desviación 7.6). El sensor B ahora se lee por GPIO2 en vez de GPIO4 —
//  debería dar los mismos números (es el mismo sensor físico, ambos GPIO
//  son ADC1 equivalentes), pero como no se volvió a capturar después del
//  cambio de pines, conviene verificarlo contra la superficie real antes
//  de confiar en el umbral a ciegas. La calibración en vivo de
//  pruebas-platformio/01 y 02 (recalibra en cada arranque) sigue siendo la
//  que manda para el robot de verdad — esto es solo una herramienta de banco.
//
// ===========================================================================

#include <Arduino.h>
#include <QTRSensors.h>

// =====================================================
// PINES — idénticos a ../firmware/src/main.cpp, más el LED RGB
// =====================================================

const uint8_t QTR_A_SENSOR = 1;   // = QTR_LEFT_OUT en hardware/conexiones-esp32-s3.md
const uint8_t QTR_A_CTRL   = 42;  // = QTR_EMITTER_CTRL en hardware/conexiones-esp32-s3.md
const uint8_t QTR_B_SENSOR = 2;   // = QTR_RIGHT_OUT en hardware/conexiones-esp32-s3.md — el que sí decide
const uint8_t QTR_B_CTRL   = QTR_A_CTRL;  // mismo pin físico que QTR_A_CTRL
const uint8_t GENERIC_IR   = 14;

const uint8_t RGB_R = 39;
const uint8_t RGB_G = 38;
const uint8_t RGB_B = 3;

// =====================================================
// UMBRAL — ver análisis en el encabezado
// =====================================================

constexpr uint16_t UMBRAL_NEGRO_GRIS = 145;  // sensor B > esto => NEGRO, si no => GRIS

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
    Serial.println("\nDetector de color - banco de calibracion");

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

    pinMode(GENERIC_IR, INPUT);

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
    int genericValue = digitalRead(GENERIC_IR);

    const bool esNegro = valuesB[0] > UMBRAL_NEGRO_GRIS;

    RgbLed::SetRaw(esNegro ? 255 : 0, esNegro ? 0 : 255, 0);

    Serial.print(esNegro ? "NEGRO" : "GRIS ");
    Serial.print("  B=");
    Serial.print(valuesB[0]);
    Serial.print("  A=");
    Serial.print(valuesA[0]);
    if (valuesA[0] == 0) Serial.print(" [sin senal]");
    Serial.print("  IR_generico=");
    Serial.println(genericValue);

    delay(200);
}
