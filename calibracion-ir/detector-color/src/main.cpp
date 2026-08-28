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
//  Por qué solo el sensor B decide: en las dos capturas más recientes
//  (IR_NEGRO_2026-08-28_07-14-49.csv, IR_GRIS_2026-08-28_07-17-36.csv,
//  740 muestras en total) el sensor A quedó en 0 todo el tiempo — sin
//  excepción, no es ruido, está sin señal. Se sigue leyendo e imprimiendo
//  aquí como diagnóstico (para notar cuándo empiece a responder), pero no
//  participa en la clasificación.
//
//  Pines: idénticos a ../firmware/src/main.cpp (el sketch de captura), más
//  el LED RGB en los mismos GPIO que usa el diseño final
//  (firmware-esp32/, pruebas-platformio/02-cuadro-color-rgb/).
//
//  Sobre el umbral (145): con esas 740 muestras, NEGRO promedió 165.2
//  (desviación 13.1) y GRIS promedió 124.4 (desviación 7.6) en el sensor B.
//  145 es el punto medio razonable entre ambas — pero sigue siendo un
//  umbral fijo sacado de un log, no una calibración en vivo. Si la luz del
//  lugar cambia o el sensor se reposiciona, verifícalo de nuevo contra la
//  superficie real. La calibración en vivo de pruebas-platformio/01 y 02
//  (recalibra en cada arranque) sigue siendo la que manda para el robot de
//  verdad — esto es solo una herramienta de banco.
//
// ===========================================================================

#include <Arduino.h>
#include <QTRSensors.h>

// =====================================================
// PINES — idénticos a ../firmware/src/main.cpp, más el LED RGB
// =====================================================

const uint8_t QTR_A_SENSOR = 35;  // sin señal ahora mismo, ver nota arriba
const uint8_t QTR_A_CTRL   = 25;
const uint8_t QTR_B_SENSOR = 4;   // el que sí decide
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
