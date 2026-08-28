// ===========================================================================
//  Detector de color — banco de calibración (ESP32-S3)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  Lee los sensores A y B (QTRX-HD-01A analógicos) y clasifica la superficie
//  entre 4 etiquetas, con umbrales fijos calculados a partir de datos reales
//  capturados con ../calibrar_ir.py (ver ../data_logs/). Imprime el
//  resultado por consola cada 200 ms y enciende el LED RGB.
//
//  LÍMITE FÍSICO IMPORTANTE — léelo antes de tocar los umbrales:
//  Un QTR analógico mide reflectancia INFRARROJA, no color visible. Con los
//  5 colores capturados (NEGRO, GRIS, ROJO, AZUL, AMARILLO — ver la tabla
//  abajo), ROJO y AZUL dieron promedios a menos de 110 unidades de
//  distancia, bien dentro del ruido de cada uno (±355 y ±361 en el sensor
//  A). No es un umbral mal elegido: ese sensor NO PUEDE distinguir rojo de
//  azul de forma confiable, sin importar qué constante se le ponga. Por eso
//  el diseño de vuelo usa el TCS34725 (sensor de color RGB de verdad) para
//  las zonas de la pista, y el QTR solo para el borde de la cinta negra.
//  Este sketch reporta "ROJO/AZUL (ambiguo)" en vez de fingir que sabe cuál
//  de los dos es.
//
//  Datos de referencia (5 capturas, sensor A / sensor B, media ± desv.est.):
//    NEGRO     A=3969±257  B=2925±23
//    GRIS      A=3504±301  B=2867±55
//    AZUL      A=2691±361  B=2613±181
//    ROJO      A=2587±355  B=2677±187
//    AMARILLO  A=2551±650  B=2288±291
//
//  De ahí salen los 3 umbrales de abajo: primero se separa "oscuro"
//  (negro/gris, A alto) de "color" (rojo/azul/amarillo, A bajo) — es la
//  separación más limpia de las cinco. Dentro de "oscuro", A vuelve a
//  decidir negro vs. gris. Dentro de "color", el amarillo se distingue
//  porque su B es notablemente más bajo que rojo/azul; lo que quede no se
//  reparte, se reporta como ambiguo.
//
//  Pines: cableado físico actual confirmado por el equipo — sensores en
//  GPIO1/GPIO2, control compartido en GPIO42 (los mismos GPIO que
//  QTR_LEFT_OUT/QTR_RIGHT_OUT/QTR_EMITTER_CTRL del diseño de vuelo, ver
//  hardware/conexiones-esp32-s3.md) — más el LED RGB en los mismos GPIO
//  que usa el diseño final (firmware-esp32/,
//  pruebas-platformio/02-cuadro-color-rgb/). El módulo IR genérico no está
//  en uso (no está cableado), así que este sketch ya no lo lee.
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
// PINES — idénticos a ../firmware/src/main.cpp, más el LED RGB
// =====================================================

const uint8_t QTR_A_SENSOR = 1;   // = QTR_LEFT_OUT en hardware/conexiones-esp32-s3.md
const uint8_t QTR_A_CTRL   = 42;  // = QTR_EMITTER_CTRL en hardware/conexiones-esp32-s3.md
const uint8_t QTR_B_SENSOR = 2;   // = QTR_RIGHT_OUT en hardware/conexiones-esp32-s3.md
const uint8_t QTR_B_CTRL   = QTR_A_CTRL;  // mismo pin físico que QTR_A_CTRL

const uint8_t RGB_R = 39;
const uint8_t RGB_G = 38;
const uint8_t RGB_B = 3;

// =====================================================
// UMBRALES — ver el análisis completo en el encabezado
// =====================================================

constexpr uint16_t UMBRAL_A_OSCURO_COLOR = 3100;  // sensor A > esto => tier "oscuro" (negro/gris)
constexpr uint16_t UMBRAL_A_NEGRO_GRIS   = 3700;  // dentro de "oscuro": A > esto => NEGRO, si no => GRIS
constexpr uint16_t UMBRAL_B_AMARILLO     = 2450;  // dentro de "color": B < esto => AMARILLO

// =====================================================
// QTR OBJECTS
// =====================================================

QTRSensors qtrA;
QTRSensors qtrB;

uint16_t valuesA[1];
uint16_t valuesB[1];

// =====================================================
// CLASIFICACIÓN
// =====================================================

enum class Color : uint8_t { NEGRO, GRIS, AMARILLO, ROJO_AZUL };

const char *ColorNombre(Color c) {
    switch (c) {
        case Color::NEGRO:     return "NEGRO";
        case Color::GRIS:      return "GRIS";
        case Color::AMARILLO:  return "AMARILLO";
        case Color::ROJO_AZUL: return "ROJO/AZUL (ambiguo)";
        default:                return "?";
    }
}

Color Clasificar(uint16_t a, uint16_t b) {
    if (a > UMBRAL_A_OSCURO_COLOR) {
        return (a > UMBRAL_A_NEGRO_GRIS) ? Color::NEGRO : Color::GRIS;
    }
    return (b < UMBRAL_B_AMARILLO) ? Color::AMARILLO : Color::ROJO_AZUL;
}

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

    // Rojo=negro y verde=gris quedan igual que antes. Amarillo se mapea a
    // amarillo. Para "ROJO/AZUL (ambiguo)" se usa púrpura a propósito: no es
    // ni rojo ni azul puro, para no fingir una respuesta que el sensor no
    // puede dar.
    void ApplyColor(Color c) {
        switch (c) {
            case Color::NEGRO:     SetRaw(255,   0,   0); break;
            case Color::GRIS:      SetRaw(  0, 255,   0); break;
            case Color::AMARILLO:  SetRaw(255, 200,   0); break;
            case Color::ROJO_AZUL: SetRaw(160,   0, 200); break;
        }
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

    const Color color = Clasificar(valuesA[0], valuesB[0]);

    RgbLed::ApplyColor(color);

    Serial.print(ColorNombre(color));
    Serial.print("  A=");
    Serial.print(valuesA[0]);
    Serial.print("  B=");
    Serial.println(valuesB[0]);

    delay(200);
}
