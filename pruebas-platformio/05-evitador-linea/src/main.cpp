// ===========================================================================
//  Prueba 05 — Evitador de línea (QTR con prioridad + color de respaldo)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  OBJETIVO: el robot se queda SIEMPRE dentro del rectángulo delimitado por
//  la cinta negra. En cuanto detecta el borde: retrocede ~1.5 s en línea
//  recta (kReverseMs, cronometrado) y después gira sobre su propio eje
//  hacia un lado elegido AL AZAR (izquierda o derecha, 50/50) hasta que
//  confirma que ya no hay negro debajo — el giro en sí sigue siendo
//  reactivo, sin temporizador: dura lo que el sensor diga, no un número
//  cronometrado.
//
//  DOS TIPOS DE SENSOR, CON PRIORIDAD DISTINTA (no son iguales a propósito):
//
//    1. QTR — AMBOS (izquierdo GPIO1 y derecho GPIO2), reflectancia,
//       `analogRead()` en <1 ms. Es el sensor CON PRIORIDAD: decide solo,
//       rápido (confirmado en ~20 ms), tanto para EMPEZAR a girar como
//       para VOLVER a avanzar. No necesita que el color esté de acuerdo
//       para nada.
//    2. Color — TCS34725 delantero (I2C0) Y trasero (I2C1), clasificados
//       con la MISMA lógica de umbrales que
//       calibracion/color/detector-tcs/src/main.cpp (Umbral::/Clasificar(),
//       recalibrados contra datos reales — ver ese archivo para el detalle
//       y las limitaciones). Es un sensor DE RESPALDO, de menor prioridad:
//       - Más lento (se muestrea cada COLOR_PERIOD_MS, no cada vuelta de
//         loop()) y exige MÁS lecturas seguidas para confiar en él
//         (kConfirmacionesColor > kConfirmacionesQtr) — tarda más en
//         convencerse.
//       - SOLO puede iniciar un giro si el QTR todavía no lo hizo (red de
//         seguridad ante un QTR que falle o esté mal calibrado); NUNCA
//         decide cuándo volver a avanzar — esa decisión es 100% del QTR.
//       - SOLO le importa si la superficie es NEGRO. Cualquier otra
//         clasificación (AMARILLO, ROJO, AZUL, GRIS) se ignora POR
//         COMPLETO para la máquina de estados — no dispara nada, no evita
//         nada. Sí se sigue mostrando en el LED RGB y la consola, como
//         diagnóstico.
//
//  Sketch de banco, deliberadamente simple (setup/loop, sin FreeRTOS ni
//  colas) — usa los MISMOS pines que firmware-esp32/ (ver
//  hardware/conexiones-esp32-s3.md): los 2 QTR, los 2 TCS34725, los 2
//  L298N y el LED RGB. Es, en los sensores que toca, un
//  subconjunto completo del firmware final (todo menos el PCA9685).
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>

// ===========================================================================
//  [1] PINES — idénticos a hardware/conexiones-esp32-s3.md y a firmware-esp32
// ===========================================================================

namespace Pins {
    // Reflectancia QTRX-HD-01A — los DOS, con prioridad sobre el color.
    constexpr uint8_t QTR_LEFT_OUT      = 1;   // ADC1_CH0
    constexpr uint8_t QTR_RIGHT_OUT     = 2;   // ADC1_CH1
    constexpr uint8_t QTR_EMITTER_CTRL  = 42;  // enciende los LED IR de ambos sensores

    // TCS34725 — DELANTERO (bus I2C0) y TRASERO (bus I2C1). Misma
    // dirección fija (0x29) en los dos, por eso van en buses separados.
    constexpr uint8_t I2C0_SDA = 8;    // delantero
    constexpr uint8_t I2C0_SCL = 9;
    constexpr uint8_t I2C1_SDA = 47;   // trasero
    constexpr uint8_t I2C1_SCL = 48;
    constexpr uint8_t TCS_LED_FRONT = 18;
    constexpr uint8_t TCS_LED_BACK  = 21;

    // Motores: 2x L298N (quitar los jumpers de ENA/ENB en ambos, o el PWM no
    // hace nada). Driver IZQUIERDO mueve FL+RL, driver DERECHO mueve FR+RR.
    constexpr uint8_t L298N_L_IN1 = 4;
    constexpr uint8_t L298N_L_IN2 = 5;
    constexpr uint8_t L298N_L_ENA = 6;
    constexpr uint8_t L298N_L_IN3 = 7;
    constexpr uint8_t L298N_L_IN4 = 15;
    constexpr uint8_t L298N_L_ENB = 16;

    constexpr uint8_t L298N_R_IN1 = 10;
    constexpr uint8_t L298N_R_IN2 = 11;
    constexpr uint8_t L298N_R_ENA = 12;
    constexpr uint8_t L298N_R_IN3 = 13;
    constexpr uint8_t L298N_R_IN4 = 14;
    constexpr uint8_t L298N_R_ENB = 17;

    // LED RGB — muestra en vivo la clasificación de color del sensor
    // DELANTERO, y el destello de alerta durante la maniobra de borde
    // (diagnóstico; no decide nada por sí solo, ver encabezado).
    // LED RGB del robot. Antes había además 2 LED discretos rojo/azul en
    // GPIO 40 y 41 para señalar la maniobra de borde: ya no existen, el
    // equipo los reemplazó por este único LED RGB, y el GPIO 41 pasó a ser
    // su canal AZUL.
    //
    // El canal azul también se movió: estaba en GPIO 3, que ahora es el
    // XSHUT del VL53L1X (ver hardware/conexiones-esp32-s3.md). Manejarlo
    // desde acá dejaría al ToF en reset permanente en el chasis real.
    constexpr uint8_t RGB_R = 39;
    constexpr uint8_t RGB_G = 38;
    constexpr uint8_t RGB_B = 41;
}

namespace I2CAddr {
    constexpr uint8_t TCS34725 = 0x29;
}

#define DEBUG_LINK Serial0   // consola por el puerto UART del DevKit

// ===========================================================================
//  [2] CALIBRACIÓN DEL QTR EN ARRANQUE — los DOS sensores
// ===========================================================================
//
//  Durante CALIBRATION_MS el LED RGB parpadea en blanco: es la
//  señal para pasar el robot a mano por ENCIMA de la cinta negra y del piso
//  gris varias veces, cubriendo ambos sensores. Cada uno se queda con su
//  propio mínimo/máximo y arma su propio umbral — igual que hacía la
//  primera versión de 01-mantente-en-cuadro, antes de simplificarla a un
//  solo sensor. Acá se usan los DOS a propósito (ver encabezado: el QTR
//  tiene prioridad, así que le conviene cubrir ambos lados del robot).

constexpr uint32_t CALIBRATION_MS = 3000;
constexpr uint16_t MIN_USABLE_SPAN = 300;

// Umbrales de reserva si un sensor no vio suficiente contraste al calibrar:
// el derecho usa el valor ya confirmado contra datos reales (sensor B, ver
// calibracion/reflectancia/detector-negro-gris/src/main.cpp: 2910). El izquierdo no tiene
// ese mismo nivel de confirmación — su análisis (sensor A) dio un
// contraste NEGRO/GRIS más parejo (~3400-4060) y el punto medio 3700 fue
// el que se usó históricamente antes de que el equipo confirmara que el
// sensor B medía mejor (ver 01-mantente-en-cuadro/src/main.cpp).
constexpr uint16_t FALLBACK_THRESHOLD_LEFT  = 3700;
constexpr uint16_t FALLBACK_THRESHOLD_RIGHT = 2910;

uint16_t g_leftThreshold  = FALLBACK_THRESHOLD_LEFT;
uint16_t g_rightThreshold = FALLBACK_THRESHOLD_RIGHT;

// Declaración adelantada: la calibración parpadea el LED, pero vive antes que
// el bloque [6] donde el RGB está definido. Se declara acá en vez de mover
// código para no reordenar un sketch que ya está probado en banco.
// setup() llama a RgbLed::Setup() antes de RunCalibration(), así que el PWM
// ya está enganchado cuando esto corre.
namespace RgbLed { void SetRaw(uint8_t r, uint8_t g, uint8_t b); }

void RunCalibration() {
    uint16_t leftMin = 4095, leftMax = 0;
    uint16_t rightMin = 4095, rightMax = 0;

    const uint32_t start = millis();
    uint32_t lastBlink = start;
    bool blinkState = false;

    DEBUG_LINK.println("[Calib] moviendo el robot sobre negro y gris ahora (3 s)...");

    while ((uint32_t)(millis() - start) < CALIBRATION_MS) {
        const uint16_t l = (uint16_t)analogRead(Pins::QTR_LEFT_OUT);
        const uint16_t r = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);
        if (l < leftMin) leftMin = l;
        if (l > leftMax) leftMax = l;
        if (r < rightMin) rightMin = r;
        if (r > rightMax) rightMax = r;

        if ((uint32_t)(millis() - lastBlink) > 150) {
            lastBlink = millis();
            blinkState = !blinkState;
            RgbLed::SetRaw(blinkState ? 255 : 0, blinkState ? 255 : 0, blinkState ? 255 : 0);
        }
        delay(5);
    }
    RgbLed::SetRaw(0, 0, 0);

    const uint16_t leftSpan  = leftMax - leftMin;
    const uint16_t rightSpan = rightMax - rightMin;

    if (leftSpan >= MIN_USABLE_SPAN) {
        g_leftThreshold = (uint16_t)((leftMin + leftMax) / 2);
    } else {
        DEBUG_LINK.println("[Calib] sensor IZQUIERDO no vio suficiente contraste, uso el umbral de reserva.");
    }
    if (rightSpan >= MIN_USABLE_SPAN) {
        g_rightThreshold = (uint16_t)((rightMin + rightMax) / 2);
    } else {
        DEBUG_LINK.println("[Calib] sensor DERECHO no vio suficiente contraste, uso el umbral de reserva.");
    }

    DEBUG_LINK.printf("[Calib] izq: min=%u max=%u umbral=%u\n", leftMin, leftMax, g_leftThreshold);
    DEBUG_LINK.printf("[Calib] der: min=%u max=%u umbral=%u\n", rightMin, rightMax, g_rightThreshold);
}

// ===========================================================================
//  [3] DRIVER TCS34725 — copia exacta de firmware-esp32/ y calibracion/color/
// ===========================================================================

namespace Tcs34725 {
    constexpr uint8_t CMD_BIT      = 0x80;
    constexpr uint8_t CMD_AUTO_INC = 0x20;

    constexpr uint8_t REG_ENABLE  = 0x00;
    constexpr uint8_t REG_ATIME   = 0x01;
    constexpr uint8_t REG_CONTROL = 0x0F;
    constexpr uint8_t REG_ID      = 0x12;
    constexpr uint8_t REG_CDATAL  = 0x14;

    constexpr uint8_t ENABLE_PON = 0x01;
    constexpr uint8_t ENABLE_AEN = 0x02;

    constexpr uint8_t ATIME_24MS = 0xEB;
    constexpr uint8_t GAIN_4X    = 0x01;

    struct Rgbc { uint16_t c = 0, r = 0, g = 0, b = 0; };

    bool WriteReg(TwoWire &bus, uint8_t reg, uint8_t value) {
        bus.beginTransmission(I2CAddr::TCS34725);
        bus.write(CMD_BIT | reg);
        bus.write(value);
        return bus.endTransmission() == 0;
    }

    bool ReadReg(TwoWire &bus, uint8_t reg, uint8_t &out) {
        bus.beginTransmission(I2CAddr::TCS34725);
        bus.write(CMD_BIT | reg);
        if (bus.endTransmission() != 0) return false;
        if (bus.requestFrom((int)I2CAddr::TCS34725, 1) != 1) return false;
        out = (uint8_t)bus.read();
        return true;
    }

    bool Init(TwoWire &bus) {
        uint8_t id = 0;
        if (!ReadReg(bus, REG_ID, id)) return false;
        if (id != 0x44 && id != 0x4D) return false;

        if (!WriteReg(bus, REG_ATIME, ATIME_24MS)) return false;
        if (!WriteReg(bus, REG_CONTROL, GAIN_4X)) return false;
        if (!WriteReg(bus, REG_ENABLE, ENABLE_PON)) return false;
        delay(3);
        return WriteReg(bus, REG_ENABLE, ENABLE_PON | ENABLE_AEN);
    }

    bool Read(TwoWire &bus, Rgbc &out) {
        bus.beginTransmission(I2CAddr::TCS34725);
        bus.write(CMD_BIT | CMD_AUTO_INC | REG_CDATAL);
        if (bus.endTransmission() != 0) return false;
        if (bus.requestFrom((int)I2CAddr::TCS34725, 8) != 8) return false;

        out.c = (uint16_t)(bus.read() | (bus.read() << 8));
        out.r = (uint16_t)(bus.read() | (bus.read() << 8));
        out.g = (uint16_t)(bus.read() | (bus.read() << 8));
        out.b = (uint16_t)(bus.read() | (bus.read() << 8));
        return true;
    }
}

// ===========================================================================
//  [4] CLASIFICACIÓN DE COLOR — idéntica a calibracion/color/detector-tcs/
// ===========================================================================
//
//  Umbral::, Clasificar() y LabelName() son una copia TAL CUAL de
//  calibracion/color/detector-tcs/src/main.cpp — el evitador debe
//  comportarse igual que ese banco de referencia, así que usa exactamente
//  la misma clasificación (recalibrada contra 970 muestras reales del
//  sensor delantero: 14.3% de error total, ver el detalle y las
//  limitaciones en ese archivo). Si se vuelve a recalibrar, hay que
//  actualizar LOS TRES lugares (acá, detector-tcs/, y ClassifyColor() en
//  firmware-esp32/) o quedan desincronizados.
//
//  Para ESTE sketch, en la práctica solo importa el caso NEGRO (ver
//  encabezado del archivo): las demás ramas (ROJO/AZUL/AMARILLO) se
//  calculan igual que en detector-tcs por completitud/diagnóstico, pero la
//  máquina de estados de más abajo las ignora por completo.

enum class ColorLabel : uint8_t { NEGRO, AMARILLO, ROJO, AZUL, GRIS };

const char *LabelName(ColorLabel l) {
    switch (l) {
        case ColorLabel::NEGRO:    return "NEGRO";
        case ColorLabel::AMARILLO: return "AMARILLO";
        case ColorLabel::ROJO:     return "ROJO";
        case ColorLabel::AZUL:     return "AZUL";
        case ColorLabel::GRIS:     return "GRIS";
    }
    return "?";
}

namespace Umbral {
    constexpr uint16_t CLEAR_NEGRO_MAX = 392;
    constexpr float ROJO_R_MIN     = 0.450f;
    constexpr float ROJO_G_MAX     = 0.312f;
    constexpr float ROJO_B_MAX     = 0.300f;
    constexpr float AZUL_B_MIN     = 0.216f;
    constexpr float AZUL_R_MAX     = 0.390f;
    constexpr float AMARILLO_R_MIN = 0.416f;
    constexpr float AMARILLO_G_MIN = 0.350f;
    constexpr float AMARILLO_B_MAX = 0.250f;
}

ColorLabel Clasificar(const Tcs34725::Rgbc &s) {
    if (s.c < Umbral::CLEAR_NEGRO_MAX) return ColorLabel::NEGRO;

    const float total = (float)s.c;
    const float r = (float)s.r / total;
    const float g = (float)s.g / total;
    const float b = (float)s.b / total;

    if (r > Umbral::ROJO_R_MIN && g < Umbral::ROJO_G_MAX && b < Umbral::ROJO_B_MAX) return ColorLabel::ROJO;
    if (b > Umbral::AZUL_B_MIN && r < Umbral::AZUL_R_MAX) return ColorLabel::AZUL;
    if (r > Umbral::AMARILLO_R_MIN && g > Umbral::AMARILLO_G_MIN && b < Umbral::AMARILLO_B_MAX) return ColorLabel::AMARILLO;

    return ColorLabel::GRIS;
}

// ===========================================================================
//  [5] MOTORES — igual que firmware-esp32/ y 01-mantente-en-cuadro
// ===========================================================================

namespace Pwm {
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;
    constexpr uint8_t  MOTOR_RESOLUTION = 8;
    constexpr uint32_t RGB_FREQ_HZ      = 5000;
    constexpr uint8_t  RGB_RESOLUTION   = 8;
}

struct Motor {
    uint8_t in1, in2, en, ledc_channel;
};

// kMotorFL: con el cableado físico actual, la rueda conectada a OUT1/OUT2
// del L298N izquierdo gira al revés respecto a las otras tres — se
// compensa intercambiando el ORDEN de los dos GPIO aquí (ver el mismo
// comentario, con más detalle, en firmware-esp32/src/main.cpp).
constexpr Motor kMotorFL = {Pins::L298N_L_IN2, Pins::L298N_L_IN1, Pins::L298N_L_ENA, 0};
constexpr Motor kMotorRL = {Pins::L298N_L_IN3, Pins::L298N_L_IN4, Pins::L298N_L_ENB, 1};
constexpr Motor kMotorFR = {Pins::L298N_R_IN1, Pins::L298N_R_IN2, Pins::L298N_R_ENA, 2};
constexpr Motor kMotorRR = {Pins::L298N_R_IN3, Pins::L298N_R_IN4, Pins::L298N_R_ENB, 3};

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

void MotorSetup(const Motor &m) {
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
    PwmAttach(m.en, m.ledc_channel, Pwm::MOTOR_FREQ_HZ, Pwm::MOTOR_RESOLUTION);
    PwmWrite(m.en, m.ledc_channel, 0);
}

void MotorApply(const Motor &m, int speed) {
    speed = constrain(speed, -100, 100);
    const bool forward = (speed >= 0);
    digitalWrite(m.in1, forward ? HIGH : LOW);
    digitalWrite(m.in2, forward ? LOW  : HIGH);
    PwmWrite(m.en, m.ledc_channel, (uint32_t)abs(speed) * 255u / 100u);
}

void DriveSides(int leftSpeed, int rightSpeed) {
    MotorApply(kMotorFL, leftSpeed);
    MotorApply(kMotorRL, leftSpeed);
    MotorApply(kMotorFR, rightSpeed);
    MotorApply(kMotorRR, rightSpeed);
}

void MotorsStop() { DriveSides(0, 0); }

// ===========================================================================
//  [6] LED RGB — muestra la clasificación de color en vivo (diagnóstico)
// ===========================================================================

// El LED del robot es CÁTODO COMÚN (confirmado con el LED físico): duty
// alto = canal más brillante. Mismo valor que firmware-esp32/.
constexpr bool kCommonAnode = false;

namespace RgbLed {
    constexpr uint8_t CH_R = 0;
    constexpr uint8_t CH_G = 1;
    constexpr uint8_t CH_B = 2;

    void Setup() {
        PwmAttach(Pins::RGB_R, CH_R, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(Pins::RGB_G, CH_G, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(Pins::RGB_B, CH_B, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
    }

    void SetRaw(uint8_t r, uint8_t g, uint8_t b) {
        if (kCommonAnode) { r = 255 - r; g = 255 - g; b = 255 - b; }
        PwmWrite(Pins::RGB_R, CH_R, r);
        PwmWrite(Pins::RGB_G, CH_G, g);
        PwmWrite(Pins::RGB_B, CH_B, b);
    }

    void ApplyLabel(ColorLabel l) {
        switch (l) {
            case ColorLabel::ROJO:     SetRaw(255, 0, 0);   break;
            case ColorLabel::AZUL:     SetRaw(0, 0, 255);   break;
            case ColorLabel::AMARILLO: SetRaw(255, 190, 0); break;
            case ColorLabel::NEGRO:    SetRaw(0, 0, 0);     break;
            case ColorLabel::GRIS:
            default:                   SetRaw(40, 40, 40);  break;
        }
    }
}

// ===========================================================================
//  [7] SENSOR DE COLOR — lectura muestreada de los DOS TCS34725, de respaldo
// ===========================================================================
//
//  El TCS34725 integra a ~24 ms (ATIME_24MS); leerlo más seguido que eso
//  solo repite el mismo dato. Se muestrean los DOS sensores cada
//  COLOR_PERIOD_MS (igual que ColorSensorTask en firmware-esp32/, que usa
//  100 ms), con reintento perezoso si alguno no respondió.
//
//  g_colorConfirmaNegro es la única salida que le importa a la máquina de
//  estados — true solo cuando CUALQUIERA de los dos (delantero O trasero)
//  clasificó NEGRO durante kConfirmacionesColor muestreos SEGUIDOS. Ese
//  contador es más exigente que el del QTR (kConfirmacionesQtr) a
//  propósito: el color es la señal de RESPALDO, de menor prioridad —
//  tarda más en convencerse antes de disparar nada por su cuenta.

constexpr uint32_t COLOR_PERIOD_MS = 100;
constexpr uint8_t  kConfirmacionesColor = 3;   // ~3 x 100 ms = 300 ms sostenidos

bool g_frontOk = false;
bool g_backOk  = false;
bool g_colorConfirmaNegro = false;

// Para telemetría únicamente (no alimentan la máquina de estados salvo a
// través de g_colorConfirmaNegro).
bool g_frontValido = false, g_backValido = false;
ColorLabel g_frontLabel = ColorLabel::GRIS, g_backLabel = ColorLabel::GRIS;
Tcs34725::Rgbc g_frontRaw, g_backRaw;

uint8_t g_confirmColorNegro = 0;

bool Confirmar(bool crudo, uint8_t &contador, uint8_t necesarias) {
    if (crudo) {
        if (contador < necesarias) contador++;
    } else {
        contador = 0;
    }
    return contador >= necesarias;
}

void ActualizarColor() {
    static uint32_t lastReadMs = 0;
    static uint32_t lastRetryMs = 0;
    const uint32_t now = millis();

    if ((uint32_t)(now - lastReadMs) < COLOR_PERIOD_MS) return;
    lastReadMs = now;

    if ((!g_frontOk || !g_backOk) && (uint32_t)(now - lastRetryMs) > 1000) {
        lastRetryMs = now;
        if (!g_frontOk) g_frontOk = Tcs34725::Init(Wire);
        if (!g_backOk)  g_backOk  = Tcs34725::Init(Wire1);
    }

    g_frontValido = g_frontOk && Tcs34725::Read(Wire, g_frontRaw);
    if (g_frontOk && !g_frontValido) g_frontOk = false;
    if (g_frontValido) g_frontLabel = Clasificar(g_frontRaw);

    g_backValido = g_backOk && Tcs34725::Read(Wire1, g_backRaw);
    if (g_backOk && !g_backValido) g_backOk = false;
    if (g_backValido) g_backLabel = Clasificar(g_backRaw);

    // SOLO importa NEGRO — cualquier otra clasificación se ignora por
    // completo para esta decisión (ver encabezado del archivo).
    const bool crudoNegro =
        (g_frontValido && g_frontLabel == ColorLabel::NEGRO) ||
        (g_backValido  && g_backLabel  == ColorLabel::NEGRO);
    g_colorConfirmaNegro = Confirmar(crudoNegro, g_confirmColorNegro, kConfirmacionesColor);

}

// ===========================================================================
//  [8] MÁQUINA DE ESTADOS — QTR con prioridad, color como respaldo
// ===========================================================================
//
//  DRIVING:   avanza recto. Entra a REVERSING si:
//               - el QTR (izquierdo O derecho) confirma negro (rápido,
//                 kConfirmacionesQtr) -- esto SOLO decide el QTR; o
//               - el color todavía no confirmó nada por QTR, pero confirma
//                 negro por su cuenta (más lento, kConfirmacionesColor) --
//                 red de seguridad si el QTR fallara o estuviera mal
//                 calibrado.
//  REVERSING: retrocede en línea recta durante kReverseMs (~1.5 s),
//             cronometrado, a ciegas -- no mira los sensores. Al cumplirse
//             el tiempo, sortea un lado al azar (50/50) y pasa a TURNING.
//  TURNING:   gira sobre su propio eje hacia el lado sorteado (un lado
//             adelante, el otro atrás -- NUNCA avanza ni retrocede en este
//             estado). Vuelve a DRIVING SOLO cuando el QTR confirma que ya
//             no hay negro -- el color no participa en esta decisión, ni
//             para bien ni para mal: tiene prioridad más baja, así que no
//             puede vetar ni acelerar la salida. Sin temporizador: dura lo
//             que el sensor diga, no un número copiado de otra prueba.

enum class State : uint8_t { DRIVING, REVERSING, TURNING };

constexpr int kForwardSpeed = 55;
constexpr int kReverseSpeed = -55;   // mismo duty que adelante, sentido invertido

// Mismo razonamiento que en 01-mantente-en-cuadro: un giro en el sitio con
// 4 ruedas motrices necesita mucho torque (las 4 raspan contra el piso).
// Va al máximo duty a propósito.
constexpr int kTurnSpeed = 100;

// Único temporizador fijo del archivo: cuánto dura la reversa. A
// diferencia del giro (reactivo, ver TURNING arriba), retroceder no tiene
// una señal de sensor que le diga "ya retrocediste lo suficiente" -- así
// que, igual que en 01-mantente-en-cuadro, es de lazo abierto.
constexpr uint32_t kReverseMs = 1500;

// Confirmación del QTR — rápida, es la señal CON prioridad. Mismo criterio
// que 01-mantente-en-cuadro: ~20 ms de negro sostenido (4 lecturas a
// ~5 ms/vuelta) alcanza para confiar, poco como para avanzar de más.
constexpr uint8_t kConfirmacionesQtr = 4;

uint8_t g_confirmQtrNegro = 0;
uint8_t g_confirmQtrClaro = 0;

State g_state = State::DRIVING;
uint32_t g_stateEnteredAtMs = 0;
bool g_giroDerecha = false;   // sorteado al entrar a TURNING, ver REVERSING

void EnterState(State s) {
    g_state = s;
    g_stateEnteredAtMs = millis();
}

const char *NombreEstado(State s) {
    switch (s) {
        case State::DRIVING:   return "DRIVING";
        case State::REVERSING: return "REVERSING";
        case State::TURNING:   return "TURNING";
    }
    return "?";
}

void RunStateMachine(bool qtrRawNegro) {
    switch (g_state) {
        case State::DRIVING: {
            const bool qtrConfirma = Confirmar(qtrRawNegro, g_confirmQtrNegro, kConfirmacionesQtr);

            if (qtrConfirma || g_colorConfirmaNegro) {
                DEBUG_LINK.printf(
                    "[Borde] negro confirmado (qtr=%d color_respaldo=%d) -> REVERSING\n",
                    qtrConfirma, g_colorConfirmaNegro);
                g_confirmQtrClaro = 0;
                g_confirmColorNegro = 0;   // arranca fresca, no re-dispara de inmediato al volver
                EnterState(State::REVERSING);
            } else {
                DriveSides(kForwardSpeed, kForwardSpeed);
            }
            break;
        }

        case State::REVERSING: {
            DriveSides(kReverseSpeed, kReverseSpeed);
            if ((uint32_t)(millis() - g_stateEnteredAtMs) >= kReverseMs) {
                g_giroDerecha = (esp_random() % 2) == 0;   // 50/50, RNG de hardware del ESP32
                DEBUG_LINK.printf("[Borde] reversa completa -> giro al azar hacia la %s\n",
                    g_giroDerecha ? "DERECHA" : "IZQUIERDA");
                EnterState(State::TURNING);
            }
            break;
        }

        case State::TURNING: {
            // Pivote: un lado adelante, el otro atrás -- el sentido lo
            // decide g_giroDerecha, sorteado una sola vez al entrar aquí
            // (no se vuelve a sortear en cada vuelta de loop()).
            if (g_giroDerecha) {
                DriveSides(kTurnSpeed, -kTurnSpeed);
            } else {
                DriveSides(-kTurnSpeed, kTurnSpeed);
            }

            // SOLO el QTR decide la salida -- el color no participa (tiene
            // prioridad más baja, ver encabezado).
            const bool qtrDespejado = Confirmar(!qtrRawNegro, g_confirmQtrClaro, kConfirmacionesQtr);
            if (qtrDespejado) {
                DEBUG_LINK.println("[Borde] despejado (QTR) -> DRIVING");
                g_confirmQtrNegro = 0;
                EnterState(State::DRIVING);
            }
            break;
        }
    }
}

// ---------------------------------------------------------------------------
//  LED RGB de estado
// ---------------------------------------------------------------------------
//  Con un solo LED no se puede mostrar a la vez el color del piso y que hay
//  una maniobra de borde en curso. Manda la maniobra: es el evento, y el
//  color se vuelve a ver apenas termina. El destello ROJO/AZUL alterno es el
//  mismo código de alerta que usa firmware-esp32-standalone/, y no se
//  confunde con ningún color de piso porque esos se muestran fijos.
//
//  Se refresca desde loop() y no dentro de las transiciones, para que el
//  destello siga vivo mientras dure la maniobra completa (reversa + giro).

void ActualizarRgb() {
    if (g_state != State::DRIVING) {
        if (((millis() / 125) % 2) == 0) RgbLed::SetRaw(255, 0, 0);
        else                             RgbLed::SetRaw(0, 0, 255);
        return;
    }
    RgbLed::SetRaw(0, 0, 0);
    if (g_frontValido) RgbLed::ApplyLabel(g_frontLabel);   // diagnóstico: solo el delantero
}

// ===========================================================================
//  [9] setup() / loop()
// ===========================================================================

void setup() {
    DEBUG_LINK.begin(115200);
    delay(200);
    DEBUG_LINK.println("\nPrueba 05 - Evitador de linea (QTR con prioridad + color de respaldo)");


    // Reflectancia: igual que firmware-esp32 (ADC de 12 bits, atenuación
    // 11 dB para cubrir la excursión completa del QTR a 3.3 V).
    analogReadResolution(12);
    analogSetPinAttenuation(Pins::QTR_LEFT_OUT, ADC_11db);
    analogSetPinAttenuation(Pins::QTR_RIGHT_OUT, ADC_11db);
    pinMode(Pins::QTR_EMITTER_CTRL, OUTPUT);
    digitalWrite(Pins::QTR_EMITTER_CTRL, HIGH);

    // Color: los dos buses I2C + LED de iluminación propios, encendidos
    // fijos (no depender de la luz del salón, mismo motivo que
    // firmware-esp32/).
    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);
    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    pinMode(Pins::TCS_LED_BACK, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);
    digitalWrite(Pins::TCS_LED_BACK, HIGH);
    g_frontOk = Tcs34725::Init(Wire);
    g_backOk  = Tcs34725::Init(Wire1);
    if (!g_frontOk) DEBUG_LINK.println("[Setup] TCS34725 delantero no responde. Reintentando en segundo plano.");
    if (!g_backOk)  DEBUG_LINK.println("[Setup] TCS34725 trasero no responde. Reintentando en segundo plano.");

    RgbLed::Setup();
    RgbLed::SetRaw(0, 0, 0);

    MotorSetup(kMotorFL);
    MotorSetup(kMotorRL);
    MotorSetup(kMotorFR);
    MotorSetup(kMotorRR);
    MotorsStop();

    RunCalibration();

    DEBUG_LINK.println("[Setup] listo, arrancando en DRIVING.");
}

void loop() {
    const uint16_t left  = (uint16_t)analogRead(Pins::QTR_LEFT_OUT);
    const uint16_t right = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);
    const bool qtrRawNegro = (left > g_leftThreshold) || (right > g_rightThreshold);

    ActualizarColor();          // no-op si todavia no toca (ver COLOR_PERIOD_MS)
    ActualizarRgb();
    RunStateMachine(qtrRawNegro);

    static uint32_t lastPrint = 0;
    if ((uint32_t)(millis() - lastPrint) > 200) {
        lastPrint = millis();
        DEBUG_LINK.printf(
            "izq=%u der=%u qtr=%s(%u/%u)  del=%s tra=%s color_respaldo=%s(%u/%u)  estado=%s\n",
            left, right, qtrRawNegro ? "NEGRO" : "gris", g_confirmQtrNegro, kConfirmacionesQtr,
            g_frontValido ? LabelName(g_frontLabel) : "?",
            g_backValido ? LabelName(g_backLabel) : "?",
            g_colorConfirmaNegro ? "NEGRO" : "-", g_confirmColorNegro, kConfirmacionesColor,
            NombreEstado(g_state));
    }

    delay(5);
}
