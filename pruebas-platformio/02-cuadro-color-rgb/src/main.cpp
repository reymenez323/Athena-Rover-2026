// ===========================================================================
//  Prueba 02 — Cuadro + color + RGB
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  OBJETIVO: combina la prueba 01 (mantenerse dentro del cuadro con los 2
//  QTR) con los 2 TCS34725: el robot avanza sin cruzar la cinta negra (igual
//  que 01, código sin tocar) y, además, clasifica el color que ve cada
//  sensor (delantero y trasero) mientras pasa por encima de las zonas de la
//  pista. Como solo hay UN LED RGB en el chasis, se usa para mostrar el
//  color que está leyendo el sensor DELANTERO en este instante.
//
//  Es un sketch de banco, deliberadamente simple (setup/loop, sin FreeRTOS
//  ni colas), independiente del firmware principal y de la prueba 01: sirve
//  para validar delante de la pista real el driver TCS34725 + el RGB antes
//  de integrarlos a las 7 tareas de firmware-esp32/src/main.cpp.
//
//  SOBRE LOS UMBRALES DE COLOR:
//  Se reutiliza tal cual la función ClassifyColor() de firmware-esp32/src/
//  main.cpp: normaliza cada canal (R, G, B) contra el canal C (luz total),
//  así la decisión no depende del brillo absoluto del salón. Los umbrales
//  son el mismo punto de partida documentado allá, NO valores calibrados —
//  hay que ajustarlos aquí mismo con la telemetría de este sketch sobre la
//  pista real.
//
//  SOBRE EL DATASET KNN DE UN COMPAÑERO (parametros_mejor_modelo.h):
//  NO se usa en esta prueba. Ese modelo fue entrenado con SU TCS34725, a
//  SU altura sobre la pista (~5 mm) y con SU intensidad de LED — el TCS34725
//  normaliza contra el canal C, así que cualquier diferencia de altura,
//  intensidad de LED o iluminación ambiente desplaza esos números. Reusar
//  sus 500 muestras y su media/desviación estándar tal cual probablemente
//  clasificaría mal con nuestro sensor. Si más adelante hace falta más
//  precisión que la de umbrales fijos, lo reutilizable de ese archivo es el
//  ALGORITMO (features Rn/Gn/Bn/C, estandarización, votación KNN) — pero
//  recapturando el dataset con NUESTRO sensor montado en NUESTRO chasis.
//
//  SOBRE EL UMBRAL DE "esto es cinta negra" (QTR, igual que en 01):
//  Los logs de calibración que trajo el equipo (IR_BLACK / IR_GREY) muestran
//  que un número fijo es frágil, así que este sketch CALIBRA EN CADA
//  ARRANQUE (ver [7]) en vez de confiar en una constante copiada del log.
//  Umbral de reserva si el arranque no calibra: 3700.
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>

// ===========================================================================
//  [1] PINES — idénticos a hardware/conexiones-esp32-s3.md y a firmware-esp32
// ===========================================================================

namespace Pins {
    // Reflectancia QTRX-HD-01A. ¡A 3.3 V, nunca a 5 V!
    constexpr uint8_t QTR_LEFT_OUT      = 1;   // ADC1_CH0
    constexpr uint8_t QTR_RIGHT_OUT     = 2;   // ADC1_CH1
    constexpr uint8_t QTR_EMITTER_CTRL  = 42;  // enciende los LED IR de ambos sensores

    // Motores: 2x L298N (quitar los jumpers de ENA/ENB en ambos).
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

    // LED de equipo, reutilizados como indicador de calibración y de qué
    // lado disparó el borde — igual que en la prueba 01.
    constexpr uint8_t LED_TEAM_RED  = 40;   // se enciende cuando dispara el sensor IZQUIERDO
    constexpr uint8_t LED_TEAM_BLUE = 41;   // se enciende cuando dispara el sensor DERECHO

    // -------- Bus I2C nº0: TCS34725 DELANTERO (mismo bus que el PCA9685 en
    // el firmware principal, aquí no se usa el PCA9685) --------------------
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;

    // -------- Bus I2C nº1: TCS34725 TRASERO --------------------------------
    // Los dos TCS34725 comparten la misma dirección fija (0x29), por eso
    // van en buses I2C separados en vez de compartir uno solo.
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    // LED de iluminación de cada TCS34725 (activo en alto).
    constexpr uint8_t TCS_LED_FRONT = 18;
    constexpr uint8_t TCS_LED_BACK  = 21;

    // -------- LED RGB indicador del color delantero ------------------------
    // No está en hardware/conexiones-esp32-s3.md todavía (es nuevo en esta
    // prueba). Usa los ÚNICOS 3 GPIO que la doc marca libres para
    // ampliaciones ("Resumen: mapa completo de pines usados"): 38, 39 y 3.
    // GPIO 3 es strapping de JTAG: solo importa su nivel en el instante de
    // encender/resetear, como salida normal después de bootear no da
    // problema.
    constexpr uint8_t RGB_R = 39;
    constexpr uint8_t RGB_G = 38;
    constexpr uint8_t RGB_B = 3;
}

namespace I2CAddr {
    constexpr uint8_t TCS34725 = 0x29;   // fija, no se puede cambiar
}

#define DEBUG_LINK Serial0   // consola por el puerto UART del DevKit

// ===========================================================================
//  [2] DRIVER TCS34725 — igual que firmware-esp32/src/main.cpp, sin cambios
// ===========================================================================
//
//  Los dos sensores son idénticos y comparten la dirección 0x29 (fija), por
//  eso cada uno vive en su propio bus (Wire y Wire1) y este driver recibe el
//  bus como parámetro.

namespace Tcs34725 {
    constexpr uint8_t CMD_BIT      = 0x80;   // todo acceso a registro lleva este bit
    constexpr uint8_t CMD_AUTO_INC = 0x20;

    constexpr uint8_t REG_ENABLE  = 0x00;
    constexpr uint8_t REG_ATIME   = 0x01;
    constexpr uint8_t REG_CONTROL = 0x0F;
    constexpr uint8_t REG_ID      = 0x12;
    constexpr uint8_t REG_CDATAL  = 0x14;   // luego R, G, B consecutivos

    constexpr uint8_t ENABLE_PON = 0x01;    // enciende el oscilador interno
    constexpr uint8_t ENABLE_AEN = 0x02;    // habilita el conversor RGBC

    constexpr uint8_t ATIME_24MS = 0xEB;    // 24 ms de integración
    constexpr uint8_t GAIN_4X    = 0x01;    // 0=1x, 1=4x, 2=16x, 3=60x

    struct Rgbc { uint16_t c, r, g, b; };

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
        // 0x44 = TCS34725, 0x4D = TCS34727. Cualquier otra cosa no es el sensor.
        if (id != 0x44 && id != 0x4D) return false;

        if (!WriteReg(bus, REG_ATIME, ATIME_24MS)) return false;
        if (!WriteReg(bus, REG_CONTROL, GAIN_4X)) return false;
        if (!WriteReg(bus, REG_ENABLE, ENABLE_PON)) return false;
        delay(3);                                   // arranque del oscilador
        return WriteReg(bus, REG_ENABLE, ENABLE_PON | ENABLE_AEN);
    }

    bool Read(TwoWire &bus, Rgbc &out) {
        bus.beginTransmission(I2CAddr::TCS34725);
        bus.write(CMD_BIT | CMD_AUTO_INC | REG_CDATAL);
        if (bus.endTransmission() != 0) return false;
        if (bus.requestFrom((int)I2CAddr::TCS34725, 8) != 8) return false;

        // Los cuatro canales llegan como uint16 little-endian, en orden C R G B.
        out.c = (uint16_t)(bus.read() | (bus.read() << 8));
        out.r = (uint16_t)(bus.read() | (bus.read() << 8));
        out.g = (uint16_t)(bus.read() | (bus.read() << 8));
        out.b = (uint16_t)(bus.read() | (bus.read() << 8));
        return true;
    }
}

// ===========================================================================
//  [3] CLASIFICACIÓN DE COLOR — igual que firmware-esp32/src/main.cpp
// ===========================================================================
//
//  TODO IMPORTANTE (heredado del firmware): estos umbrales son un punto de
//  partida razonable, NO valores calibrados. Ajustarlos aquí mismo con la
//  telemetría cruda que este sketch imprime por Serial0, sobre la pista real.

enum class ColorLabel : uint8_t { UNKNOWN = 0, BLACK, YELLOW, RED, BLUE, FLOOR };

const char *ColorLabelName(ColorLabel c) {
    switch (c) {
        case ColorLabel::BLACK:  return "NEGRO";
        case ColorLabel::YELLOW: return "AMARILLO";
        case ColorLabel::RED:    return "ROJO";
        case ColorLabel::BLUE:   return "AZUL";
        case ColorLabel::FLOOR:  return "GRIS(piso)";
        default:                 return "?";
    }
}

static ColorLabel ClassifyColor(const Tcs34725::Rgbc &s) {
    // Muy poca luz reflejada = cinta negra (o el sensor mirando al vacío).
    if (s.c < 300) return ColorLabel::BLACK;

    const float total = (float)s.c;
    const float r = (float)s.r / total;
    const float g = (float)s.g / total;
    const float b = (float)s.b / total;

    if (r > 0.45f && g < 0.30f && b < 0.30f) return ColorLabel::RED;
    if (b > 0.40f && r < 0.30f)              return ColorLabel::BLUE;
    if (r > 0.35f && g > 0.35f && b < 0.25f) return ColorLabel::YELLOW;

    // Canales parejos = superficie gris: el tapete de la pista.
    return ColorLabel::FLOOR;
}

// ===========================================================================
//  [4] LED RGB — muestra el color que lee el sensor DELANTERO
// ===========================================================================
//
//  Un solo LED RGB en todo el chasis, así que solo el sensor delantero lo
//  controla (el trasero solo reporta por Serial0, ver [8]).
//
//  POLARIDAD SIN CONFIRMAR: se asume cátodo común (duty alto = canal más
//  brillante). Si al probarlo los colores salen invertidos (por ejemplo,
//  "apagado" se ve más brillante que "rojo"), es ánodo común: cambiar
//  kRgbCommonAnode a true abajo, no hace falta tocar el resto del código.

constexpr bool kRgbCommonAnode = false;

namespace Pwm {
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;   // el L298N calienta y pierde par a 20 kHz
    constexpr uint8_t  MOTOR_RESOLUTION = 8;      // duty 0..255

    constexpr uint32_t RGB_FREQ_HZ      = 5000;   // fuera del rango audible, sin parpadeo visible
    constexpr uint8_t  RGB_RESOLUTION   = 8;      // duty 0..255
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
    // Canales LEDC 4..6: los motores ya ocupan 0..3 (ver [5]).
    constexpr uint8_t CH_R = 4;
    constexpr uint8_t CH_G = 5;
    constexpr uint8_t CH_B = 6;

    void Setup() {
        PwmAttach(Pins::RGB_R, CH_R, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(Pins::RGB_G, CH_G, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(Pins::RGB_B, CH_B, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
    }

    void SetRaw(uint8_t r, uint8_t g, uint8_t b) {
        if (kRgbCommonAnode) { r = 255 - r; g = 255 - g; b = 255 - b; }
        PwmWrite(Pins::RGB_R, CH_R, r);
        PwmWrite(Pins::RGB_G, CH_G, g);
        PwmWrite(Pins::RGB_B, CH_B, b);
    }

    // Un color fijo por etiqueta. GRIS se muestra como blanco tenue (no
    // apagado del todo) para poder distinguirlo de NEGRO/UNKNOWN a simple vista.
    void ApplyLabel(ColorLabel label) {
        switch (label) {
            case ColorLabel::RED:    SetRaw(255,   0,   0); break;
            case ColorLabel::BLUE:   SetRaw(  0,   0, 255); break;
            case ColorLabel::YELLOW: SetRaw(255, 255,   0); break;
            case ColorLabel::FLOOR:  SetRaw( 50,  50,  50); break;
            case ColorLabel::BLACK:
            case ColorLabel::UNKNOWN:
            default:                 SetRaw(  0,   0,   0); break;
        }
    }
}

// ===========================================================================
//  [5] MOTORES — igual que la prueba 01
// ===========================================================================

struct Motor {
    uint8_t in1, in2, en, ledc_channel;
};

// kMotorFL: con el cableado físico actual, la rueda conectada a OUT1/OUT2
// del L298N izquierdo (la trasera izquierda del chasis) gira al revés
// respecto a las otras tres. En vez de ramificar MotorApply() con un flag de
// inversión, se intercambia el ORDEN de los dos GPIO aquí mismo: Pins::
// L298N_L_IN1 y L298N_L_IN2 siguen siendo, físicamente, los pines soldados a
// los terminales IN1 e IN2 del L298N (ver hardware/conexiones-esp32-s3.md) —
// lo único que cambia es cuál de los dos hace de "in1" y cuál de "in2" para
// ESTE motor, así que MotorApply() queda idéntica para los 4 motores. Si se
// recablea este motor para que coincida con los otros tres, basta con
// volver a poner IN1, IN2 en orden aquí.
constexpr Motor kMotorFL = {Pins::L298N_L_IN2, Pins::L298N_L_IN1, Pins::L298N_L_ENA, 0};
constexpr Motor kMotorRL = {Pins::L298N_L_IN3, Pins::L298N_L_IN4, Pins::L298N_L_ENB, 1};
constexpr Motor kMotorFR = {Pins::L298N_R_IN1, Pins::L298N_R_IN2, Pins::L298N_R_ENA, 2};
constexpr Motor kMotorRR = {Pins::L298N_R_IN3, Pins::L298N_R_IN4, Pins::L298N_R_ENB, 3};

void MotorSetup(const Motor &m) {
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
    PwmAttach(m.en, m.ledc_channel, Pwm::MOTOR_FREQ_HZ, Pwm::MOTOR_RESOLUTION);
    PwmWrite(m.en, m.ledc_channel, 0);
}

// speed: -100..100. El signo define el sentido, la magnitud el duty.
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
//  [6] CALIBRACIÓN QTR EN ARRANQUE — igual que la prueba 01
// ===========================================================================

constexpr uint32_t CALIBRATION_MS     = 3000;
constexpr uint16_t MIN_USABLE_SPAN    = 300;    // por debajo de esto, no hubo contraste real
constexpr uint16_t FALLBACK_THRESHOLD = 3700;   // ver análisis del log en la prueba 01

uint16_t g_leftThreshold  = FALLBACK_THRESHOLD;
uint16_t g_rightThreshold = FALLBACK_THRESHOLD;

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
            digitalWrite(Pins::LED_TEAM_RED, blinkState ? HIGH : LOW);
            digitalWrite(Pins::LED_TEAM_BLUE, blinkState ? HIGH : LOW);
        }
        delay(5);
    }

    digitalWrite(Pins::LED_TEAM_RED, LOW);
    digitalWrite(Pins::LED_TEAM_BLUE, LOW);

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
//  [7] MÁQUINA DE ESTADOS — evasión de borde, igual que la prueba 01
// ===========================================================================

enum class State : uint8_t { FORWARD, BACK_UP, TURN };

constexpr int kForwardSpeed = 55;
constexpr int kBackSpeed    = 55;
constexpr int kTurnSpeed    = 60;

constexpr uint32_t kBackUpMs      = 300;
constexpr uint32_t kTurnMs        = 400;   // giro simple, un solo lado disparó
constexpr uint32_t kTurnCornerMs  = 700;   // giro largo, los dos lados dispararon (esquina)

enum class Edge : uint8_t { NONE, LEFT, RIGHT, BOTH };

State g_state = State::FORWARD;
Edge g_triggeredBy = Edge::NONE;
uint32_t g_stateStartMs = 0;

void EnterState(State s, Edge trigger = Edge::NONE) {
    g_state = s;
    g_stateStartMs = millis();
    if (trigger != Edge::NONE) g_triggeredBy = trigger;
}

void UpdateEdgeLeds() {
    digitalWrite(Pins::LED_TEAM_RED,  (g_state != State::FORWARD &&
        (g_triggeredBy == Edge::LEFT  || g_triggeredBy == Edge::BOTH)) ? HIGH : LOW);
    digitalWrite(Pins::LED_TEAM_BLUE, (g_state != State::FORWARD &&
        (g_triggeredBy == Edge::RIGHT || g_triggeredBy == Edge::BOTH)) ? HIGH : LOW);
}

void RunStateMachine(bool leftOnLine, bool rightOnLine) {
    switch (g_state) {
        case State::FORWARD: {
            if (leftOnLine && rightOnLine) {
                DEBUG_LINK.println("[Borde] los dos sensores a la vez -> esquina");
                EnterState(State::BACK_UP, Edge::BOTH);
            } else if (leftOnLine) {
                DEBUG_LINK.println("[Borde] izquierdo -> giro a la derecha");
                EnterState(State::BACK_UP, Edge::LEFT);
            } else if (rightOnLine) {
                DEBUG_LINK.println("[Borde] derecho -> giro a la izquierda");
                EnterState(State::BACK_UP, Edge::RIGHT);
            } else {
                DriveSides(kForwardSpeed, kForwardSpeed);
            }
            break;
        }

        case State::BACK_UP: {
            DriveSides(-kBackSpeed, -kBackSpeed);
            if ((uint32_t)(millis() - g_stateStartMs) > kBackUpMs) {
                EnterState(State::TURN);
            }
            break;
        }

        case State::TURN: {
            if (g_triggeredBy == Edge::LEFT) {
                DriveSides(kTurnSpeed, -kTurnSpeed);
            } else if (g_triggeredBy == Edge::RIGHT) {
                DriveSides(-kTurnSpeed, kTurnSpeed);
            } else {
                DriveSides(kTurnSpeed, -kTurnSpeed);
            }

            const uint32_t limit = (g_triggeredBy == Edge::BOTH) ? kTurnCornerMs : kTurnMs;
            if ((uint32_t)(millis() - g_stateStartMs) > limit) {
                g_triggeredBy = Edge::NONE;
                EnterState(State::FORWARD);
            }
            break;
        }
    }

    UpdateEdgeLeds();
}

// ===========================================================================
//  [8] LECTURA PERIÓDICA DE LOS 2 TCS34725
// ===========================================================================
//
//  No se lee en cada vuelta del loop (5 ms): el TCS34725 integra 24 ms y una
//  transacción I2C completa de por sí toma tiempo, así que leer más rápido
//  que esto solo repite el mismo dato. Se lee cada COLOR_READ_MS y, si el
//  sensor no respondió en setup(), se reintenta el Init() aquí mismo (útil
//  si se conecta el cable I2C después de encender).

constexpr uint32_t COLOR_READ_MS = 100;

bool g_frontOk = false;
bool g_backOk  = false;
ColorLabel g_frontLabel = ColorLabel::UNKNOWN;
ColorLabel g_backLabel  = ColorLabel::UNKNOWN;
Tcs34725::Rgbc g_frontRaw{};
Tcs34725::Rgbc g_backRaw{};

void UpdateColorSensors() {
    static uint32_t lastRead = 0;
    if ((uint32_t)(millis() - lastRead) < COLOR_READ_MS) return;
    lastRead = millis();

    if (!g_frontOk) g_frontOk = Tcs34725::Init(Wire);
    if (!g_backOk)  g_backOk  = Tcs34725::Init(Wire1);

    if (g_frontOk) {
        if (Tcs34725::Read(Wire, g_frontRaw)) {
            g_frontLabel = ClassifyColor(g_frontRaw);
        } else {
            g_frontOk = false;   // se cayó a media prueba, se reintenta Init() arriba
        }
    }

    if (g_backOk) {
        if (Tcs34725::Read(Wire1, g_backRaw)) {
            g_backLabel = ClassifyColor(g_backRaw);
        } else {
            g_backOk = false;
        }
    }

    RgbLed::ApplyLabel(g_frontOk ? g_frontLabel : ColorLabel::UNKNOWN);
}

// ===========================================================================
//  [9] setup() / loop()
// ===========================================================================

void setup() {
    DEBUG_LINK.begin(115200);
    delay(200);
    DEBUG_LINK.println("\nPrueba 02 - Cuadro + color + RGB");

    pinMode(Pins::LED_TEAM_RED, OUTPUT);
    pinMode(Pins::LED_TEAM_BLUE, OUTPUT);

    // Reflectancia: ADC de 12 bits, atenuación 11 dB para cubrir la
    // excursión completa del QTR a 3.3 V.
    analogReadResolution(12);
    analogSetPinAttenuation(Pins::QTR_LEFT_OUT, ADC_11db);
    analogSetPinAttenuation(Pins::QTR_RIGHT_OUT, ADC_11db);
    pinMode(Pins::QTR_EMITTER_CTRL, OUTPUT);
    digitalWrite(Pins::QTR_EMITTER_CTRL, HIGH);

    MotorSetup(kMotorFL);
    MotorSetup(kMotorRL);
    MotorSetup(kMotorFR);
    MotorSetup(kMotorRR);
    MotorsStop();

    RgbLed::Setup();
    RgbLed::SetRaw(0, 0, 0);

    // Bus I2C nº0 (delantero) y nº1 (trasero) — igual que firmware-esp32.
    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);

    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    pinMode(Pins::TCS_LED_BACK, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);
    digitalWrite(Pins::TCS_LED_BACK, HIGH);

    g_frontOk = Tcs34725::Init(Wire);
    g_backOk  = Tcs34725::Init(Wire1);
    if (!g_frontOk) DEBUG_LINK.println("[Color] sensor DELANTERO no responde (bus I2C 0).");
    if (!g_backOk)  DEBUG_LINK.println("[Color] sensor TRASERO no responde (bus I2C 1).");

    RunCalibration();

    DEBUG_LINK.println("[Setup] listo, arrancando en FORWARD.");
    g_stateStartMs = millis();
}

void loop() {
    const uint16_t left  = (uint16_t)analogRead(Pins::QTR_LEFT_OUT);
    const uint16_t right = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);

    const bool leftOnLine  = left  > g_leftThreshold;
    const bool rightOnLine = right > g_rightThreshold;

    RunStateMachine(leftOnLine, rightOnLine);
    UpdateColorSensors();

    // Telemetría de banco: bordes + color de los dos sensores. Se puede
    // comentar una vez calibrado a gusto.
    static uint32_t lastPrint = 0;
    if ((uint32_t)(millis() - lastPrint) > 200) {
        lastPrint = millis();
        DEBUG_LINK.printf(
            "izq=%u(%s) der=%u(%s) estado=%d | "
            "color_delant=%s [R=%u G=%u B=%u C=%u]%s | "
            "color_tras=%s [R=%u G=%u B=%u C=%u]%s\n",
            left, leftOnLine ? "NEGRO" : "gris",
            right, rightOnLine ? "NEGRO" : "gris",
            (int)g_state,
            ColorLabelName(g_frontLabel), g_frontRaw.r, g_frontRaw.g, g_frontRaw.b, g_frontRaw.c,
            g_frontOk ? "" : " (SIN RESPUESTA)",
            ColorLabelName(g_backLabel), g_backRaw.r, g_backRaw.g, g_backRaw.b, g_backRaw.c,
            g_backOk ? "" : " (SIN RESPUESTA)");
    }

    delay(5);
}
