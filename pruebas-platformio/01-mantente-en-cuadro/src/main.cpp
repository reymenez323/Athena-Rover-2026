// ===========================================================================
//  Prueba 01 — Mantente en el cuadro
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  OBJETIVO: el robot avanza dentro del cuadrado delimitado por la cinta
//  negra, siempre en el mismo sentido de avance que la prueba
//  03-motores-adelante (nunca gira, nunca retrocede), y en cuanto el sensor
//  de reflectancia DERECHO detecta el borde (fondo gris -> cinta negra),
//  PARA de golpe y se queda detenido. No intenta esquivar la cinta ni
//  retomar la marcha por su cuenta.
//
//  Decide SOLO el sensor derecho — es el que confirmamos que mide bien. El
//  izquierdo se sigue leyendo e imprimiendo como referencia, pero no
//  participa en la decisión (ver [4]).
//
//  Es un sketch de banco, deliberadamente simple (setup/loop, sin FreeRTOS
//  ni colas): sirve para validar el comportamiento de borde ANTES de
//  integrarlo a las 7 tareas de firmware-esp32/src/main.cpp. Usa los MISMOS
//  pines que el firmware principal (ver hardware/conexiones-esp32-s3.md) así
//  que corre tal cual sobre el chasis ya cableado.
//
//  SOBRE EL UMBRAL DE "esto es cinta negra":
//  Los logs de calibración que trajo el equipo (IR_BLACK / IR_GREY) muestran
//  que un número fijo es frágil: BLACK promedia ~4060 (satura cerca de 4095,
//  desviación ~135), pero GREY varía muchísimo según el punto medido —de
//  ~3400 a ~3960— y ese máximo de GREY se mete dentro del rango de BLACK.
//  Con el sensor sostenido a mano a distintas alturas entre puntos, es
//  esperable: la reflectancia que ve el QTR depende muchísimo de la
//  distancia a la superficie, no solo del color. Sobre el chasis, a altura
//  fija, el contraste real debería ser mucho más limpio — pero para no
//  apostarlo todo a eso, este sketch CALIBRA EN CADA ARRANQUE (ver [2]) en
//  vez de confiar en una constante copiada del log.
//
//  Umbral de reserva (si el arranque no calibra, ver [2]): 3700 — el punto
//  medio razonable entre el grueso de GREY (<3800 en 5 de 6 puntos medidos)
//  y el piso de BLACK (>4030 en los 4 puntos medidos).
//
// ===========================================================================

#include <Arduino.h>

// ===========================================================================
//  [1] PINES — idénticos a hardware/conexiones-esp32-s3.md y a firmware-esp32
// ===========================================================================

namespace Pins {
    // Reflectancia QTRX-HD-01A. ¡A 3.3 V, nunca a 5 V! (ver la tabla de
    // riesgos en hardware/conexiones-esp32-s3.md).
    constexpr uint8_t QTR_LEFT_OUT      = 1;   // ADC1_CH0
    constexpr uint8_t QTR_RIGHT_OUT     = 2;   // ADC1_CH1
    constexpr uint8_t QTR_EMITTER_CTRL  = 42;  // enciende los LED IR de ambos sensores

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

    // LED de equipo, reutilizados aquí como indicador de calibración y de
    // qué lado disparó el borde (no hace falta laptop conectada para ver si
    // el sketch está funcionando).
    constexpr uint8_t LED_TEAM_RED  = 40;   // se enciende cuando dispara el sensor IZQUIERDO
    constexpr uint8_t LED_TEAM_BLUE = 41;   // se enciende cuando dispara el sensor DERECHO
}

#define DEBUG_LINK Serial0   // consola por el puerto UART del DevKit

// ===========================================================================
//  [2] CALIBRACIÓN EN ARRANQUE
// ===========================================================================
//
//  Durante los primeros CALIBRATION_MS tras encender, los dos LED de equipo
//  parpadean juntos: es la señal para pasar el robot a mano por ENCIMA de la
//  cinta negra y del piso gris varias veces, cubriendo ambos sensores. El
//  sketch se queda con el mínimo y el máximo que vio cada sensor y arma el
//  umbral como el punto medio.
//
//  Guardarraíl: si el rango visto (max-min) es demasiado chico —el robot se
//  quedó quieto y nunca vio contraste real— se descarta la calibración y se
//  usa el umbral de reserva documentado arriba, avisando por consola.

constexpr uint32_t CALIBRATION_MS = 3000;
constexpr uint16_t MIN_USABLE_SPAN = 300;     // por debajo de esto, no hubo contraste real
constexpr uint16_t FALLBACK_THRESHOLD = 3700; // ver análisis del log en el encabezado

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
//  [3] MOTORES — igual que firmware-esp32/src/main.cpp, sin el resto
// ===========================================================================

namespace Pwm {
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;   // el L298N calienta y pierde par a 20 kHz
    constexpr uint8_t  MOTOR_RESOLUTION = 8;      // duty 0..255
}

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

// La API de LEDC cambió entre el core 2.x y el 3.x de Arduino-ESP32.
void PwmAttach(uint8_t pin, uint8_t channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;
    ledcAttach(pin, Pwm::MOTOR_FREQ_HZ, Pwm::MOTOR_RESOLUTION);
#else
    ledcSetup(channel, Pwm::MOTOR_FREQ_HZ, Pwm::MOTOR_RESOLUTION);
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
    PwmAttach(m.en, m.ledc_channel);
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
//  [4] MÁQUINA DE ESTADOS — avanzar y parar en seco
// ===========================================================================
//
//  FORWARD: avanza recto (mismo sentido que 03-motores-adelante, nunca
//           retrocede, nunca gira), mirando el sensor DERECHO.
//  STOPPED: se quedó sin cinta debajo, motores a 0 y ahí se queda. No hay
//           forma de volver a FORWARD sin resetear el ESP32 — a propósito,
//           para poder mirar exactamente dónde paró antes de que el robot
//           haga cualquier otra cosa.
//
//  SOLO decide el sensor DERECHO (QTR_RIGHT_OUT). El IZQUIERDO se sigue
//  leyendo, calibrando e imprimiendo por consola —sirve como referencia y
//  por si hace falta reactivarlo más adelante— pero no participa en la
//  decisión de parar: es el que confirmamos que mide bien.

enum class State : uint8_t { FORWARD, STOPPED };

constexpr int kForwardSpeed = 55;

// Confirmación: una sola lectura por encima del umbral no basta para parar
// (el ADC tiene ruido puntual, sobre todo cerca del umbral). Se exigen
// kConfirmacionesNecesarias lecturas SEGUIDAS por encima del umbral —
// cualquier lectura de gris en medio reinicia el conteo a 0 — antes de
// confiar en que de verdad es la cinta negra. A ~5 ms por vuelta de loop(),
// 4 lecturas son ~20 ms de negro sostenido: suficiente para filtrar un pico
// de ruido, poco como para que el robot avance mucho de más antes de parar.
constexpr uint8_t kConfirmacionesNecesarias = 4;

uint8_t g_confirmRight = 0;

// Devuelve true solo cuando la lectura cruda lleva kConfirmacionesNecesarias
// vueltas seguidas por encima del umbral. "contador" se pasa por referencia
// para que quien llame lleve su propio historial.
bool Confirmar(bool crudo, uint8_t &contador) {
    if (crudo) {
        if (contador < kConfirmacionesNecesarias) contador++;
    } else {
        contador = 0;  // una sola lectura de gris descarta todo lo acumulado
    }
    return contador >= kConfirmacionesNecesarias;
}

State g_state = State::FORWARD;

void RunStateMachine(bool onLine) {
    switch (g_state) {
        case State::FORWARD: {
            if (onLine) {
                MotorsStop();
                g_state = State::STOPPED;
                DEBUG_LINK.println("[Borde] linea negra detectada (sensor derecho) -> STOP");
                digitalWrite(Pins::LED_TEAM_BLUE, HIGH);
            } else {
                DriveSides(kForwardSpeed, kForwardSpeed);
            }
            break;
        }

        case State::STOPPED: {
            MotorsStop();  // redundante a propósito: si algo lo perturbara, se vuelve a frenar
            break;
        }
    }
}

// ===========================================================================
//  [5] setup() / loop()
// ===========================================================================

void setup() {
    DEBUG_LINK.begin(115200);
    delay(200);
    DEBUG_LINK.println("\nPrueba 01 - Mantente en el cuadro");

    pinMode(Pins::LED_TEAM_RED, OUTPUT);
    pinMode(Pins::LED_TEAM_BLUE, OUTPUT);

    // Reflectancia: igual que firmware-esp32 (ADC de 12 bits, atenuación
    // 11 dB para cubrir la excursión completa del QTR a 3.3 V).
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

    RunCalibration();

    DEBUG_LINK.println("[Setup] listo, arrancando en FORWARD.");
}

void loop() {
    const uint16_t left  = (uint16_t)analogRead(Pins::QTR_LEFT_OUT);
    const uint16_t right = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);

    const bool leftRaw  = left  > g_leftThreshold;   // referencia, no decide nada
    const bool rightRaw = right > g_rightThreshold;

    // No se actúa sobre la lectura cruda directamente: hace falta que se
    // sostenga varias vueltas seguidas (ver Confirmar()) antes de tratarla
    // como negro de verdad. Solo se confirma/decide con el sensor derecho.
    const bool rightOnLine = Confirmar(rightRaw, g_confirmRight);

    RunStateMachine(rightOnLine);

    // Telemetría de banco, para ajustar velocidades y umbrales con el
    // monitor serial abierto. Se puede comentar una vez calibrado a gusto.
    // "izq" es solo referencia (no decide); "der" es el que manda.
    static uint32_t lastPrint = 0;
    if ((uint32_t)(millis() - lastPrint) > 200) {
        lastPrint = millis();
        DEBUG_LINK.printf("izq=%u(%s, ref) der=%u(%s,%u/%u) estado=%s\n",
            left, leftRaw ? "NEGRO" : "gris",
            right, rightRaw ? "NEGRO" : "gris", g_confirmRight, kConfirmacionesNecesarias,
            g_state == State::FORWARD ? "FORWARD" : "STOPPED");
    }

    delay(5);
}
