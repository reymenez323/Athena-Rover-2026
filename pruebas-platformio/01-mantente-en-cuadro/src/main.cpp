// ===========================================================================
//  Prueba 01 — Mantente en el cuadro
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  OBJETIVO: el robot arranca yendo hacia adelante dentro del cuadrado
//  delimitado por la cinta negra, y en cuanto el sensor de reflectancia
//  DERECHO detecta el borde (fondo gris -> cinta negra), retrocede un
//  poco, gira 180° sobre su propio eje y retoma la marcha hacia adelante
//  — así se queda rebotando dentro del cuadro en vez de atascarse contra
//  el borde.
//
//  Usa SOLO el sensor derecho (QTR_RIGHT_OUT) — es el que confirmamos que
//  mide bien (ver calibracion-ir/). El sensor izquierdo NO se lee en
//  ningún punto de este archivo: no aportaba nada a la decisión, así que
//  se le quitó también la calibración y la telemetría (ver [1]).
//
//  Es un sketch de banco, deliberadamente simple (setup/loop, sin FreeRTOS
//  ni colas): sirve para validar el comportamiento de borde ANTES de
//  integrarlo a las 7 tareas de firmware-esp32/src/main.cpp. Usa los MISMOS
//  pines que el firmware principal (ver hardware/conexiones-esp32-s3.md) así
//  que corre tal cual sobre el chasis ya cableado.
//
//  SOBRE EL UMBRAL DE "esto es cinta negra":
//  Como SOLO decide el sensor derecho (= "sensor B" en calibracion-ir/, ver
//  [4]), el umbral de reserva se saca del análisis de ESE sensor, no del
//  izquierdo. Los logs más recientes con la atenuación del ADC ya fijada
//  (calibracion-ir/data_logs/IR_NEGRO_2026-08-28_15-36-20.csv e
//  IR_GRIS_2026-08-28_15-33-07.csv — ver el análisis completo en
//  calibracion-ir/detector-color/src/main.cpp) dan, para el sensor B:
//  NEGRO 2781–2938, GRIS 2747–2935 — un solape amplio, porque esas
//  muestras se tomaron a mano, sin altura fija. Con el sensor sostenido a
//  mano a distintas alturas, la reflectancia que ve el QTR depende
//  muchísimo de la distancia a la superficie, no solo del color. Sobre el
//  chasis, a altura fija, el contraste real debería ser mucho más limpio
//  — pero para no apostarlo todo a eso, este sketch CALIBRA EN CADA
//  ARRANQUE (ver [2]) en vez de confiar en una constante copiada del log.
//
//  Umbral de reserva (si el arranque no calibra, ver [2]): 2910 — el mejor
//  punto de corte encontrado sobre esos mismos logs del sensor B (52/740
//  errores; ver el detalle en calibracion-ir/detector-color/src/main.cpp).
//  OJO: antes de este ajuste, esta constante valía 3700 — un número sacado
//  del sensor IZQUIERDO (A), que dejó de decidir nada desde que el commit
//  c4f9b47 pasó la decisión al sensor derecho (B) y nunca se actualizó
//  este umbral. Con 3700, si la calibración en vivo del sensor derecho
//  llegaba a fallar (span < MIN_USABLE_SPAN), el fallback NUNCA se
//  activaba —el sensor B no pasa de ~2940— y el robot habría seguido de
//  largo sobre la cinta negra sin detectarla nunca.
//
// ===========================================================================

#include <Arduino.h>

// ===========================================================================
//  [1] PINES — idénticos a hardware/conexiones-esp32-s3.md y a firmware-esp32
// ===========================================================================

namespace Pins {
    // Reflectancia QTRX-HD-01A DERECHO — el único que se lee en este
    // archivo. ¡A 3.3 V, nunca a 5 V! (ver la tabla de riesgos en
    // hardware/conexiones-esp32-s3.md). El sensor IZQUIERDO (QTR_LEFT_OUT
    // = GPIO1) existe físicamente en el chasis pero no se declara aquí a
    // propósito: nunca decidió nada, así que no vale la pena seguir
    // calibrándolo ni imprimiéndolo.
    constexpr uint8_t QTR_RIGHT_OUT     = 2;   // ADC1_CH1
    constexpr uint8_t QTR_EMITTER_CTRL  = 42;  // enciende los LED IR de ambos sensores físicos (A y B comparten control)

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
    // maniobra de borde en curso (no hace falta laptop conectada para ver
    // si el sketch está funcionando).
    constexpr uint8_t LED_TEAM_RED  = 40;   // solo parpadea durante la calibración de arranque
    constexpr uint8_t LED_TEAM_BLUE = 41;   // encendido mientras dura la reversa+giro (sensor DERECHO)
}

#define DEBUG_LINK Serial0   // consola por el puerto UART del DevKit

// ===========================================================================
//  [2] CALIBRACIÓN EN ARRANQUE
// ===========================================================================
//
//  Durante los primeros CALIBRATION_MS tras encender, los dos LED de equipo
//  parpadean juntos: es la señal para pasar el robot a mano por ENCIMA de la
//  cinta negra y del piso gris varias veces, cubriendo el sensor derecho. El
//  sketch se queda con el mínimo y el máximo que vio y arma el umbral como
//  el punto medio.
//
//  Guardarraíl: si el rango visto (max-min) es demasiado chico —el robot se
//  quedó quieto y nunca vio contraste real— se descarta la calibración y se
//  usa el umbral de reserva documentado arriba, avisando por consola.

constexpr uint32_t CALIBRATION_MS = 3000;
constexpr uint16_t MIN_USABLE_SPAN = 300;     // por debajo de esto, no hubo contraste real
constexpr uint16_t FALLBACK_THRESHOLD = 2910; // sensor derecho (B); ver análisis del log en el encabezado

uint16_t g_rightThreshold = FALLBACK_THRESHOLD;

void RunCalibration() {
    uint16_t rightMin = 4095, rightMax = 0;

    const uint32_t start = millis();
    uint32_t lastBlink = start;
    bool blinkState = false;

    DEBUG_LINK.println("[Calib] moviendo el robot sobre negro y gris ahora (3 s)...");

    while ((uint32_t)(millis() - start) < CALIBRATION_MS) {
        const uint16_t r = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);

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

    const uint16_t rightSpan = rightMax - rightMin;

    if (rightSpan >= MIN_USABLE_SPAN) {
        g_rightThreshold = (uint16_t)((rightMin + rightMax) / 2);
    } else {
        DEBUG_LINK.println("[Calib] sensor DERECHO no vio suficiente contraste, uso el umbral de reserva.");
    }

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
//  [4] MÁQUINA DE ESTADOS — reversa + giro de 180° al toparse con el borde
// ===========================================================================
//
//  FORWARD:   avanza recto, mirando el sensor DERECHO. En cuanto confirma
//             el borde (cinta negra sostenida), pasa a REVERSING.
//  REVERSING: retrocede en línea recta durante kReverseMs, a ciegas (no
//             mira los sensores) — es una maniobra cronometrada para
//             alejarse de la cinta antes de girar; girar con una rueda
//             todavía encima o pegada al borde es más propenso a trabarse
//             o a salirse del cuadro. Al cumplirse el tiempo, pasa a
//             TURNING.
//  TURNING:   gira sobre su propio eje (una rueda adelante, la otra atrás)
//             durante kTurn180Ms, calibrado para completar ~180°. Al
//             cumplirse el tiempo, vuelve a FORWARD.
//
//  REVERSING y TURNING son de LAZO ABIERTO (sin encoders ni giroscopio): la
//  única señal que decide cuándo terminan es el reloj, así que kReverseMs y
//  kTurn180Ms son estimaciones de partida que HAY que ajustar sobre el
//  chasis real — ver la nota junto a cada constante.
//
//  Usa SOLO el sensor DERECHO (QTR_RIGHT_OUT) para todo: calibrar y
//  arrancar la maniobra. El izquierdo no se lee en ningún punto de este
//  archivo (ver [1]).

enum class State : uint8_t { FORWARD, REVERSING, TURNING };

constexpr int kForwardSpeed = 55;
constexpr int kReverseSpeed = -55;   // mismo duty que adelante, sentido invertido

// El giro en el sitio necesita MUCHO más torque que avanzar derecho: con 4
// ruedas motrices, pivotar significa que las 4 raspan contra el piso en vez
// de rodar limpio (fricción de deslizamiento en las 4, no solo resistencia
// a rodar) — a 55 casi no giraba. Va al máximo duty (100) a propósito.
constexpr int kTurnSpeed = 100;

// Duraciones de las maniobras a ciegas — AJUSTAR sobre el robot real: se
// cronometra (a ojo, o mirando los timestamps de DEBUG_LINK) cuánto tarda
// el robot en alejarse una distancia razonable en reversa a kReverseSpeed,
// y cuánto tarda en dar 180° exactos girando en el sitio a kTurnSpeed, y se
// ajustan estos dos números. Los de acá NO son una calibración, son un
// punto de partida.
//
// HISTORIAL: 650 ms de giro a duty 55 resultó muy corto (giró casi nada).
// Al subir a 2000 ms el reporte fue "retrocede de forma entrecortada y se
// traba" — consistente con un giro que TAMPOCO estaba completando los
// 180°: si el robot vuelve a FORWARD todavía encima o cerca de la cinta,
// el debounce lo detecta en ~20 ms (ver kConfirmacionesNecesarias) y
// dispara OTRA reversa casi de inmediato — eso se ve como "retrocede en
// ráfagas" con giros que parecen trabarse en el medio, aunque cada
// maniobra individualmente esté corriendo bien. Por eso el fix fue subir
// kTurnSpeed a 100 (arriba), no alargar más este tiempo: si el pivote
// estaba estancado por falta de torque, más milisegundos a la misma
// velocidad no giran más grados, solo queman motor en el sitio.
//
// Si TODAVÍA se traba en TURNING con kTurnSpeed=100: ya no es un ajuste de
// software — revisar tracción de las ruedas, que la batería de motores no
// esté cayendo de voltaje bajo carga, y que nada roce mecánicamente.
constexpr uint32_t kReverseMs = 1500;
constexpr uint32_t kTurn180Ms = 2000;

// Confirmación: una sola lectura por encima del umbral no basta para
// arrancar la maniobra (el ADC tiene ruido puntual, sobre todo cerca del
// umbral). Se exigen kConfirmacionesNecesarias lecturas SEGUIDAS por encima
// del umbral —cualquier lectura de gris en medio reinicia el conteo a 0—
// antes de confiar en que de verdad es la cinta negra. A ~5 ms por vuelta
// de loop(), 4 lecturas son ~20 ms de negro sostenido: suficiente para
// filtrar un pico de ruido, poco como para que el robot avance mucho de más
// antes de reaccionar.
//
// El conteo SOLO se acumula en FORWARD (ver RunStateMachine): durante
// REVERSING y TURNING no se toca, y se reinicia a 0 justo al volver a
// FORWARD para exigir una detección fresca — si no, una rueda que sigue
// sobre la cinta apenas termina el giro dispararía otra maniobra sin que
// el robot haya avanzado nada.
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
uint32_t g_stateEnteredAtMs = 0;

void EnterState(State s) {
    g_state = s;
    g_stateEnteredAtMs = millis();
}

const char *StateName(State s) {
    switch (s) {
        case State::FORWARD:   return "FORWARD";
        case State::REVERSING: return "REVERSING";
        case State::TURNING:   return "TURNING";
    }
    return "?";
}

void RunStateMachine(bool rightRaw) {
    switch (g_state) {
        case State::FORWARD: {
            if (Confirmar(rightRaw, g_confirmRight)) {
                DEBUG_LINK.println("[Borde] linea negra detectada (sensor derecho) -> reversa");
                digitalWrite(Pins::LED_TEAM_BLUE, HIGH);
                EnterState(State::REVERSING);
            } else {
                DriveSides(kForwardSpeed, kForwardSpeed);
            }
            break;
        }

        case State::REVERSING: {
            DriveSides(kReverseSpeed, kReverseSpeed);
            if ((uint32_t)(millis() - g_stateEnteredAtMs) >= kReverseMs) {
                DEBUG_LINK.println("[Borde] reversa completa -> giro de 180");
                EnterState(State::TURNING);
            }
            break;
        }

        case State::TURNING: {
            DriveSides(kTurnSpeed, -kTurnSpeed);   // pivote: izq adelante, der atrás
            if ((uint32_t)(millis() - g_stateEnteredAtMs) >= kTurn180Ms) {
                DEBUG_LINK.println("[Borde] giro completo -> FORWARD");
                digitalWrite(Pins::LED_TEAM_BLUE, LOW);
                g_confirmRight = 0;   // exige una deteccion fresca, ver nota arriba
                EnterState(State::FORWARD);
            }
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
    const uint16_t right = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);
    const bool rightRaw = right > g_rightThreshold;

    // El debounce (Confirmar) vive dentro de RunStateMachine y solo se
    // acumula en FORWARD — ver la nota junto a kConfirmacionesNecesarias.
    RunStateMachine(rightRaw);

    // Telemetría de banco, para ajustar velocidades y tiempos con el
    // monitor serial abierto. Se puede comentar una vez calibrado a gusto.
    static uint32_t lastPrint = 0;
    if ((uint32_t)(millis() - lastPrint) > 200) {
        lastPrint = millis();
        DEBUG_LINK.printf("der=%u(%s,%u/%u) estado=%s\n",
            right, rightRaw ? "NEGRO" : "gris", g_confirmRight, kConfirmacionesNecesarias,
            StateName(g_state));
    }

    delay(5);
}
