// ===========================================================================
//  Athena Rover 2026 — Firmware ESP32-S3 AUTÓNOMO (sin Raspberry Pi)
//  Retos del Rover H07 · INTEC · Reymildo & Montse
//
//  VARIANTE v4-merodeo-color: prueba de merodeo puro. Avanza en línea recta;
//  al detectar el borde negro con el QTR (solo el DERECHO -- el izquierdo
//  sigue roto, ver más abajo) se detiene DE INMEDIATO y gira un ángulo
//  aleatorio entre 60° y 180°, en dirección aleatoria, para seguir dentro
//  de la pista, y retoma la marcha. Sin fin: merodea indefinidamente. NO
//  agarra nada, NO se detiene en ningún color -- a diferencia de
//  v3-color-delantero, aquí la llave/el gripper no existen para nada.
//
//  Durante TODO el recorrido, el LED RGB muestra en vivo qué color de zona
//  está viendo el sensor de color activo (mismos parámetros recalibrados
//  que v3-color-delantero, sin tocar): blanco=GRIS, amarillo=AMARILLO,
//  rojo=ROJO, azul=AZUL, apagado=NEGRO (o sin lectura). Esto es 100%
//  independiente de la lógica de merodeo -- el QTR decide cuándo girar, el
//  color solo se MUESTRA, nunca frena ni redirige al robot.
//  CONSERVAR TAL CUAL -- no modificar este archivo salvo pedido explícito.
// ===========================================================================
//
//  QTR IZQUIERDO: SIGUE ROTO
//  ----------------------------------------------------------------------
//  Confirmado en banco (ver firmware-esp32/src/main.cpp y
//  standalones/v2-merodeo/): el sensor izquierdo se queda pegado en 4095
//  sin importar superficie ni estado del emisor. Se fuerza left_on_line en
//  false (nunca dispara el giro) y se sigue reportando left_raw/left_restado
//  solo para diagnóstico. Toda la detección de borde depende del DERECHO.
//
//  POR QUÉ SIN MANIOBRA DE RETROCESO (a diferencia de v2-merodeo)
//  ----------------------------------------------------------------------
//  v2-merodeo hace un baile de 4 fases (parar, retroceder, parar, girar
//  180° siempre a la derecha) antes de retomar. Pedido explícito para esta
//  prueba: más simple -- parar y girar nada más, ángulo Y dirección
//  ALEATORIOS (60°-180°) en vez de siempre 180° a la derecha. La duración
//  del giro se calcula con la misma tasa ya calibrada en banco anoche
//  (kEvasionGiroMs=2000 ms para 180° en v2-merodeo, ver kMsPorGrado).
//
//  SEGURIDADES QUE SÍ SE CONSERVAN DE v2-merodeo (no pedidas explícitamente,
//  pero ya resolvieron un problema real anoche, no cuestan nada agregar):
//    · Debounce de 50 ms sobre el borde: un solo instante suelto no
//      dispara el giro (ruido de pista sucia).
//    · Gracia de 1 s al entrar a MERODEAR (al arrancar o tras un giro):
//      ignora el borde por completo, para no volver a dispararse contra el
//      mismo borde del que recién se alejó.
//
//  HARDWARE:
//    · 2x L298N            -> 4 motores
//    · 2x TCS34725         -> delantero (I2C0) + trasero (I2C1), SOLO para
//                              mostrar color en el LED -- nada de gripper,
//                              nada de PCA9685 en esta variante.
//    · 2x QTRX-HD-01A      -> reflectancia (solo el derecho funciona)
//    · LED RGB (1x)        -> muestra el color detectado, en vivo
//    · Switch 3 posiciones -> traba de seguridad
//
//  ÍNDICE
//    [1] Configuración: pines, prioridades, stacks, periodos
//    [2] Tipos compartidos entre tareas
//    [3] Colas
//    [4] Watchdog cooperativo (heartbeats)
//    [5] Driver TCS34725 (los dos sensores, por I2C)
//    [6] Clasificación de color -> etiqueta
//    [7] Tareas de hardware (motores, color, reflectancia, LED)
//    [8] MissionTask — el cerebro: merodear + girar al ver el borde
//    [9] SupervisorTask, setup() / loop()
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>   // esp_reset_reason(), esp_random()

// ===========================================================================
//  [1] CONFIGURACIÓN
// ===========================================================================

namespace Pins {
    // -------- Motores: 2x L298N, cada uno mueve 2 motores ------------------
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

    // -------- Bus I2C nº0: TCS34725 DELANTERO -------------------------------
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;
    constexpr uint8_t TOF_XSHUT = 3;        // en reset (LOW) todo el tiempo
    constexpr uint8_t TCS_LED_FRONT = 18;   // LED de iluminación, activo en alto

    // -------- Bus I2C nº1: TCS34725 TRASERO ---------------------------------
    // Sin PCA9685 en esta variante (no hay gripper) -- el trasero es el
    // ÚNICO dispositivo en este bus, así que no hace falta mutex: ninguna
    // otra tarea lo toca.
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    // -------- Reflectancia QTRX-HD-01A --------------------------------------
    // ¡ALIMENTARLOS A 3.3 V!
    constexpr uint8_t QTR_LEFT_OUT  = 1;
    constexpr uint8_t QTR_RIGHT_OUT = 2;
    constexpr uint8_t QTR_EMITTER_CTRL = 42;

    // -------- LED RGB: muestra el color detectado, en vivo ------------------
    constexpr uint8_t LED_RGB_R = 39;
    constexpr uint8_t LED_RGB_G = 38;
    constexpr uint8_t LED_RGB_B = 41;

    // -------- Switch de 3 posiciones: SOLO traba de seguridad --------------
    constexpr uint8_t TEAM_SWITCH_BLUE = 40;
    constexpr uint8_t TEAM_SWITCH_RED  = 21;
}

namespace I2CAddr {
    constexpr uint8_t TCS34725 = 0x29;   // fija -- por eso los dos sensores van en buses separados
}

namespace Pwm {
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;
    constexpr uint8_t  MOTOR_RESOLUTION = 8;

    constexpr uint32_t RGB_FREQ_HZ    = 5000;
    constexpr uint8_t  RGB_RESOLUTION = 8;
}

namespace TaskPriority {
    constexpr UBaseType_t SUPERVISOR      = 6;
    constexpr UBaseType_t MOTOR_CONTROL   = 5;
    constexpr UBaseType_t MISSION         = 4;
    constexpr UBaseType_t REFLECTANCE     = 3;
    constexpr UBaseType_t COLOR_SENSOR    = 2;
    constexpr UBaseType_t LED_STATUS      = 1;
}

namespace TaskStack {
    constexpr uint32_t SUPERVISOR      = 3072;
    constexpr uint32_t MISSION         = 4096;
    constexpr uint32_t MOTOR_CONTROL   = 3072;
    constexpr uint32_t REFLECTANCE     = 2560;
    constexpr uint32_t COLOR_SENSOR    = 3584;
    constexpr uint32_t LED_STATUS      = 2048;
}

namespace TaskPeriodMs {
    constexpr uint32_t SUPERVISOR    = 200;
    constexpr uint32_t MISSION       = 50;
    constexpr uint32_t MOTOR_CONTROL = 20;
    constexpr uint32_t REFLECTANCE   = 20;   // 50 Hz, seguimiento de línea
    constexpr uint32_t COLOR_SENSOR  = 100;
    constexpr uint32_t LED_STATUS    = 100;
}

constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;
constexpr uint32_t MISSION_FAILSAFE_TIMEOUT_MS = 500;

enum class TaskId : uint8_t {
    MOTOR_CONTROL = 0,
    REFLECTANCE,
    COLOR_SENSOR,
    LED_STATUS,
    MISSION,
    COUNT
};

#define DEBUG_LINK Serial

// ===========================================================================
//  [2] TIPOS COMPARTIDOS ENTRE TAREAS
// ===========================================================================

enum class MotorMode  : uint8_t { STOP = 0, DRIVE = 1 };
enum class ColorLabel : uint8_t { UNKNOWN = 0, BLACK, YELLOW, RED, BLUE, FLOOR };

struct MotorCommand {
    MotorMode mode = MotorMode::STOP;
    int8_t    left  = 0;
    int8_t    right = 0;
};

struct LedCommand {
    ColorLabel color = ColorLabel::UNKNOWN;
};

struct ColorReading {
    uint32_t   timestamp_ms = 0;
    ColorLabel front        = ColorLabel::UNKNOWN;
    bool       front_valid  = false;
    ColorLabel back         = ColorLabel::UNKNOWN;
    bool       back_valid   = false;
};

// Mismos campos que standalones/v2-merodeo/ (left_restado/right_restado se
// reportan solo para diagnóstico, no se usan para decidir on_line).
struct ReflectanceReading {
    uint32_t timestamp_ms  = 0;
    uint16_t left_raw      = 0;
    uint16_t right_raw     = 0;
    int16_t  left_restado  = 0;
    int16_t  right_restado = 0;
    bool     left_on_line  = false;
    bool     right_on_line = false;
};

struct HealthReport {
    uint32_t timestamp_ms          = 0;
    uint8_t  faulted_tasks_bitmask = 0;
};

// El delantero manda si dio lectura válida este ciclo; si no, se cae al
// trasero. UNKNOWN si ninguno de los dos respondió. Mismo criterio que
// v3-color-delantero.
inline ColorLabel PriorityColor(const ColorReading &c) {
    if (c.front_valid) return c.front;
    if (c.back_valid)  return c.back;
    return ColorLabel::UNKNOWN;
}

// ===========================================================================
//  [3] COLAS
// ===========================================================================
//
//   MissionTask     --> motorCmdQueue (1, overwrite) --> MotorTask
//   ColorSensorTask --> colorQueue    (4, FIFO)       --> MissionTask
//   ReflectanceTask --> reflectQueue  (4, FIFO)       --> MissionTask
//   SupervisorTask  --> healthQueue   (1, overwrite)  --> MissionTask (solo log)
//
//  Sin mutex de bus I2C en esta variante: cada bus (0 y 1) lo toca UNA sola
//  tarea (ColorSensorTask, las dos), no hay concurrencia que proteger.

static QueueHandle_t g_motorCmdQueue = nullptr;
static QueueHandle_t g_ledCmdQueue   = nullptr;
static QueueHandle_t g_colorQueue    = nullptr;
static QueueHandle_t g_reflectQueue  = nullptr;
static QueueHandle_t g_healthQueue   = nullptr;

// ===========================================================================
//  [4] WATCHDOG COOPERATIVO
// ===========================================================================

static volatile uint32_t g_lastHeartbeatMs[(size_t)TaskId::COUNT];

static void WatchdogInit() {
    const uint32_t now = millis();
    for (size_t i = 0; i < (size_t)TaskId::COUNT; ++i) g_lastHeartbeatMs[i] = now;
}

static inline void Heartbeat(TaskId id) {
    g_lastHeartbeatMs[(size_t)id] = millis();
}

static uint8_t WatchdogCheck() {
    const uint32_t now = millis();
    uint8_t faulted = 0;
    for (size_t i = 0; i < (size_t)TaskId::COUNT; ++i) {
        if ((uint32_t)(now - g_lastHeartbeatMs[i]) > WATCHDOG_TIMEOUT_MS) {
            faulted |= (uint8_t)(1u << i);
        }
    }
    return faulted;
}

// ===========================================================================
//  [5] DRIVER TCS34725 — los DOS sensores, cada uno en su bus
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
//  [6] CLASIFICACIÓN DE COLOR -> ETIQUETA
// ===========================================================================
//
//  Umbrales del DELANTERO, reajustados 2026-09-07 (segunda vuelta) --
//  IDÉNTICOS a v3-color-delantero/src/main.cpp y
//  calibracion/color/detector-tcs/src/main.cpp. Si se recalibra de nuevo,
//  actualizar los tres archivos.
//
//  ⚠️ Se usan también para el TRASERO a falta de una calibración propia.

static ColorLabel ClassifyColor(const Tcs34725::Rgbc &s) {
    if (s.c < 314) return ColorLabel::BLACK;

    const float total = (float)s.c;
    const float r = (float)s.r / total;
    const float g = (float)s.g / total;
    const float b = (float)s.b / total;

    if (r > 0.450f && g < 0.312f && b < 0.300f) return ColorLabel::RED;
    if (b > 0.206f && r < 0.390f)               return ColorLabel::BLUE;
    if (r > 0.420f && g > 0.200f && b < 0.140f) return ColorLabel::YELLOW;

    return ColorLabel::FLOOR;
}

// ===========================================================================
//  [7] TAREAS DE HARDWARE
// ===========================================================================

namespace {

// ---------------------------------------------------------------------------
//  7.1  MotorTask — 4 motores a través de 2 drivers L298N
// ---------------------------------------------------------------------------

struct Motor {
    uint8_t in1, in2, en, ledc_channel;
};

constexpr Motor kMotorFL = {Pins::L298N_L_IN2, Pins::L298N_L_IN1, Pins::L298N_L_ENA, 0};
constexpr Motor kMotorRL = {Pins::L298N_L_IN3, Pins::L298N_L_IN4, Pins::L298N_L_ENB, 1};
constexpr Motor kMotorFR = {Pins::L298N_R_IN1, Pins::L298N_R_IN2, Pins::L298N_R_ENA, 2};
constexpr Motor kMotorRR = {Pins::L298N_R_IN3, Pins::L298N_R_IN4, Pins::L298N_R_ENB, 3};

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

// speed > 0 -> "avanza" -- CONFIRMADO EN BANCO 2026-09-07 sobre el robot
// físico tal como está cableado HOY (ver el mismo hallazgo documentado en
// v3-color-delantero/src/main.cpp: la convención vieja de v1-confirmado/
// v2-merodeo, "speed < 0 = avanza", ya no coincide con este robot).
void MotorApply(const Motor &m, int speed) {
    speed = constrain(speed, -100, 100);
    const bool forward = (speed > 0);
    digitalWrite(m.in1, forward ? HIGH : LOW);
    digitalWrite(m.in2, forward ? LOW  : HIGH);
    PwmWrite(m.en, m.ledc_channel, (uint32_t)abs(speed) * 255u / 100u);
}

void MotorsStop() {
    MotorApply(kMotorFL, 0);
    MotorApply(kMotorRL, 0);
    MotorApply(kMotorFR, 0);
    MotorApply(kMotorRR, 0);
}

void MotorTask(void *) {
    MotorSetup(kMotorFL);
    MotorSetup(kMotorRL);
    MotorSetup(kMotorFR);
    MotorSetup(kMotorRR);
    MotorsStop();

    MotorCommand current{};
    uint32_t last_cmd_ms = millis();

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::MOTOR_CONTROL);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        MotorCommand incoming;
        if (xQueueReceive(g_motorCmdQueue, &incoming, 0) == pdTRUE) {
            current = incoming;
            last_cmd_ms = millis();
        }

        const bool mission_stale =
            (uint32_t)(millis() - last_cmd_ms) > MISSION_FAILSAFE_TIMEOUT_MS;

        if (mission_stale || current.mode == MotorMode::STOP) {
            MotorsStop();
        } else {
            MotorApply(kMotorFL, current.left);
            MotorApply(kMotorRL, current.left);
            MotorApply(kMotorFR, current.right);
            MotorApply(kMotorRR, current.right);
        }

        Heartbeat(TaskId::MOTOR_CONTROL);
        vTaskDelayUntil(&last_wake, period);
    }
}

template <typename T>
void PushDropOldest(QueueHandle_t queue, const T &item) {
    if (xQueueSend(queue, &item, 0) != pdTRUE) {
        T discard;
        xQueueReceive(queue, &discard, 0);
        xQueueSend(queue, &item, 0);
    }
}

// ---------------------------------------------------------------------------
//  7.2  ColorSensorTask — LOS DOS TCS34725, delantero (Wire) y trasero (Wire1)
// ---------------------------------------------------------------------------
//  Solo para alimentar el LED -- ver el aviso al principio del archivo.

void ColorSensorTask(void *) {
    pinMode(Pins::TOF_XSHUT, OUTPUT);
    digitalWrite(Pins::TOF_XSHUT, LOW);

    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);
    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);

    bool front_ok = Tcs34725::Init(Wire);
    if (!front_ok) DEBUG_LINK.println("[Color] sensor DELANTERO no responde (bus I2C 0).");

    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);
    bool back_ok = Tcs34725::Init(Wire1);
    if (!back_ok) DEBUG_LINK.println("[Color] sensor TRASERO no responde (bus I2C 1).");

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::COLOR_SENSOR);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        if ((!front_ok || !back_ok) && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            if (!front_ok) front_ok = Tcs34725::Init(Wire);
            if (!back_ok)  back_ok  = Tcs34725::Init(Wire1);
        }

        ColorReading reading;
        reading.timestamp_ms = millis();

        if (front_ok) {
            Tcs34725::Rgbc sample;
            if (Tcs34725::Read(Wire, sample)) {
                reading.front = ClassifyColor(sample);
                reading.front_valid = true;
            } else {
                front_ok = false;
            }
        }

        if (back_ok) {
            Tcs34725::Rgbc sample;
            if (Tcs34725::Read(Wire1, sample)) {
                reading.back = ClassifyColor(sample);
                reading.back_valid = true;
            } else {
                back_ok = false;
            }
        }

        PushDropOldest(g_colorQueue, reading);

        Heartbeat(TaskId::COLOR_SENSOR);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  7.3  ReflectanceTask — 1x QTRX-HD-01A (derecho; izquierdo roto), con
//       rechazo de luz ambiente -- CALIBRADO EN BANCO, idéntico a
//       standalones/v2-merodeo/ y firmware-esp32/. Ver la nota grande al
//       principio del archivo.
// ---------------------------------------------------------------------------

constexpr int16_t kBordeRestadoUmbral = 40;

void ReflectanceTask(void *) {
    analogReadResolution(12);
    analogSetPinAttenuation(Pins::QTR_LEFT_OUT, ADC_11db);
    analogSetPinAttenuation(Pins::QTR_RIGHT_OUT, ADC_11db);

    pinMode(Pins::QTR_EMITTER_CTRL, OUTPUT);
    digitalWrite(Pins::QTR_EMITTER_CTRL, HIGH);

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::REFLECTANCE);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        digitalWrite(Pins::QTR_EMITTER_CTRL, LOW);
        delay(2);   // >= 1 ms: apagado real, no un pulso de dimming
        const uint16_t left_ambiente  = (uint16_t)analogRead(Pins::QTR_LEFT_OUT);
        const uint16_t right_ambiente = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);

        digitalWrite(Pins::QTR_EMITTER_CTRL, HIGH);
        delayMicroseconds(200);
        const uint16_t left_raw  = (uint16_t)analogRead(Pins::QTR_LEFT_OUT);
        const uint16_t right_raw = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);

        ReflectanceReading reading;
        reading.timestamp_ms  = millis();
        reading.left_raw      = left_raw;
        reading.right_raw     = right_raw;
        reading.left_restado  = (int16_t)((int32_t)left_raw  - (int32_t)left_ambiente);
        reading.right_restado = (int16_t)((int32_t)right_raw - (int32_t)right_ambiente);

        reading.right_on_line = abs(reading.right_restado) < kBordeRestadoUmbral;

        // IZQUIERDO: sensor confirmado roto -- forzado en false, ver el
        // aviso al principio del archivo. left_raw/left_restado se siguen
        // reportando para diagnóstico.
        reading.left_on_line = false;

        PushDropOldest(g_reflectQueue, reading);

        Heartbeat(TaskId::REFLECTANCE);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  7.4  LedTask — muestra en vivo el color detectado (prioridad delantero)
// ---------------------------------------------------------------------------

namespace RgbLed {
    constexpr uint8_t CH_R = 4;
    constexpr uint8_t CH_G = 5;
    constexpr uint8_t CH_B = 6;

    constexpr bool kCommonAnode = false;   // cátodo común, duty alto = más brillante

    void Setup() {
        PwmAttach(Pins::LED_RGB_R, CH_R, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(Pins::LED_RGB_G, CH_G, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(Pins::LED_RGB_B, CH_B, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
    }

    void SetRaw(uint8_t r, uint8_t g, uint8_t b) {
        if (kCommonAnode) { r = 255 - r; g = 255 - g; b = 255 - b; }
        PwmWrite(Pins::LED_RGB_R, CH_R, r);
        PwmWrite(Pins::LED_RGB_G, CH_G, g);
        PwmWrite(Pins::LED_RGB_B, CH_B, b);
    }

    void ApplyColor(ColorLabel color) {
        switch (color) {
            case ColorLabel::FLOOR:   SetRaw(60, 60, 60); break;   // blanco -- piso gris
            case ColorLabel::YELLOW:  SetRaw(255, 170, 0); break;
            case ColorLabel::RED:     SetRaw(255, 0, 0);   break;
            case ColorLabel::BLUE:    SetRaw(0, 0, 255);   break;
            case ColorLabel::BLACK:                                // negro -> apagado
            case ColorLabel::UNKNOWN:
            default:
                SetRaw(0, 0, 0);
                break;
        }
    }
}

void LedTask(void *) {
    RgbLed::Setup();
    RgbLed::SetRaw(0, 0, 0);

    ColorLabel color = ColorLabel::UNKNOWN;

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::LED_STATUS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        LedCommand cmd;
        if (xQueueReceive(g_ledCmdQueue, &cmd, 0) == pdTRUE) {
            color = cmd.color;
        }

        RgbLed::ApplyColor(color);

        Heartbeat(TaskId::LED_STATUS);
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace anónimo (tareas de hardware)

// ===========================================================================
//  [8] MISSIONTASK — merodear, y girar al ver el borde
// ===========================================================================

namespace Mission {

constexpr bool kMotionEnabled = true;

constexpr int kVelocidadCrucero = 60;   // % de PWM al avanzar recto
constexpr int kVelocidadGiroMax = 100;  // % al girar -- máxima, en ambos lados

constexpr uint32_t kStartupDelayMs = 3000;   // tiempo para ubicar el robot en la pista

// Debounce: un solo instante con on_line=true no dispara el giro -- tiene
// que sostenerse sin interrupción al menos este tiempo. Corto a propósito:
// solo filtra ruido de un instante, "pare inmediatamente" sigue siendo
// prácticamente inmediato. Mismo valor que standalones/v2-merodeo/.
constexpr uint32_t kBordeDebounceMs = 50;

// Gracia al entrar a MERODEAR (al arrancar, o al terminar un giro): ignora
// el borde por completo durante este tiempo, para no volver a dispararse
// contra el mismo borde del que recién se alejó. Mismo valor que
// standalones/v2-merodeo/, donde resolvió justo ese problema en banco.
constexpr uint32_t kIgnorarBordeAlEntrarMs = 1000;

// Tasa de giro calibrada en banco anoche (standalones/v2-merodeo/,
// kEvasionGiroMs=2000 para un giro de 180° a kVelocidadGiroMax): de ahí
// sale cuántos ms le toman a un grado.
constexpr float kMsPorGrado = 2000.0f / 180.0f;

constexpr int kGiroAnguloMinDeg = 60;
constexpr int kGiroAnguloMaxDeg = 180;

enum class Phase : uint8_t {
    ARRANQUE = 0,
    MERODEAR,
    GIRAR,
};

inline const char *PhaseName(Phase phase) {
    switch (phase) {
        case Phase::ARRANQUE:  return "ARRANQUE";
        case Phase::MERODEAR:  return "MERODEAR";
        case Phase::GIRAR:     return "GIRAR";
        default:                return "DESCONOCIDA";
    }
}

} // namespace Mission

inline void SetDrive(MotorCommand &m, int left, int right) {
    m.mode  = MotorMode::DRIVE;
    m.left  = (int8_t)constrain(left, -100, 100);
    m.right = (int8_t)constrain(right, -100, 100);
}

void MissionTask(void *) {
    Mission::Phase phase = Mission::Phase::ARRANQUE;
    Mission::Phase last_logged_phase = phase;
    uint32_t phase_started_ms = millis();

    ColorReading last_color{};
    ReflectanceReading last_reflect{};
    uint32_t borde_detectado_desde_ms = 0;   // 0 = no se está confirmando ahora

    // Elegidos UNA vez al entrar a GIRAR, no en cada vuelta -- si no, el
    // ángulo/dirección "temblarían" cuadro a cuadro en vez de ser un solo
    // giro coherente.
    uint32_t giro_duracion_ms = 0;
    bool     giro_a_la_derecha = true;

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::MISSION);
    TickType_t last_wake = xTaskGetTickCount();

    DEBUG_LINK.println("[Mission] v4-merodeo-color -- merodeo puro, sin gripper, LED muestra color en vivo.");

    for (;;) {
        ColorReading c;
        while (xQueueReceive(g_colorQueue, &c, 0) == pdTRUE) last_color = c;
        ReflectanceReading r;
        while (xQueueReceive(g_reflectQueue, &r, 0) == pdTRUE) last_reflect = r;
        HealthReport h;
        while (xQueueReceive(g_healthQueue, &h, 0) == pdTRUE) {
            DEBUG_LINK.printf("[Mission] tareas colgadas, bitmask=0x%02X\n", h.faulted_tasks_bitmask);
        }

        MotorCommand motor;   // por defecto: STOP

        if (Mission::kMotionEnabled) {

            if (phase != last_logged_phase) {
                DEBUG_LINK.printf("[Mission] fase -> %s\n", Mission::PhaseName(phase));
                last_logged_phase = phase;
            }

            switch (phase) {

                case Mission::Phase::ARRANQUE: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kStartupDelayMs) {
                        phase = Mission::Phase::MERODEAR;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Avanza recto. Ignora el borde durante la gracia de
                // entrada (kIgnorarBordeAlEntrarMs); pasada esa gracia,
                // exige que el borde se sostenga kBordeDebounceMs antes de
                // disparar el giro -- un solo instante suelto no cuenta.
                case Mission::Phase::MERODEAR: {
                    const bool en_gracia =
                        (uint32_t)(millis() - phase_started_ms) <= Mission::kIgnorarBordeAlEntrarMs;

                    if (!en_gracia) {
                        const bool en_borde = last_reflect.left_on_line || last_reflect.right_on_line;
                        if (en_borde) {
                            if (borde_detectado_desde_ms == 0) {
                                borde_detectado_desde_ms = millis();
                            } else if ((uint32_t)(millis() - borde_detectado_desde_ms) >= Mission::kBordeDebounceMs) {
                                // Borde confirmado: PARA DE INMEDIATO (motor
                                // se queda en su valor por defecto, STOP --
                                // no se llama a SetDrive) y sortea ángulo +
                                // dirección para el giro que sigue.
                                const uint32_t angulo = Mission::kGiroAnguloMinDeg +
                                    (esp_random() % (uint32_t)(Mission::kGiroAnguloMaxDeg - Mission::kGiroAnguloMinDeg + 1));
                                giro_duracion_ms = (uint32_t)(angulo * Mission::kMsPorGrado);
                                giro_a_la_derecha = (esp_random() % 2) == 0;

                                DEBUG_LINK.printf("[Mission] borde detectado -- girando %u grados hacia %s\n",
                                                   angulo, giro_a_la_derecha ? "la derecha" : "la izquierda");

                                phase = Mission::Phase::GIRAR;
                                phase_started_ms = millis();
                                borde_detectado_desde_ms = 0;
                                break;
                            }
                        } else {
                            borde_detectado_desde_ms = 0;
                        }
                    }
                    SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    break;
                }

                // Gira sobre su propio eje el ángulo/dirección sorteados al
                // entrar aquí, a velocidad máxima en ambos lados. Un lado
                // "avanza" y el otro "retrocede" -- así el robot rota en
                // el lugar en vez de describir un arco.
                case Mission::Phase::GIRAR: {
                    if ((uint32_t)(millis() - phase_started_ms) > giro_duracion_ms) {
                        phase = Mission::Phase::MERODEAR;
                        phase_started_ms = millis();
                    } else if (giro_a_la_derecha) {
                        SetDrive(motor, Mission::kVelocidadGiroMax, -Mission::kVelocidadGiroMax);
                    } else {
                        SetDrive(motor, -Mission::kVelocidadGiroMax, Mission::kVelocidadGiroMax);
                    }
                    break;
                }

                default:
                    break;
            }

        } // if (Mission::kMotionEnabled)

        xQueueOverwrite(g_motorCmdQueue, &motor);

        // LED: refleja el color activo SIEMPRE, sin importar la fase de
        // merodeo -- las dos lógicas son independientes (ver el aviso al
        // principio del archivo).
        LedCommand led;
        led.color = PriorityColor(last_color);
        xQueueOverwrite(g_ledCmdQueue, &led);

        Heartbeat(TaskId::MISSION);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ===========================================================================
//  [9] SUPERVISORTASK, setup() / loop()
// ===========================================================================

namespace {

void SupervisorTask(void *) {
    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::SUPERVISOR);
    TickType_t last_wake = xTaskGetTickCount();

    uint8_t previous_faults = 0;

    for (;;) {
        const uint8_t faulted = WatchdogCheck();

        if (faulted != 0) {
            HealthReport report;
            report.timestamp_ms = millis();
            report.faulted_tasks_bitmask = faulted;
            xQueueOverwrite(g_healthQueue, &report);
        }

        if (faulted != previous_faults) {
            DEBUG_LINK.printf("[Supervisor] tareas colgadas: 0x%02X\n", faulted);
            previous_faults = faulted;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace anónimo

static bool CreateQueues() {
    g_motorCmdQueue = xQueueCreate(1, sizeof(MotorCommand));
    g_ledCmdQueue   = xQueueCreate(1, sizeof(LedCommand));
    g_colorQueue    = xQueueCreate(4, sizeof(ColorReading));
    g_reflectQueue  = xQueueCreate(4, sizeof(ReflectanceReading));
    g_healthQueue   = xQueueCreate(1, sizeof(HealthReport));

    return g_motorCmdQueue && g_ledCmdQueue && g_colorQueue &&
           g_reflectQueue && g_healthQueue;
}

constexpr uint32_t SERIAL_BAUD_RATE = 115200;

static const char *ResetReasonToString(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON (se energizo desde cero)";
        case ESP_RST_EXT:       return "EXTERNO (pin de reset)";
        case ESP_RST_SW:        return "SOFTWARE (esp_restart() o similar)";
        case ESP_RST_PANIC:     return "PANIC (excepcion/crash del firmware)";
        case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";
        case ESP_RST_TASK_WDT:  return "TASK WATCHDOG (una tarea no volvio a tiempo)";
        case ESP_RST_WDT:       return "OTRO WATCHDOG";
        case ESP_RST_DEEPSLEEP: return "DEEP SLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT: el voltaje cayo debajo del minimo -- revisar bateria/alimentacion";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "DESCONOCIDO";
    }
}

void setup() {
    DEBUG_LINK.begin(SERIAL_BAUD_RATE);

    constexpr uint32_t kSerialWaitMs = 1500;
    const uint32_t wait_start = millis();
    while (!DEBUG_LINK && (millis() - wait_start) < kSerialWaitMs) {
        delay(10);
    }
    delay(200);

    DEBUG_LINK.println("\nAthena Rover 2026 - v4-merodeo-color (sin Raspberry Pi, sin gripper)");
    DEBUG_LINK.printf("[Setup] Motivo del ultimo reinicio: %s\n",
                       ResetReasonToString(esp_reset_reason()));

    // -- Switch: SOLO traba de seguridad -------------------------------------
    pinMode(Pins::TEAM_SWITCH_BLUE, INPUT_PULLUP);
    pinMode(Pins::TEAM_SWITCH_RED,  INPUT_PULLUP);
    delay(5);

    RgbLed::Setup();
    DEBUG_LINK.println("[Setup] Esperando el switch (posicion 0 = esperando, cualquier otra arma el robot)...");
    uint32_t blink_ms = millis();
    bool blink_on = false;
    for (;;) {
        const bool blue_closed = digitalRead(Pins::TEAM_SWITCH_BLUE) == LOW;
        const bool red_closed  = digitalRead(Pins::TEAM_SWITCH_RED)  == LOW;
        if (blue_closed != red_closed) break;   // cualquiera de los dos tiros arma el robot

        if ((uint32_t)(millis() - blink_ms) > 300) {
            blink_ms = millis();
            blink_on = !blink_on;
            RgbLed::SetRaw(blink_on ? 40 : 0, blink_on ? 40 : 0, blink_on ? 40 : 0);
        }
        delay(20);
    }
    RgbLed::SetRaw(0, 0, 0);
    DEBUG_LINK.println("[Setup] Switch fuera del centro -- robot armado.");

    if (!CreateQueues()) {
        DEBUG_LINK.println("[FATAL] no se pudieron crear las colas. Arranque detenido.");
        for (;;) delay(1000);
    }

    WatchdogInit();

    xTaskCreatePinnedToCore(SupervisorTask, "Supervisor", TaskStack::SUPERVISOR,
                            nullptr, TaskPriority::SUPERVISOR, nullptr, 0);
    xTaskCreatePinnedToCore(MissionTask, "Mission", TaskStack::MISSION,
                            nullptr, TaskPriority::MISSION, nullptr, 1);
    xTaskCreatePinnedToCore(MotorTask, "Motors", TaskStack::MOTOR_CONTROL,
                            nullptr, TaskPriority::MOTOR_CONTROL, nullptr, 1);
    xTaskCreatePinnedToCore(ColorSensorTask, "ColorSensors", TaskStack::COLOR_SENSOR,
                            nullptr, TaskPriority::COLOR_SENSOR, nullptr, 0);
    xTaskCreatePinnedToCore(ReflectanceTask, "Reflectance", TaskStack::REFLECTANCE,
                            nullptr, TaskPriority::REFLECTANCE, nullptr, 0);
    xTaskCreatePinnedToCore(LedTask, "ColorLed", TaskStack::LED_STATUS,
                            nullptr, TaskPriority::LED_STATUS, nullptr, 0);

    DEBUG_LINK.println("Todas las tareas lanzadas. Merodeando.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
