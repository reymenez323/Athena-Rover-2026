// ===========================================================================
//  Athena Rover 2026 — Firmware ESP32-S3 AUTÓNOMO (sin Raspberry Pi)
//  Retos del Rover H07 · INTEC · Reymildo & Montse
//
//  VARIANTE v5-agarrar-bandera: avanza recto, usa el ToF (VL53L1X) para
//  saber cuándo la bandera está al alcance del gripper, se detiene POR
//  COMPLETO para medir con precisión, ajusta su posición con pasos chicos
//  si hace falta, cierra el gripper solo cuando confirma el rango de
//  agarre de forma sostenida (no con una sola lectura suelta), y se aleja
//  con la bandera agarrada.
//
//  SIN RASPBERRY PI, SIN CÁMARA: "detectar la bandera" acá es 100% el ToF
//  viendo un objeto a la distancia de agarre -- no hay reconocimiento de
//  color ni de forma. Esa parte (identificar que ES la bandera y no otra
//  cosa) es trabajo de la cámara + Edge Impulse en la misión real
//  (raspberry-pi/), pedido explícito del reglamento -- acá solo se prueba
//  la mecánica de acercarse y agarrar, con el objeto ya puesto a mano.
//  CONSERVAR TAL CUAL -- no modificar este archivo salvo pedido explícito.
// ===========================================================================
//
//  DISTANCIA DE AGARRE: 56-62 mm, CONFIRMADA EN BANCO 2026-09-07
//  ----------------------------------------------------------------------
//  Con calibracion/tof/firmware/, sosteniendo la bandera roja y la azul a
//  mano frente al sensor: ambas agarraron bien en ~60 mm (roja probada en
//  56-62, azul en 56-59 -- un solo umbral compartido alcanza para las
//  dos). Fuera de ese rango (probado a 72 mm) el agarre falla. No hace
//  falta un umbral distinto por color.
//
//  POR QUÉ PARAR POR COMPLETO ANTES DE MEDIR, Y POR QUÉ EN PASOS CHICOS
//  ----------------------------------------------------------------------
//  Pedido explícito: el robot tiene que estar COMPLETAMENTE QUIETO al
//  momento de cerrar el gripper -- cerrar a mitad de un frenado, con el
//  chasis todavía asentándose, arriesga cerrar sobre el aire o empujar la
//  bandera antes de agarrarla. Por eso el ciclo de acercamiento fino es
//  parar -> esperar a que la lectura se asiente -> medir -> si no está en
//  rango, UN paso chico (adelante si está lejos, atrás si está cerca) ->
//  parar de nuevo -> repetir. Nunca se cierra el gripper con el robot en
//  movimiento.
//
//  "CONSTANTEMENTE EN RANGO": no basta una lectura suelta -- exige
//  kLecturasConsecutivasRequeridas lecturas seguidas (cada una con el
//  robot ya detenido y asentado) dentro de 56-62 mm antes de cerrar. Una
//  sola lectura en rango puede ser ruido del sensor; varias seguidas, con
//  el robot quieto entre medio, son un agarre de verdad al alcance.
//
//  SIN TCS34725 EN ESTA VARIANTE
//  ----------------------------------------------------------------------
//  El ToF y el TCS34725 delantero comparten el bus I2C nº0 (misma
//  dirección fija 0x29) y la reasignación de dirección que los separaría
//  no está funcionando todavía (ver calibracion/tof/README.md, hallazgo
//  del 2026-09-07) -- así que, igual que esa herramienta de banco, ESTA
//  variante asume que el ToF es el ÚNICO dispositivo en el bus 0.
//
//  HARDWARE:
//    · 2x L298N            -> 4 motores
//    · 1x VL53L1X (I2C0)   -> distancia a la bandera
//    · 1x PCA9685 (I2C1)   -> el servo del gripper
//    · LED RGB (1x)        -> indicador de fase (ver LedTask)
//    · Switch 3 posiciones -> traba de seguridad
//
//  NADA de QTR, NADA de sensores de color en esta variante.
//
//  ÍNDICE
//    [1] Configuración: pines, prioridades, stacks, periodos
//    [2] Tipos compartidos entre tareas
//    [3] Colas
//    [4] Watchdog cooperativo (heartbeats)
//    [5] Driver PCA9685 (servo por I2C)
//    [6] Driver VL53L1X (Pololu)
//    [7] Tareas de hardware (motores, gripper, ToF, LED)
//    [8] MissionTask — el cerebro: acercarse, afinar, agarrar, alejarse
//    [9] SupervisorTask, setup() / loop()
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <esp_system.h>   // esp_reset_reason()

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

    // -------- Bus I2C nº0: VL53L1X (ToF), SOLO -------------------------------
    // Sin TCS34725 delantero acá -- ver el aviso al principio del archivo.
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;
    constexpr uint8_t TOF_XSHUT = 3;

    // -------- Bus I2C nº1: PCA9685 (gripper) --------------------------------
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    // -------- LED RGB: indicador de fase ------------------------------------
    constexpr uint8_t LED_RGB_R = 39;
    constexpr uint8_t LED_RGB_G = 38;
    constexpr uint8_t LED_RGB_B = 41;

    // -------- Switch de 3 posiciones: SOLO traba de seguridad --------------
    constexpr uint8_t TEAM_SWITCH_BLUE = 40;
    constexpr uint8_t TEAM_SWITCH_RED  = 21;
}

namespace I2CAddr {
    constexpr uint8_t PCA9685 = 0x40;
    // Dirección de fábrica del VL53L1X. Sin reasignar -- ver el aviso
    // grande al principio del archivo (setAddress() no funciona todavía).
    constexpr uint8_t VL53L1X_BOOT_ADDR = 0x29;
}

namespace ServoChannel {
    constexpr uint8_t CLAW = 0;
}

namespace Pwm {
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;
    constexpr uint8_t  MOTOR_RESOLUTION = 8;

    constexpr uint32_t RGB_FREQ_HZ    = 5000;
    constexpr uint8_t  RGB_RESOLUTION = 8;

    constexpr uint32_t SERVO_FREQ_HZ  = 50;
    constexpr uint16_t SERVO_TICK_MIN = 205;
    constexpr uint16_t SERVO_TICK_MAX = 410;
}

constexpr int kClawOpenDeg          = 0;
constexpr int kClawClosedBanderaDeg = 65;

namespace TaskPriority {
    constexpr UBaseType_t SUPERVISOR      = 6;
    constexpr UBaseType_t MOTOR_CONTROL   = 5;
    constexpr UBaseType_t MISSION         = 4;
    constexpr UBaseType_t GRIPPER_CONTROL = 3;
    constexpr UBaseType_t TOF_SENSOR      = 2;
    constexpr UBaseType_t LED_STATUS      = 1;
}

namespace TaskStack {
    constexpr uint32_t SUPERVISOR      = 3072;
    constexpr uint32_t MISSION         = 4096;
    constexpr uint32_t MOTOR_CONTROL   = 3072;
    constexpr uint32_t GRIPPER_CONTROL = 3072;
    constexpr uint32_t TOF_SENSOR      = 3072;
    constexpr uint32_t LED_STATUS      = 2048;
}

namespace TaskPeriodMs {
    constexpr uint32_t SUPERVISOR    = 200;
    constexpr uint32_t MISSION       = 50;
    constexpr uint32_t MOTOR_CONTROL = 20;
    constexpr uint32_t GRIPPER       = 50;
    constexpr uint32_t TOF_SENSOR    = 50;   // igual al ranging period del VL53L1X
    constexpr uint32_t LED_STATUS    = 100;
}

constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;
constexpr uint32_t MISSION_FAILSAFE_TIMEOUT_MS = 500;

enum class TaskId : uint8_t {
    MOTOR_CONTROL = 0,
    GRIPPER_CONTROL,
    TOF_SENSOR,
    LED_STATUS,
    MISSION,
    COUNT
};

#define DEBUG_LINK Serial

// ===========================================================================
//  [2] TIPOS COMPARTIDOS ENTRE TAREAS
// ===========================================================================

enum class MotorMode     : uint8_t { STOP = 0, DRIVE = 1 };
enum class GripperAction : uint8_t { OPEN = 0, CLOSE_BANDERA = 1 };

// Fase de la misión, para que LedTask sepa qué mostrar sin duplicar la
// máquina de estados -- MissionTask es dueño de decidir, LedTask solo
// traduce a color.
enum class EstadoVisible : uint8_t {
    BUSCANDO = 0,     // acercamiento grueso
    AJUSTANDO,        // parado, midiendo/dando pasos chicos
    AGARRADA,         // gripper cerrado, alejándose
    TERMINADO,
    FALLO,            // se agotaron los pasos de ajuste sin confirmar rango
};

struct MotorCommand {
    MotorMode mode = MotorMode::STOP;
    int8_t    left  = 0;
    int8_t    right = 0;
};

struct GripperCommand {
    GripperAction action = GripperAction::OPEN;
};

struct LedCommand {
    EstadoVisible estado = EstadoVisible::BUSCANDO;
};

struct TofReading {
    uint32_t timestamp_ms = 0;
    uint16_t distance_mm  = 0;
    bool     valid        = false;
};

struct HealthReport {
    uint32_t timestamp_ms          = 0;
    uint8_t  faulted_tasks_bitmask = 0;
};

// ===========================================================================
//  [3] COLAS
// ===========================================================================
//
//   MissionTask   --> motorCmdQueue   (1, overwrite) --> MotorTask
//                 --> gripperCmdQueue (4, FIFO)       --> GripperTask
//                 --> ledCmdQueue     (1, overwrite)  --> LedTask
//   TofSensorTask --> tofQueue        (4, FIFO)       --> MissionTask
//   SupervisorTask--> healthQueue     (1, overwrite)  --> MissionTask (solo log)
//
//  Sin mutex de bus I2C: cada bus (0 y 1) lo toca UNA sola tarea.

static QueueHandle_t g_motorCmdQueue   = nullptr;
static QueueHandle_t g_gripperCmdQueue = nullptr;
static QueueHandle_t g_ledCmdQueue     = nullptr;
static QueueHandle_t g_tofQueue        = nullptr;
static QueueHandle_t g_healthQueue     = nullptr;

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
//  [5] DRIVER PCA9685 — servo del gripper, bus I2C nº1
// ===========================================================================

namespace Pca9685 {
    constexpr uint8_t REG_MODE1    = 0x00;
    constexpr uint8_t REG_MODE2    = 0x01;
    constexpr uint8_t REG_LED0_ON_L = 0x06;
    constexpr uint8_t REG_PRESCALE = 0xFE;

    constexpr uint8_t MODE1_RESTART = 0x80;
    constexpr uint8_t MODE1_AI      = 0x20;
    constexpr uint8_t MODE1_SLEEP   = 0x10;
    constexpr uint8_t MODE2_OUTDRV  = 0x04;

    bool WriteReg(uint8_t reg, uint8_t value) {
        Wire1.beginTransmission(I2CAddr::PCA9685);
        Wire1.write(reg);
        Wire1.write(value);
        return Wire1.endTransmission() == 0;
    }

    bool ReadReg(uint8_t reg, uint8_t &out) {
        Wire1.beginTransmission(I2CAddr::PCA9685);
        Wire1.write(reg);
        if (Wire1.endTransmission(false) != 0) return false;
        if (Wire1.requestFrom((int)I2CAddr::PCA9685, 1) != 1) return false;
        out = (uint8_t)Wire1.read();
        return true;
    }

    bool Init(uint32_t freq_hz) {
        if (!WriteReg(REG_MODE1, MODE1_SLEEP)) return false;
        const uint32_t prescale = (25000000UL / (4096UL * freq_hz)) - 1UL;
        if (!WriteReg(REG_PRESCALE, (uint8_t)prescale)) return false;
        if (!WriteReg(REG_MODE1, MODE1_AI)) return false;
        delayMicroseconds(500);
        if (!WriteReg(REG_MODE1, MODE1_AI | MODE1_RESTART)) return false;
        if (!WriteReg(REG_MODE2, MODE2_OUTDRV)) return false;
        uint8_t check = 0;
        return ReadReg(REG_MODE1, check);
    }

    bool SetChannel(uint8_t channel, uint16_t ticks) {
        if (channel > 15) return false;
        if (ticks > 4095) ticks = 4095;
        Wire1.beginTransmission(I2CAddr::PCA9685);
        Wire1.write(REG_LED0_ON_L + 4 * channel);
        Wire1.write(0x00);
        Wire1.write(0x00);
        Wire1.write((uint8_t)(ticks & 0xFF));
        Wire1.write((uint8_t)(ticks >> 8));
        return Wire1.endTransmission() == 0;
    }
}

static uint16_t ServoAngleToTicks(int angle_deg) {
    angle_deg = constrain(angle_deg, 0, 180);
    return (uint16_t)(Pwm::SERVO_TICK_MIN +
        ((uint32_t)(Pwm::SERVO_TICK_MAX - Pwm::SERVO_TICK_MIN) * (uint32_t)angle_deg) / 180UL);
}

// ===========================================================================
//  [6] DRIVER VL53L1X (Pololu) — bus I2C nº0, SOLO (ver aviso al principio)
// ===========================================================================

namespace Tof {
    constexpr uint16_t TIMING_BUDGET_US = 50000;   // 50 ms
    constexpr uint32_t RANGING_PERIOD_MS = 50;
}

VL53L1X g_tof;

bool TofBringUp() {
    g_tof.setBus(&Wire);
    g_tof.setTimeout(500);
    // Sin setAddress(): se deja en su dirección de fábrica (0x29) a
    // propósito -- ver el aviso grande al principio del archivo.
    if (!g_tof.init()) return false;
    g_tof.setDistanceMode(VL53L1X::Long);
    g_tof.setMeasurementTimingBudget(Tof::TIMING_BUDGET_US);
    g_tof.startContinuous(Tof::RANGING_PERIOD_MS);
    return true;
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

// speed > 0 -> "avanza" -- misma convención confirmada en banco hoy que
// v3-color-delantero/v4-merodeo-color (NO la vieja "speed < 0" de
// v1-confirmado/v2-merodeo, que ya no coincide con este robot).
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

// ---------------------------------------------------------------------------
//  7.2  GripperTask — 1 servo vía PCA9685 (bus I2C nº1)
// ---------------------------------------------------------------------------

void GripperTask(void *) {
    bool pca_ok = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
    if (pca_ok) {
        Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
    } else {
        DEBUG_LINK.println("[Gripper] PCA9685 no responde. Reintentando en segundo plano.");
    }

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::GRIPPER);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        if (!pca_ok && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            pca_ok = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
            if (pca_ok) Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
        }

        GripperCommand cmd;
        if (xQueueReceive(g_gripperCmdQueue, &cmd, 0) == pdTRUE && pca_ok) {
            switch (cmd.action) {
                case GripperAction::OPEN:
                    pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
                    break;
                case GripperAction::CLOSE_BANDERA:
                    pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawClosedBanderaDeg));
                    break;
                default:
                    break;
            }
        }

        Heartbeat(TaskId::GRIPPER_CONTROL);
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
//  7.3  TofSensorTask — VL53L1X, bus I2C nº0
// ---------------------------------------------------------------------------

void TofSensorTask(void *) {
    pinMode(Pins::TOF_XSHUT, OUTPUT);
    digitalWrite(Pins::TOF_XSHUT, HIGH);   // fuera de reset; sin la coreografía
                                            // de reasignación (ver aviso arriba)

    bool tof_ok = TofBringUp();
    if (!tof_ok) DEBUG_LINK.println("[ToF] VL53L1X no responde. Reintentando en segundo plano.");

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::TOF_SENSOR);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        if (!tof_ok && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            tof_ok = TofBringUp();
        }

        TofReading reading;
        reading.timestamp_ms = millis();

        if (tof_ok) {
            if (g_tof.dataReady()) {
                reading.distance_mm = g_tof.read(false);
                reading.valid = !g_tof.timeoutOccurred() &&
                                 g_tof.ranging_data.range_status == VL53L1X::RangeValid;
                if (g_tof.timeoutOccurred()) tof_ok = false;
            }
        }

        PushDropOldest(g_tofQueue, reading);

        Heartbeat(TaskId::TOF_SENSOR);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  7.4  LedTask — indicador de fase (ver EstadoVisible)
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
}

void LedTask(void *) {
    RgbLed::Setup();
    RgbLed::SetRaw(0, 0, 0);

    EstadoVisible estado = EstadoVisible::BUSCANDO;
    uint32_t blink_ms = millis();
    bool blink_on = false;

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::LED_STATUS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        LedCommand cmd;
        if (xQueueReceive(g_ledCmdQueue, &cmd, 0) == pdTRUE) {
            estado = cmd.estado;
        }

        if ((uint32_t)(millis() - blink_ms) > 300) {
            blink_ms = millis();
            blink_on = !blink_on;
        }

        switch (estado) {
            case EstadoVisible::BUSCANDO:
                RgbLed::SetRaw(40, 40, 40);           // blanco tenue fijo
                break;
            case EstadoVisible::AJUSTANDO:
                RgbLed::SetRaw(blink_on ? 255 : 0, blink_on ? 190 : 0, 0);   // amarillo parpadeando
                break;
            case EstadoVisible::AGARRADA:
                RgbLed::SetRaw(0, 255, 0);            // verde fijo
                break;
            case EstadoVisible::TERMINADO:
                RgbLed::SetRaw(0, 255, 0);            // verde fijo
                break;
            case EstadoVisible::FALLO:
                RgbLed::SetRaw(blink_on ? 255 : 0, 0, 0);   // rojo parpadeando
                break;
        }

        Heartbeat(TaskId::LED_STATUS);
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace anónimo (tareas de hardware)

// ===========================================================================
//  [8] MISSIONTASK — acercarse, afinar, agarrar, alejarse
// ===========================================================================

namespace Mission {

constexpr bool kMotionEnabled = true;

constexpr uint32_t kStartupDelayMs = 3000;   // tiempo para ubicar el robot y la bandera

// -- Acercamiento grueso --------------------------------------------------
constexpr int kVelocidadCrucero = 60;   // % de PWM al avanzar recto
// Umbral para pasar del acercamiento grueso al ajuste fino -- generoso a
// propósito: mejor empezar a medir con cuidado un poco antes de tiempo que
// arriesgar pasarse del rango de agarre (56-62 mm) entre una lectura del
// ToF y la siguiente yendo a velocidad de crucero.
constexpr uint16_t kDistanciaAproximacionMm = 150;

// -- Rango de agarre, confirmado en banco 2026-09-07 -----------------------
constexpr uint16_t kRangoAgarreMinMm = 56;
constexpr uint16_t kRangoAgarreMaxMm = 62;

// -- Ajuste fino: parar, medir, paso chico, repetir ------------------------
constexpr int kVelocidadPaso = 30;            // % de PWM, mucho más lento que crucero
constexpr uint32_t kPasoDuracionMs = 150;     // un paso chico, no un tramo largo
constexpr uint32_t kSettleTrasParoMs = 200;   // asentar el chasis y refrescar el ToF antes de confiar en la lectura
constexpr int kLecturasConsecutivasRequeridas = 3;   // "constantemente en rango", no una lectura suelta
// Tope de seguridad: si tras esta cantidad de pasos todavía no confirma
// el rango, algo anda mal (la bandera se movió, el ToF da problemas) --
// mejor avisar con el LED que seguir empujando indefinidamente.
constexpr int kMaxPasosSeguridad = 40;

// -- Tras agarrar: asentar, girar, alejarse --------------------------------
constexpr uint32_t kGripperSettleMs = 500;
constexpr float kMsPorGrado = 2000.0f / 180.0f;   // misma tasa calibrada que v4-merodeo-color
constexpr int kGiroTrasAgarreDeg = 90;
constexpr int kVelocidadGiroMax = 100;
constexpr uint32_t kAvanceTrasGiroMs = 2500;

enum class Phase : uint8_t {
    ARRANQUE = 0,
    AVANCE_GRUESO,
    DETENER_PARA_MEDIR,
    PASO_AJUSTE,
    CERRAR_GRIPPER,
    GIRAR,
    ALEJARSE,
    TERMINADO,
    FALLO_AJUSTE,
};

inline const char *PhaseName(Phase phase) {
    switch (phase) {
        case Phase::ARRANQUE:            return "ARRANQUE";
        case Phase::AVANCE_GRUESO:       return "AVANCE_GRUESO";
        case Phase::DETENER_PARA_MEDIR:  return "DETENER_PARA_MEDIR";
        case Phase::PASO_AJUSTE:         return "PASO_AJUSTE";
        case Phase::CERRAR_GRIPPER:      return "CERRAR_GRIPPER";
        case Phase::GIRAR:               return "GIRAR";
        case Phase::ALEJARSE:            return "ALEJARSE";
        case Phase::TERMINADO:           return "TERMINADO";
        case Phase::FALLO_AJUSTE:        return "FALLO_AJUSTE";
        default:                         return "DESCONOCIDA";
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

    TofReading last_tof{};
    int lecturas_en_rango_seguidas = 0;
    int pasos_dados = 0;
    // -1 = paso hacia atrás (muy cerca), +1 = hacia adelante (muy lejos) --
    // decidido al ENTRAR a PASO_AJUSTE, con la última lectura de
    // DETENER_PARA_MEDIR, para no cambiar de opinión a mitad del paso.
    int sentido_paso = 1;

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::MISSION);
    TickType_t last_wake = xTaskGetTickCount();

    DEBUG_LINK.println("[Mission] v5-agarrar-bandera -- ToF + gripper, sin camara ni QTR.");

    for (;;) {
        TofReading t;
        while (xQueueReceive(g_tofQueue, &t, 0) == pdTRUE) last_tof = t;
        HealthReport h;
        while (xQueueReceive(g_healthQueue, &h, 0) == pdTRUE) {
            DEBUG_LINK.printf("[Mission] tareas colgadas, bitmask=0x%02X\n", h.faulted_tasks_bitmask);
        }

        MotorCommand motor;   // por defecto: STOP
        GripperCommand gripper;
        bool send_gripper = false;
        EstadoVisible estado_led = EstadoVisible::BUSCANDO;

        if (Mission::kMotionEnabled) {

            if (phase != last_logged_phase) {
                DEBUG_LINK.printf("[Mission] fase -> %s (tof=%u mm, valido=%d)\n",
                                   Mission::PhaseName(phase), last_tof.distance_mm, last_tof.valid);
                last_logged_phase = phase;
            }

            switch (phase) {

                case Mission::Phase::ARRANQUE: {
                    estado_led = EstadoVisible::BUSCANDO;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kStartupDelayMs) {
                        phase = Mission::Phase::AVANCE_GRUESO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Avanza recto hasta que el ToF dé una lectura válida
                // dentro de la distancia de aproximación -- ahí se pasa al
                // ajuste fino, SIEMPRE con un full stop primero (nunca se
                // mide ni se cierra el gripper en movimiento).
                case Mission::Phase::AVANCE_GRUESO: {
                    estado_led = EstadoVisible::BUSCANDO;
                    if (last_tof.valid && last_tof.distance_mm <= Mission::kDistanciaAproximacionMm) {
                        phase = Mission::Phase::DETENER_PARA_MEDIR;
                        phase_started_ms = millis();
                        lecturas_en_rango_seguidas = 0;
                        pasos_dados = 0;
                        break;
                    }
                    SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    break;
                }

                // Robot COMPLETAMENTE QUIETO (motor en su valor por
                // defecto, STOP). Espera a que el chasis se asiente y el
                // ToF refresque una lectura nueva antes de confiar en
                // ella -- kSettleTrasParoMs cubre las dos cosas.
                case Mission::Phase::DETENER_PARA_MEDIR: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kSettleTrasParoMs) {
                        break;   // sigue quieto, todavía asentando
                    }

                    const bool en_rango = last_tof.valid &&
                        last_tof.distance_mm >= Mission::kRangoAgarreMinMm &&
                        last_tof.distance_mm <= Mission::kRangoAgarreMaxMm;

                    if (en_rango) {
                        ++lecturas_en_rango_seguidas;
                        DEBUG_LINK.printf("[Mission] en rango (%u mm), %d/%d lecturas seguidas\n",
                                           last_tof.distance_mm, lecturas_en_rango_seguidas,
                                           Mission::kLecturasConsecutivasRequeridas);
                        if (lecturas_en_rango_seguidas >= Mission::kLecturasConsecutivasRequeridas) {
                            phase = Mission::Phase::CERRAR_GRIPPER;
                            phase_started_ms = millis();
                        } else {
                            // Sigue confirmando: otra vuelta de espera+medida,
                            // sin moverse -- reinicia el reloj de asentamiento
                            // para la siguiente lectura.
                            phase_started_ms = millis();
                        }
                        break;
                    }

                    // Fuera de rango (o sin lectura válida, que se trata
                    // como "muy lejos"): se perdió la racha de lecturas
                    // en rango, y hace falta un paso de ajuste.
                    lecturas_en_rango_seguidas = 0;

                    if (pasos_dados >= Mission::kMaxPasosSeguridad) {
                        phase = Mission::Phase::FALLO_AJUSTE;
                        phase_started_ms = millis();
                        break;
                    }

                    sentido_paso = (last_tof.valid && last_tof.distance_mm < Mission::kRangoAgarreMinMm)
                        ? -1    // muy cerca: paso hacia atrás
                        : 1;    // muy lejos, o sin lectura: paso hacia adelante
                    phase = Mission::Phase::PASO_AJUSTE;
                    phase_started_ms = millis();
                    break;
                }

                // Un paso chico, a velocidad reducida, en la dirección
                // decidida al entrar. Vuelve a DETENER_PARA_MEDIR al
                // terminar -- nunca mide en movimiento.
                case Mission::Phase::PASO_AJUSTE: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    if ((uint32_t)(millis() - phase_started_ms) >= Mission::kPasoDuracionMs) {
                        ++pasos_dados;
                        phase = Mission::Phase::DETENER_PARA_MEDIR;
                        phase_started_ms = millis();
                        break;
                    }
                    const int v = Mission::kVelocidadPaso * sentido_paso;
                    SetDrive(motor, v, v);
                    break;
                }

                // Rango confirmado de forma sostenida Y el robot sigue
                // quieto (no se llama a SetDrive en ningún momento desde
                // que se entró a DETENER_PARA_MEDIR) -- recién ahora se
                // cierra el gripper.
                case Mission::Phase::CERRAR_GRIPPER: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    gripper.action = GripperAction::CLOSE_BANDERA;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleMs) {
                        DEBUG_LINK.println("[Mission] bandera agarrada.");
                        phase = Mission::Phase::GIRAR;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Gira ~90° sobre su propio eje para encarar "otro lado"
                // antes de alejarse -- demuestra que ya puede transportar
                // la bandera agarrada, no solo sujetarla en el sitio.
                case Mission::Phase::GIRAR: {
                    estado_led = EstadoVisible::AGARRADA;
                    const uint32_t duracion_ms = (uint32_t)(Mission::kGiroTrasAgarreDeg * Mission::kMsPorGrado);
                    if ((uint32_t)(millis() - phase_started_ms) > duracion_ms) {
                        phase = Mission::Phase::ALEJARSE;
                        phase_started_ms = millis();
                    } else {
                        SetDrive(motor, Mission::kVelocidadGiroMax, -Mission::kVelocidadGiroMax);
                    }
                    break;
                }

                case Mission::Phase::ALEJARSE: {
                    estado_led = EstadoVisible::AGARRADA;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kAvanceTrasGiroMs) {
                        phase = Mission::Phase::TERMINADO;
                        phase_started_ms = millis();
                    } else {
                        SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    }
                    break;
                }

                case Mission::Phase::TERMINADO:
                    estado_led = EstadoVisible::TERMINADO;
                    break;

                case Mission::Phase::FALLO_AJUSTE:
                default:
                    estado_led = EstadoVisible::FALLO;
                    break;
            }

        } // if (Mission::kMotionEnabled)

        xQueueOverwrite(g_motorCmdQueue, &motor);
        if (send_gripper) xQueueSend(g_gripperCmdQueue, &gripper, 0);

        LedCommand led;
        led.estado = estado_led;
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
    g_motorCmdQueue   = xQueueCreate(1, sizeof(MotorCommand));
    g_gripperCmdQueue = xQueueCreate(4, sizeof(GripperCommand));
    g_ledCmdQueue     = xQueueCreate(1, sizeof(LedCommand));
    g_tofQueue        = xQueueCreate(4, sizeof(TofReading));
    g_healthQueue     = xQueueCreate(1, sizeof(HealthReport));

    return g_motorCmdQueue && g_gripperCmdQueue && g_ledCmdQueue &&
           g_tofQueue && g_healthQueue;
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

    DEBUG_LINK.println("\nAthena Rover 2026 - v5-agarrar-bandera (sin Raspberry Pi, sin camara, sin QTR)");
    DEBUG_LINK.printf("[Setup] Motivo del ultimo reinicio: %s\n",
                       ResetReasonToString(esp_reset_reason()));

    // -- Switch: SOLO traba de seguridad -------------------------------------
    pinMode(Pins::TEAM_SWITCH_BLUE, INPUT_PULLUP);
    pinMode(Pins::TEAM_SWITCH_RED,  INPUT_PULLUP);
    delay(5);

    // Bus 1 (PCA9685) temprano, gripper a 0 (abierto) mientras se espera
    // el switch -- mismo criterio que los standalones anteriores.
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL, 400000);
    Wire1.setTimeOut(25);
    if (Pca9685::Init(Pwm::SERVO_FREQ_HZ)) {
        Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
        DEBUG_LINK.println("[Setup] Gripper a 0 (abierto) mientras se espera el switch.");
    } else {
        DEBUG_LINK.println("[Setup] PCA9685 no respondio al intentar abrir el gripper temprano.");
    }

    RgbLed::Setup();
    DEBUG_LINK.println("[Setup] Esperando el switch (posicion 0 = esperando, cualquier otra arma el robot)...");
    uint32_t blink_ms = millis();
    bool blink_on = false;
    for (;;) {
        const bool blue_closed = digitalRead(Pins::TEAM_SWITCH_BLUE) == LOW;
        const bool red_closed  = digitalRead(Pins::TEAM_SWITCH_RED)  == LOW;
        if (blue_closed != red_closed) break;

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
    xTaskCreatePinnedToCore(GripperTask, "Gripper", TaskStack::GRIPPER_CONTROL,
                            nullptr, TaskPriority::GRIPPER_CONTROL, nullptr, 1);
    xTaskCreatePinnedToCore(TofSensorTask, "ToF", TaskStack::TOF_SENSOR,
                            nullptr, TaskPriority::TOF_SENSOR, nullptr, 0);
    xTaskCreatePinnedToCore(LedTask, "Led", TaskStack::LED_STATUS,
                            nullptr, TaskPriority::LED_STATUS, nullptr, 0);

    DEBUG_LINK.println("Todas las tareas lanzadas. Buscando la bandera.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
