// ===========================================================================
//  Athena Rover 2026 — Firmware ESP32-S3 AUTÓNOMO (sin Raspberry Pi)
//  Retos del Rover H07 · INTEC · Reymildo & Montse
//
//  VARIANTE v6-mision-completa: integra en una sola secuencia lo validado
//  por separado en v3-color-delantero (caja + sensor de color delantero) y
//  v5-agarrar-bandera (ToF + gripper). Sin Raspberry Pi, sin cámara, sin
//  QTR (el robot todavía no puede doblar sobre las ruedas, así que todo el
//  recorrido es en línea recta) -- pedido explícito para esta prueba.
//
//  SECUENCIA COMPLETA:
//    1. Espera el switch de equipo (ROJO/AZUL) -- define también cuál es
//       el color de la "zona enemiga" más adelante (el opuesto al propio).
//    2. Asegura la CAJA/LLAVE (gripper a kClawClosedLlaveDeg, 128°) --
//       se asume ya puesta bajo el gripper al arrancar, igual que v3.
//    3. Avanza recto hasta ver AMARILLO con el sensor de color delantero
//       (zona de depósito) -- full stop, y suelta la caja.
//    4. GIRA sobre su propio eje para esquivar la caja recién soltada (ver
//       "GIRO DE ESQUIVE TRAS LA CAJA" más abajo) -- no seguir de largo por
//       encima de la caja.
//    5. ESPERA MEDIA (no muy corta, no muy larga -- pensada para que una
//       persona ponga la bandera al frente del robot, a la vista del ToF,
//       ya con la caja fuera del camino gracias al giro del paso anterior).
//    6. Se acerca a la bandera guiado por el ToF -- MISMO algoritmo de
//       v5-agarrar-bandera (avance grueso, full stop, pasos chicos,
//       confirmar rango 3 veces seguidas, recién ahí cerrar el gripper a
//       kClawClosedBanderaDeg). SIN GIRO tras agarrarla -- pedido explícito:
//       sigue recto (distinto del giro de esquive de la caja, paso 4).
//    7. Avanza recto hasta ver el color de la ZONA ENEMIGA (opuesto al
//       equipo elegido en el switch) con el sensor delantero -- full
//       stop, espera, y suelta la bandera.
//    8. TERMINADO.
//
//  POR QUÉ EL SENSOR DE COLOR VA EN EL BUS 1 Y NO EN EL BUS 0
//  ----------------------------------------------------------------------
//  Esta variante necesita el ToF Y el sensor de color delantero A LA VEZ
//  (v3 solo tenía color, v5 solo tenía ToF -- nunca convivieron antes).
//  El 2026-09-08 se confirmó en banco que el TCS34725 y el VL53L1X NO
//  pueden compartir el bus 0x29 ni con el orden correcto de
//  init()/setAddress() -- con el TCS34725 realmente encendido, el init()
//  del ToF fallaba por completo (ver calibracion/tof/README.md, la
//  sección "RESUELTO 2026-09-08 (de verdad)"). El fix real fue separar
//  los buses FÍSICAMENTE: el TCS34725 delantero se recableó al bus I2C
//  nº1 (junto al PCA9685, dirección distinta -- 0x29 vs 0x40, sin choque),
//  el TCS34725 trasero se desconectó, y el ToF se quedó SOLO en el bus 0
//  sin necesitar reasignación. Esta variante asume ese cableado -- ver
//  Pins:: más abajo. Si el TCS34725 delantero sigue en el bus 0 (GPIO8/9)
//  en tu robot, hace falta recablearlo al bus 1 (GPIO47/48) antes de usar
//  este firmware.
//
//  GIRO DE ESQUIVE TRAS LA CAJA (2026-09-09)
//  ----------------------------------------------------------------------
//  Tras soltar la caja en la zona amarilla, el robot gira sobre su propio
//  eje (Phase::ESQUIVAR_CAJA) antes de seguir de largo -- mismo primitivo
//  de giro que v5-agarrar-bandera (un lado ADELANTE, el otro ATRAS, mismo
//  duty), calibrado con pruebas-platformio/08-calibracion-giro/. Ver esa
//  herramienta para cómo se midió Mission::kMsPorGradoEsquive -- el valor
//  de acá es un punto de partida (heredado de v5, bench-confirmado
//  2026-09-07), NO una calibración verificada para el chasis actual: ese
//  bench se corrió antes del fix de nombres FL/RL
//  (pruebas-platformio/07-caracterizacion-motores/) y antes de reemplazar
//  el motor trasero derecho, así que la fricción/torque real del chasis
//  ya cambió. Recalibrar con 08-calibracion-giro/ antes de confiar en el
//  ángulo real que resulta.
//
//  SIN GIRO DESPUÉS DE AGARRAR LA BANDERA
//  ----------------------------------------------------------------------
//  v5-agarrar-bandera giraba ~90° tras cerrar el gripper, para demostrar
//  que podía transportar la bandera y no solo sujetarla en el sitio. Acá
//  NO: pedido explícito para esta prueba -- distinto del giro de esquive
//  de arriba, que sí está habilitado (motivo distinto: alejarse de la caja,
//  no demostrar transporte).
//
//  EL ToF NO SIRVE PARA LA CAJA
//  ----------------------------------------------------------------------
//  El ToF está montado a una altura que nunca va a poder sensar la caja
//  (confirmado por el equipo) -- por eso la caja se agarra al arrancar
//  (se asume ya puesta) y se suelta por COLOR (zona amarilla), nunca por
//  distancia. El ToF solo entra en juego para la bandera, más adelante en
//  la secuencia, cuando ya no hay caja de por medio.
//
//  HARDWARE:
//    · 2x L298N            -> 4 motores (cada driver mueve 2)
//    · 1x VL53L1X (I2C0)   -> distancia a la bandera, SOLO en su bus
//    · 1x PCA9685 (I2C1)   -> el servo del gripper
//    · 1x TCS34725 (I2C1)  -> sensor de color DELANTERO (recableado, ver arriba)
//    · LED RGB (1x)        -> color sensado durante la búsqueda de zona,
//                             indicador de fase durante el resto (ver LedTask)
//    · Switch 3 posiciones -> elige equipo Y arma el robot
//
//  NADA de QTR, NADA de cámara, NADA de TCS34725 trasero (desconectado).
//
//  ÍNDICE
//    [1] Configuración: pines, prioridades, stacks, periodos
//    [2] Tipos compartidos entre tareas
//    [3] Colas y mutex del bus I2C nº1
//    [4] Watchdog cooperativo (heartbeats)
//    [5] Driver PCA9685 (servo por I2C)
//    [6] Driver TCS34725 (color, bus I2C nº1)
//    [7] Clasificación de color -> etiqueta
//    [8] Driver VL53L1X (Pololu, bus I2C nº0)
//    [9] Tareas de hardware (motores, gripper, color, ToF, LED)
//    [10] MissionTask — el cerebro: caja, zona amarilla, bandera, zona enemiga
//    [11] SupervisorTask, setup() / loop()
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <freertos/semphr.h>
#include <esp_system.h>

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
    // Sin TCS34725 acá -- ver el aviso grande al principio del archivo.
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;
    constexpr uint8_t TOF_XSHUT = 3;

    // -------- Bus I2C nº1: PCA9685 (gripper) + TCS34725 delantero ----------
    // El TCS34725 delantero vive ACÁ, no en el bus 0 -- ver el aviso grande
    // al principio del archivo.
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;
    constexpr uint8_t TCS_LED_FRONT = 18;   // LED de iluminación del TCS34725, GPIO aparte (no I2C)

    // -------- LED RGB: color sensado / indicador de fase --------------------
    constexpr uint8_t LED_RGB_R = 39;
    constexpr uint8_t LED_RGB_G = 38;
    constexpr uint8_t LED_RGB_B = 41;

    // -------- Switch de 3 posiciones: elige equipo Y arma el robot ---------
    constexpr uint8_t TEAM_SWITCH_BLUE = 40;
    constexpr uint8_t TEAM_SWITCH_RED  = 21;
}

namespace I2CAddr {
    constexpr uint8_t PCA9685  = 0x40;
    constexpr uint8_t TCS34725 = 0x29;   // bus 1
    // VL53L1X: dirección de fábrica, sin reasignar -- el bus 0 es solo
    // suyo, no hace falta (ver el aviso grande al principio del archivo).
    constexpr uint8_t VL53L1X = 0x29;    // bus 0
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

// Ángulos ya calibrados -- ver pruebas-platformio/06-calibracion-gripper/.
constexpr int kClawOpenDeg          = 0;
constexpr int kClawClosedLlaveDeg   = 128;
constexpr int kClawClosedBanderaDeg = 65;

namespace TaskPriority {
    constexpr UBaseType_t SUPERVISOR      = 6;
    constexpr UBaseType_t MOTOR_CONTROL   = 5;
    constexpr UBaseType_t MISSION         = 4;
    constexpr UBaseType_t GRIPPER_CONTROL = 3;
    constexpr UBaseType_t COLOR_SENSOR    = 2;
    constexpr UBaseType_t TOF_SENSOR      = 2;
    constexpr UBaseType_t LED_STATUS      = 1;
}

namespace TaskStack {
    constexpr uint32_t SUPERVISOR      = 3072;
    constexpr uint32_t MISSION         = 4096;
    constexpr uint32_t MOTOR_CONTROL   = 3072;
    constexpr uint32_t GRIPPER_CONTROL = 3072;
    constexpr uint32_t COLOR_SENSOR    = 3584;
    constexpr uint32_t TOF_SENSOR      = 3072;
    constexpr uint32_t LED_STATUS      = 2048;
}

namespace TaskPeriodMs {
    constexpr uint32_t SUPERVISOR    = 200;
    constexpr uint32_t MISSION       = 50;
    constexpr uint32_t MOTOR_CONTROL = 20;
    constexpr uint32_t GRIPPER       = 50;
    constexpr uint32_t COLOR_SENSOR  = 100;
    constexpr uint32_t TOF_SENSOR    = 50;   // igual al ranging period del VL53L1X
    constexpr uint32_t LED_STATUS    = 100;
}

constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;
constexpr uint32_t MISSION_FAILSAFE_TIMEOUT_MS = 500;

enum class TaskId : uint8_t {
    MOTOR_CONTROL = 0,
    GRIPPER_CONTROL,
    COLOR_SENSOR,
    TOF_SENSOR,
    LED_STATUS,
    MISSION,
    COUNT
};

#define DEBUG_LINK Serial0   // consola por el puerto UART del DevKit -- así flashear y
                              // monitorear usan el mismo cable, sin cambiar de puerto

// ===========================================================================
//  [2] TIPOS COMPARTIDOS ENTRE TAREAS
// ===========================================================================

enum class TeamColor     : uint8_t { NONE = 0, RED = 1, BLUE = 2 };
enum class MotorMode     : uint8_t { STOP = 0, DRIVE = 1 };
enum class GripperAction : uint8_t { OPEN = 0, CLOSE_LLAVE = 1, CLOSE_BANDERA = 2 };
enum class ColorLabel    : uint8_t { UNKNOWN = 0, BLACK, YELLOW, RED, BLUE, FLOOR };

// Fase "visible" para el LED -- MissionTask decide, LedTask solo traduce.
// COLOR: mientras se busca una zona por color, el LED muestra el color
// sensado en vivo (igual que v3). Las demás fases muestran un indicador
// de progreso fijo/parpadeante (igual que v5).
enum class EstadoVisible : uint8_t {
    COLOR = 0,        // buscando zona -- LED = color sensado en vivo
    BUSCANDO_BANDERA, // acercamiento grueso a la bandera (ToF)
    AJUSTANDO,        // parado, midiendo/dando pasos chicos hacia la bandera
    AGARRADA,         // gripper cerrado sobre la bandera, avanzando
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
    EstadoVisible estado = EstadoVisible::COLOR;
    ColorLabel    color  = ColorLabel::UNKNOWN;   // solo se usa si estado == COLOR
};

struct ColorReading {
    uint32_t   timestamp_ms = 0;
    ColorLabel color        = ColorLabel::UNKNOWN;
    bool       valid        = false;
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
//  [3] COLAS Y MUTEX DEL BUS I2C Nº1
// ===========================================================================
//
//   MissionTask     --> motorCmdQueue   (1, overwrite) --> MotorTask
//                   --> gripperCmdQueue (4, FIFO)       --> GripperTask
//                   --> ledCmdQueue     (1, overwrite)  --> LedTask
//   ColorSensorTask --> colorQueue      (4, FIFO)       --> MissionTask
//   TofSensorTask   --> tofQueue        (4, FIFO)       --> MissionTask
//   SupervisorTask  --> healthQueue     (1, overwrite)  --> MissionTask (solo log)

static QueueHandle_t g_motorCmdQueue   = nullptr;
static QueueHandle_t g_gripperCmdQueue = nullptr;
static QueueHandle_t g_ledCmdQueue     = nullptr;
static QueueHandle_t g_colorQueue      = nullptr;
static QueueHandle_t g_tofQueue        = nullptr;
static QueueHandle_t g_healthQueue     = nullptr;

// Protege el bus Nº1 (Wire1): el TCS34725 delantero (ColorSensorTask,
// núcleo 0) y el PCA9685 (GripperTask, núcleo 1) lo comparten ahora que el
// color se movió a este bus -- ver el aviso grande al principio del
// archivo. El bus Nº0 (Wire, ToF) lo toca ÚNICAMENTE TofSensorTask, así
// que no necesita mutex propio.
static SemaphoreHandle_t g_i2c1Mutex = nullptr;
constexpr uint32_t I2C1_LOCK_TIMEOUT_MS = 50;

static inline bool I2c1Lock() {
    return xSemaphoreTake(g_i2c1Mutex, pdMS_TO_TICKS(I2C1_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline void I2c1Unlock() {
    xSemaphoreGive(g_i2c1Mutex);
}

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
//  [6] DRIVER TCS34725 — color delantero, bus I2C nº1
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

    bool WriteReg(uint8_t reg, uint8_t value) {
        Wire1.beginTransmission(I2CAddr::TCS34725);
        Wire1.write(CMD_BIT | reg);
        Wire1.write(value);
        return Wire1.endTransmission() == 0;
    }

    bool ReadReg(uint8_t reg, uint8_t &out) {
        Wire1.beginTransmission(I2CAddr::TCS34725);
        Wire1.write(CMD_BIT | reg);
        if (Wire1.endTransmission() != 0) return false;
        if (Wire1.requestFrom((int)I2CAddr::TCS34725, 1) != 1) return false;
        out = (uint8_t)Wire1.read();
        return true;
    }

    bool Init() {
        uint8_t id = 0;
        if (!ReadReg(REG_ID, id)) return false;
        if (id != 0x44 && id != 0x4D) return false;

        if (!WriteReg(REG_ATIME, ATIME_24MS)) return false;
        if (!WriteReg(REG_CONTROL, GAIN_4X)) return false;
        if (!WriteReg(REG_ENABLE, ENABLE_PON)) return false;
        delay(3);
        return WriteReg(REG_ENABLE, ENABLE_PON | ENABLE_AEN);
    }

    bool Read(Rgbc &out) {
        Wire1.beginTransmission(I2CAddr::TCS34725);
        Wire1.write(CMD_BIT | CMD_AUTO_INC | REG_CDATAL);
        if (Wire1.endTransmission() != 0) return false;
        if (Wire1.requestFrom((int)I2CAddr::TCS34725, 8) != 8) return false;

        out.c = (uint16_t)(Wire1.read() | (Wire1.read() << 8));
        out.r = (uint16_t)(Wire1.read() | (Wire1.read() << 8));
        out.g = (uint16_t)(Wire1.read() | (Wire1.read() << 8));
        out.b = (uint16_t)(Wire1.read() | (Wire1.read() << 8));
        return true;
    }
}

// ===========================================================================
//  [7] CLASIFICACIÓN DE COLOR -> ETIQUETA
// ===========================================================================
//
//  Umbrales del DELANTERO, reajustados 2026-09-07 -- MISMOS que
//  calibracion/color/detector-tcs/src/main.cpp y v3-color-delantero. Se
//  calibraron con el TCS34725 en el bus 0; ahora vive en el bus 1 (ver el
//  aviso grande al principio del archivo) -- si el bus nuevo le cambia el
//  ruido eléctrico percibido, la primera señal de alerta sería confusiones
//  entre GRIS y AMARILLO, igual que las que ya pasaron el 2026-09-07.

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

inline const char *ColorLabelName(ColorLabel label) {
    switch (label) {
        case ColorLabel::BLACK:   return "NEGRO";
        case ColorLabel::YELLOW:  return "AMARILLO";
        case ColorLabel::RED:     return "ROJO";
        case ColorLabel::BLUE:    return "AZUL";
        case ColorLabel::FLOOR:   return "GRIS/PISO";
        case ColorLabel::UNKNOWN:
        default:                  return "DESCONOCIDO";
    }
}

// ===========================================================================
//  [8] DRIVER VL53L1X (Pololu) — bus I2C nº0, SOLO
// ===========================================================================

namespace Tof {
    constexpr uint16_t TIMING_BUDGET_US = 50000;   // 50 ms
    constexpr uint32_t RANGING_PERIOD_MS = 50;
}

VL53L1X g_tof;

bool TofBringUp() {
    g_tof.setBus(&Wire);
    g_tof.setTimeout(500);
    // Sin setAddress(): el bus 0 es solo suyo, se queda en su dirección de
    // fábrica -- ver el aviso grande al principio del archivo.
    if (!g_tof.init()) return false;
    g_tof.setDistanceMode(VL53L1X::Long);
    g_tof.setMeasurementTimingBudget(Tof::TIMING_BUDGET_US);
    g_tof.startContinuous(Tof::RANGING_PERIOD_MS);
    return true;
}

// ===========================================================================
//  [9] TAREAS DE HARDWARE
// ===========================================================================

namespace {

// ---------------------------------------------------------------------------
//  9.1  MotorTask — 4 motores a través de 2 drivers L298N
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

// speed > 0 -> "avanza" -- misma convención confirmada en banco que
// v3-color-delantero/v4-merodeo-color/v5-agarrar-bandera.
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
//  9.2  GripperTask — 1 servo vía PCA9685 (bus I2C nº1, con mutex)
// ---------------------------------------------------------------------------

void GripperTask(void *) {
    bool pca_ok = false;
    if (I2c1Lock()) {
        pca_ok = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
        if (pca_ok) {
            Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
        }
        I2c1Unlock();
    }
    if (!pca_ok) {
        DEBUG_LINK.println("[Gripper] PCA9685 no responde. Reintentando en segundo plano.");
    }

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::GRIPPER);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        if (!pca_ok && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            if (I2c1Lock()) {
                pca_ok = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
                if (pca_ok) {
                    pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
                }
                I2c1Unlock();
            }
        }

        GripperCommand cmd;
        if (xQueueReceive(g_gripperCmdQueue, &cmd, 0) == pdTRUE && pca_ok) {
            if (I2c1Lock()) {
                switch (cmd.action) {
                    case GripperAction::OPEN:
                        pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
                        break;
                    case GripperAction::CLOSE_LLAVE:
                        pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawClosedLlaveDeg));
                        break;
                    case GripperAction::CLOSE_BANDERA:
                        pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawClosedBanderaDeg));
                        break;
                    default:
                        break;
                }
                I2c1Unlock();
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
//  9.3  ColorSensorTask — TCS34725 delantero, bus I2C nº1 (con mutex)
// ---------------------------------------------------------------------------

void ColorSensorTask(void *) {
    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);   // iluminación fija: no depender de la luz del salón

    bool front_ok = false;
    if (I2c1Lock()) {
        front_ok = Tcs34725::Init();
        I2c1Unlock();
    }
    if (!front_ok) DEBUG_LINK.println("[Color] TCS34725 delantero no responde (bus I2C 1).");

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::COLOR_SENSOR);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        if (!front_ok && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            if (I2c1Lock()) {
                front_ok = Tcs34725::Init();
                I2c1Unlock();
            }
        }

        ColorReading reading;
        reading.timestamp_ms = millis();

        if (front_ok) {
            Tcs34725::Rgbc sample;
            bool read_ok = false;
            if (I2c1Lock()) {
                read_ok = Tcs34725::Read(sample);
                I2c1Unlock();
            }
            if (read_ok) {
                reading.color = ClassifyColor(sample);
                reading.valid = true;
            } else {
                front_ok = false;
            }
        }

        PushDropOldest(g_colorQueue, reading);

        Heartbeat(TaskId::COLOR_SENSOR);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  9.4  TofSensorTask — VL53L1X, bus I2C nº0 (sin mutex, único dueño)
// ---------------------------------------------------------------------------

void TofSensorTask(void *) {
    pinMode(Pins::TOF_XSHUT, OUTPUT);
    digitalWrite(Pins::TOF_XSHUT, HIGH);   // fuera de reset; sin reasignación (ver aviso arriba)

    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);   // bus 0 -- SOLO el ToF

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
//  9.5  LedTask — color sensado (fases COLOR) o indicador de fase (resto)
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
            case ColorLabel::FLOOR:   SetRaw(60, 60, 60);  break;   // blanco -- piso gris
            case ColorLabel::YELLOW:  SetRaw(255, 170, 0); break;
            case ColorLabel::RED:     SetRaw(255, 0, 0);   break;
            case ColorLabel::BLUE:    SetRaw(0, 0, 255);   break;
            case ColorLabel::BLACK:
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

    EstadoVisible estado = EstadoVisible::COLOR;
    ColorLabel color = ColorLabel::UNKNOWN;
    uint32_t blink_ms = millis();
    bool blink_on = false;

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::LED_STATUS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        LedCommand cmd;
        if (xQueueReceive(g_ledCmdQueue, &cmd, 0) == pdTRUE) {
            estado = cmd.estado;
            color  = cmd.color;
        }

        if ((uint32_t)(millis() - blink_ms) > 300) {
            blink_ms = millis();
            blink_on = !blink_on;
        }

        switch (estado) {
            case EstadoVisible::COLOR:
                RgbLed::ApplyColor(color);
                break;
            case EstadoVisible::BUSCANDO_BANDERA:
                RgbLed::SetRaw(40, 40, 40);   // blanco tenue fijo
                break;
            case EstadoVisible::AJUSTANDO:
                RgbLed::SetRaw(blink_on ? 255 : 0, blink_on ? 190 : 0, 0);   // amarillo parpadeando
                break;
            case EstadoVisible::AGARRADA:
                RgbLed::SetRaw(0, 255, 0);    // verde fijo
                break;
            case EstadoVisible::TERMINADO:
                RgbLed::SetRaw(0, 255, 0);    // verde fijo
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
//  [10] MISSIONTASK — caja, zona amarilla, bandera, zona enemiga
// ===========================================================================

namespace Mission {

constexpr bool kMotionEnabled = true;

constexpr int kVelocidadCrucero = 60;   // % de PWM al avanzar recto

// -- Arranque / caja --------------------------------------------------------
constexpr uint32_t kStartupDelayMs = 3000;   // tiempo para ubicar el robot y la caja
constexpr uint32_t kGripperSettleCajaMs = 400;         // tiempo mecánico para que el servo llegue
constexpr uint32_t kDelayTrasAsegurarCajaMs = 600;     // margen extra para que el agarre quede firme

// -- Zona amarilla (depósito de la caja) ------------------------------------
constexpr uint32_t kFullStopAntesDeSoltarCajaMs = 400;   // full stop antes de soltar, no se desliza
constexpr uint32_t kGripperSettleAperturaCajaMs = 400;

// -- Giro de esquive tras soltar la caja -------------------------------------
// Ver "GIRO DE ESQUIVE TRAS LA CAJA" al principio del archivo.
// ⚠️ kMsPorGradoEsquive es el valor heredado de v5-agarrar-bandera
// (3000ms/180°, bench-confirmado 2026-09-07) -- PENDIENTE de recalibrar
// para el chasis actual con pruebas-platformio/08-calibracion-giro/. No
// asumir que el ángulo real que resulta hoy es 90° hasta medirlo.
constexpr float kMsPorGradoEsquive = 3000.0f / 180.0f;
constexpr int kGiroEsquiveCajaDeg = 90;
constexpr int kVelocidadGiroEsquive = 100;   // % de PWM -- a fondo, igual que v5 (la única palanca es tiempo)
// true = gira hacia la derecha (visto desde arriba) al esquivar; false =
// hacia la izquierda. Cuál conviene depende de dónde queda la caja/pista
// respecto al robot -- ajustar según la pista real, no es simétrico.
constexpr bool kGiroEsquiveHaciaDerecha = true;
constexpr uint32_t kDuracionGiroEsquiveMs = (uint32_t)(kGiroEsquiveCajaDeg * kMsPorGradoEsquive);

// -- Espera media entre caja y bandera ---------------------------------------
// Pedido explícito: "ni muy corta ni muy larga" -- tiempo para que una
// persona ponga la bandera al frente del robot (ya reorientado por el giro
// de esquive), a la vista del ToF. Ajustar acá si 6 s se queda corto/largo.
constexpr uint32_t kEsperaReacomodoMs = 6000;

// -- Acercamiento a la bandera (ToF) -- MISMOS valores que v5-agarrar-bandera,
// confirmados en banco 2026-09-07 -----------------------------------------
constexpr uint16_t kDistanciaAproximacionMm = 150;
constexpr uint16_t kRangoAgarreMinMm = 56;
constexpr uint16_t kRangoAgarreMaxMm = 60;
constexpr int kVelocidadPaso = 50;
constexpr uint32_t kPasoDuracionMs = 140;
constexpr uint32_t kSettleTrasParoMs = 200;
constexpr int kLecturasConsecutivasRequeridas = 3;
constexpr int kMaxPasosSeguridad = 40;
constexpr uint32_t kGripperSettleBanderaMs = 500;

// -- Tras agarrar la bandera: SIN GIRO, pedido explícito (el robot todavía
// no puede doblar de forma confiable) -- solo un respiro corto y sigue recto.
constexpr uint32_t kEsperaTrasAgarrarBanderaMs = 600;

// -- Zona enemiga (depósito de la bandera) -----------------------------------
constexpr uint32_t kFullStopZonaEnemigaMs = 700;   // "espere un poco" antes de soltar
constexpr uint32_t kGripperSettleSueltaBanderaMs = 400;

enum class Phase : uint8_t {
    ARRANQUE = 0,
    ASEGURAR_CAJA,
    ESPERAR_ANTES_DE_AVANZAR,
    BUSCAR_ZONA_AMARILLA,
    DETENER_ZONA_AMARILLA,
    DEPOSITAR_CAJA,
    ESQUIVAR_CAJA,
    ESPERAR_REACOMODO,
    AVANCE_GRUESO_BANDERA,
    DETENER_PARA_MEDIR,
    PASO_AJUSTE,
    CERRAR_GRIPPER_BANDERA,
    ESPERAR_TRAS_AGARRE,
    AVANZAR_ZONA_ENEMIGA,
    DETENER_ZONA_ENEMIGA,
    SOLTAR_BANDERA,
    TERMINADO,
    FALLO_AJUSTE,
};

inline const char *PhaseName(Phase phase) {
    switch (phase) {
        case Phase::ARRANQUE:                  return "ARRANQUE";
        case Phase::ASEGURAR_CAJA:              return "ASEGURAR_CAJA";
        case Phase::ESPERAR_ANTES_DE_AVANZAR:   return "ESPERAR_ANTES_DE_AVANZAR";
        case Phase::BUSCAR_ZONA_AMARILLA:       return "BUSCAR_ZONA_AMARILLA";
        case Phase::DETENER_ZONA_AMARILLA:      return "DETENER_ZONA_AMARILLA";
        case Phase::DEPOSITAR_CAJA:             return "DEPOSITAR_CAJA";
        case Phase::ESQUIVAR_CAJA:              return "ESQUIVAR_CAJA";
        case Phase::ESPERAR_REACOMODO:          return "ESPERAR_REACOMODO";
        case Phase::AVANCE_GRUESO_BANDERA:      return "AVANCE_GRUESO_BANDERA";
        case Phase::DETENER_PARA_MEDIR:         return "DETENER_PARA_MEDIR";
        case Phase::PASO_AJUSTE:                return "PASO_AJUSTE";
        case Phase::CERRAR_GRIPPER_BANDERA:     return "CERRAR_GRIPPER_BANDERA";
        case Phase::ESPERAR_TRAS_AGARRE:        return "ESPERAR_TRAS_AGARRE";
        case Phase::AVANZAR_ZONA_ENEMIGA:       return "AVANZAR_ZONA_ENEMIGA";
        case Phase::DETENER_ZONA_ENEMIGA:       return "DETENER_ZONA_ENEMIGA";
        case Phase::SOLTAR_BANDERA:             return "SOLTAR_BANDERA";
        case Phase::TERMINADO:                  return "TERMINADO";
        case Phase::FALLO_AJUSTE:               return "FALLO_AJUSTE";
        default:                                return "DESCONOCIDA";
    }
}

} // namespace Mission

inline void SetDrive(MotorCommand &m, int left, int right) {
    m.mode  = MotorMode::DRIVE;
    m.left  = (int8_t)constrain(left, -100, 100);
    m.right = (int8_t)constrain(right, -100, 100);
}

// pvTeam apunta a g_myTeam (global, ver setup()) -- de ahí se deriva
// también g_enemyColor, calculado una sola vez al arrancar la tarea.
void MissionTask(void *pvTeam) {
    const TeamColor team = *reinterpret_cast<TeamColor *>(pvTeam);
    const ColorLabel enemy_color = (team == TeamColor::RED) ? ColorLabel::BLUE : ColorLabel::RED;

    Mission::Phase phase = Mission::Phase::ARRANQUE;
    Mission::Phase last_logged_phase = phase;
    uint32_t phase_started_ms = millis();

    ColorReading last_color{};
    TofReading last_tof{};

    // Cerrojos: en cuanto se ve el color buscado UNA vez en la fase
    // correspondiente, esto pasa a true y ya no vuelve a false -- mismo
    // criterio que v1/v3 (no seguir de largo por una lectura suelta que
    // cambió un instante después).
    bool zona_amarilla_detectada = false;
    bool zona_enemiga_detectada  = false;

    // Log periódico de diagnóstico SOLO en AVANZAR_ZONA_ENEMIGA -- a
    // diferencia del log de cambio de fase de más abajo, este imprime en
    // vivo mientras se busca el color de la zona enemiga, para poder ver
    // qué está leyendo el sensor en el momento exacto en que debería
    // reconocer rojo/azul y no lo hace.
    uint32_t last_color_debug_ms = 0;

    int lecturas_en_rango_seguidas = 0;
    int pasos_dados = 0;
    int sentido_paso = 1;   // -1 = paso hacia atrás, +1 = hacia adelante

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::MISSION);
    TickType_t last_wake = xTaskGetTickCount();

    DEBUG_LINK.printf("[Mission] v6-mision-completa -- equipo=%s, zona enemiga=%s\n",
                       team == TeamColor::RED ? "ROJO" : "AZUL",
                       ColorLabelName(enemy_color));

    for (;;) {
        ColorReading c;
        while (xQueueReceive(g_colorQueue, &c, 0) == pdTRUE) last_color = c;
        TofReading t;
        while (xQueueReceive(g_tofQueue, &t, 0) == pdTRUE) last_tof = t;
        HealthReport h;
        while (xQueueReceive(g_healthQueue, &h, 0) == pdTRUE) {
            DEBUG_LINK.printf("[Mission] tareas colgadas, bitmask=0x%02X\n", h.faulted_tasks_bitmask);
        }

        const ColorLabel color_activo = last_color.valid ? last_color.color : ColorLabel::UNKNOWN;

        MotorCommand motor;   // por defecto: STOP
        GripperCommand gripper;
        bool send_gripper = false;
        EstadoVisible estado_led = EstadoVisible::COLOR;
        ColorLabel led_color = color_activo;

        if (Mission::kMotionEnabled) {

            // --- Cerrojos: detenerse YA, para siempre, al ver el color -----
            if (!zona_amarilla_detectada && phase == Mission::Phase::BUSCAR_ZONA_AMARILLA &&
                color_activo == ColorLabel::YELLOW) {
                zona_amarilla_detectada = true;
                phase = Mission::Phase::DETENER_ZONA_AMARILLA;
                phase_started_ms = millis();
            }
            if (!zona_enemiga_detectada && phase == Mission::Phase::AVANZAR_ZONA_ENEMIGA &&
                color_activo == enemy_color) {
                zona_enemiga_detectada = true;
                phase = Mission::Phase::DETENER_ZONA_ENEMIGA;
                phase_started_ms = millis();
            }

            if (phase == Mission::Phase::AVANZAR_ZONA_ENEMIGA &&
                (uint32_t)(millis() - last_color_debug_ms) > 300) {
                last_color_debug_ms = millis();
                DEBUG_LINK.printf("[Mission] buscando=%s  color=%s  valido=%d\n",
                                   ColorLabelName(enemy_color), ColorLabelName(color_activo),
                                   last_color.valid);
            }

            if (phase != last_logged_phase) {
                DEBUG_LINK.printf("[Mission] fase -> %s (color=%s, tof=%u mm, valido=%d)\n",
                                   Mission::PhaseName(phase), ColorLabelName(color_activo),
                                   last_tof.distance_mm, last_tof.valid);
                last_logged_phase = phase;
            }

            switch (phase) {

                case Mission::Phase::ARRANQUE: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kStartupDelayMs) {
                        phase = Mission::Phase::ASEGURAR_CAJA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::ASEGURAR_CAJA: {
                    gripper.action = GripperAction::CLOSE_LLAVE;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleCajaMs) {
                        phase = Mission::Phase::ESPERAR_ANTES_DE_AVANZAR;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::ESPERAR_ANTES_DE_AVANZAR: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kDelayTrasAsegurarCajaMs) {
                        phase = Mission::Phase::BUSCAR_ZONA_AMARILLA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Sin QTR -- avanza recto sin más hasta que el cerrojo de
                // arriba detecte amarillo.
                case Mission::Phase::BUSCAR_ZONA_AMARILLA: {
                    SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    break;
                }

                // Full stop antes de soltar -- no se abre la pinza mientras
                // el robot todavía se desliza por inercia.
                case Mission::Phase::DETENER_ZONA_AMARILLA: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kFullStopAntesDeSoltarCajaMs) {
                        phase = Mission::Phase::DEPOSITAR_CAJA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::DEPOSITAR_CAJA: {
                    gripper.action = GripperAction::OPEN;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleAperturaCajaMs) {
                        phase = Mission::Phase::ESQUIVAR_CAJA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Gira sobre su propio eje para alejarse de la caja recién
                // soltada, en vez de seguir de largo por encima de ella --
                // ver "GIRO DE ESQUIVE TRAS LA CAJA" al principio del
                // archivo. Mismo primitivo de giro que Phase::GIRAR en
                // v5-agarrar-bandera (un lado ADELANTE, el otro ATRAS).
                case Mission::Phase::ESQUIVAR_CAJA: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kDuracionGiroEsquiveMs) {
                        const int v = Mission::kVelocidadGiroEsquive;
                        if (Mission::kGiroEsquiveHaciaDerecha) {
                            SetDrive(motor, v, -v);
                        } else {
                            SetDrive(motor, -v, v);
                        }
                    } else {
                        phase = Mission::Phase::ESPERAR_REACOMODO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Robot quieto (motor en STOP por defecto): tiempo para que
                // una persona ponga la bandera al frente, a la vista del
                // ToF -- la caja ya quedó fuera del camino por el giro de
                // la fase anterior.
                case Mission::Phase::ESPERAR_REACOMODO: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kEsperaReacomodoMs) {
                        phase = Mission::Phase::AVANCE_GRUESO_BANDERA;
                        phase_started_ms = millis();
                        lecturas_en_rango_seguidas = 0;
                        pasos_dados = 0;
                    }
                    break;
                }

                // ---- A partir de acá: MISMO algoritmo que v5-agarrar-bandera ----

                case Mission::Phase::AVANCE_GRUESO_BANDERA: {
                    estado_led = EstadoVisible::BUSCANDO_BANDERA;
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
                            phase = Mission::Phase::CERRAR_GRIPPER_BANDERA;
                            phase_started_ms = millis();
                        } else {
                            phase_started_ms = millis();
                        }
                        break;
                    }

                    lecturas_en_rango_seguidas = 0;

                    if (pasos_dados >= Mission::kMaxPasosSeguridad) {
                        // Ya no se rinde: tras kMaxPasosSeguridad
                        // correcciones sin caer en rango, agarra la bandera
                        // de todos modos en la posición actual en vez de
                        // quedarse plantado en FALLO_AJUSTE -- a pedido
                        // explícito, termina la misión aunque el agarre no
                        // esté confirmado en rango.
                        phase = Mission::Phase::CERRAR_GRIPPER_BANDERA;
                        phase_started_ms = millis();
                        break;
                    }

                    sentido_paso = (last_tof.valid && last_tof.distance_mm < Mission::kRangoAgarreMinMm)
                        ? -1
                        : 1;
                    phase = Mission::Phase::PASO_AJUSTE;
                    phase_started_ms = millis();
                    break;
                }

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

                case Mission::Phase::CERRAR_GRIPPER_BANDERA: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    gripper.action = GripperAction::CLOSE_BANDERA;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleBanderaMs) {
                        DEBUG_LINK.println("[Mission] bandera agarrada.");
                        phase = Mission::Phase::ESPERAR_TRAS_AGARRE;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Sin giro -- pedido explícito. Robot quieto (STOP por
                // defecto), respiro corto y sigue derecho.
                case Mission::Phase::ESPERAR_TRAS_AGARRE: {
                    estado_led = EstadoVisible::AGARRADA;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kEsperaTrasAgarrarBanderaMs) {
                        phase = Mission::Phase::AVANZAR_ZONA_ENEMIGA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Sin QTR -- avanza recto hasta que el cerrojo de arriba
                // detecte el color de la zona enemiga.
                case Mission::Phase::AVANZAR_ZONA_ENEMIGA: {
                    estado_led = EstadoVisible::COLOR;
                    SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    break;
                }

                case Mission::Phase::DETENER_ZONA_ENEMIGA: {
                    estado_led = EstadoVisible::AGARRADA;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kFullStopZonaEnemigaMs) {
                        phase = Mission::Phase::SOLTAR_BANDERA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::SOLTAR_BANDERA: {
                    estado_led = EstadoVisible::AGARRADA;
                    gripper.action = GripperAction::OPEN;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleSueltaBanderaMs) {
                        phase = Mission::Phase::TERMINADO;
                        phase_started_ms = millis();
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
        led.color  = led_color;
        xQueueOverwrite(g_ledCmdQueue, &led);

        Heartbeat(TaskId::MISSION);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ===========================================================================
//  [11] SUPERVISORTASK, setup() / loop()
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
    g_colorQueue      = xQueueCreate(4, sizeof(ColorReading));
    g_tofQueue        = xQueueCreate(4, sizeof(TofReading));
    g_healthQueue     = xQueueCreate(1, sizeof(HealthReport));

    return g_motorCmdQueue && g_gripperCmdQueue && g_ledCmdQueue &&
           g_colorQueue && g_tofQueue && g_healthQueue;
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

// Sobrescrito por el switch de equipo en setup(), ANTES de crear
// MissionTask -- se pasa por puntero (global, no de pila) igual que
// v1-confirmado.
static TeamColor g_myTeam = TeamColor::BLUE;

void setup() {
    DEBUG_LINK.begin(SERIAL_BAUD_RATE);

    constexpr uint32_t kSerialWaitMs = 1500;
    const uint32_t wait_start = millis();
    while (!DEBUG_LINK && (millis() - wait_start) < kSerialWaitMs) {
        delay(10);
    }
    delay(200);

    DEBUG_LINK.println("\nAthena Rover 2026 - v6-mision-completa (sin Raspberry Pi, sin camara, sin QTR)");
    DEBUG_LINK.printf("[Setup] Motivo del ultimo reinicio: %s\n",
                       ResetReasonToString(esp_reset_reason()));

    // -- Selector de equipo: switch físico de 3 posiciones -------------------
    pinMode(Pins::TEAM_SWITCH_BLUE, INPUT_PULLUP);
    pinMode(Pins::TEAM_SWITCH_RED,  INPUT_PULLUP);
    delay(5);

    // Bus 1 (PCA9685 + TCS34725 delantero) temprano, gripper a 0 (abierto)
    // mientras se espera el switch -- mismo criterio que los standalones
    // anteriores.
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL, 400000);
    Wire1.setTimeOut(25);
    if (Pca9685::Init(Pwm::SERVO_FREQ_HZ)) {
        Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
        DEBUG_LINK.println("[Setup] Gripper a 0 (abierto) mientras se espera el switch de equipo.");
    } else {
        DEBUG_LINK.println("[Setup] PCA9685 no respondio al intentar abrir el gripper temprano "
                            "-- GripperTask lo reintentara despues de elegir equipo.");
    }

    RgbLed::Setup();
    DEBUG_LINK.println("[Setup] Esperando el switch de equipo (posicion 0 = esperando)...");
    uint32_t blink_ms = millis();
    bool blink_on = false;
    for (;;) {
        const bool blue_closed = digitalRead(Pins::TEAM_SWITCH_BLUE) == LOW;
        const bool red_closed  = digitalRead(Pins::TEAM_SWITCH_RED)  == LOW;
        if (blue_closed && !red_closed) { g_myTeam = TeamColor::BLUE; break; }
        if (red_closed  && !blue_closed) { g_myTeam = TeamColor::RED;  break; }

        if ((uint32_t)(millis() - blink_ms) > 300) {
            blink_ms = millis();
            blink_on = !blink_on;
            RgbLed::SetRaw(blink_on ? 40 : 0, blink_on ? 40 : 0, blink_on ? 40 : 0);
        }
        delay(20);
    }
    RgbLed::SetRaw(0, 0, 0);
    DEBUG_LINK.printf("[Setup] Switch de equipo -> %s (zona enemiga = %s)\n",
                       g_myTeam == TeamColor::RED ? "ROJO" : "AZUL",
                       g_myTeam == TeamColor::RED ? "AZUL" : "ROJO");

    if (!CreateQueues()) {
        DEBUG_LINK.println("[FATAL] no se pudieron crear las colas. Arranque detenido.");
        for (;;) delay(1000);
    }

    g_i2c1Mutex = xSemaphoreCreateMutex();
    if (g_i2c1Mutex == nullptr) {
        DEBUG_LINK.println("[FATAL] no se pudo crear el mutex del bus I2C. Arranque detenido.");
        for (;;) delay(1000);
    }

    WatchdogInit();

    xTaskCreatePinnedToCore(SupervisorTask, "Supervisor", TaskStack::SUPERVISOR,
                            nullptr, TaskPriority::SUPERVISOR, nullptr, 0);
    xTaskCreatePinnedToCore(MissionTask, "Mission", TaskStack::MISSION,
                            &g_myTeam, TaskPriority::MISSION, nullptr, 1);
    xTaskCreatePinnedToCore(MotorTask, "Motors", TaskStack::MOTOR_CONTROL,
                            nullptr, TaskPriority::MOTOR_CONTROL, nullptr, 1);
    xTaskCreatePinnedToCore(GripperTask, "Gripper", TaskStack::GRIPPER_CONTROL,
                            nullptr, TaskPriority::GRIPPER_CONTROL, nullptr, 1);
    xTaskCreatePinnedToCore(ColorSensorTask, "ColorSensor", TaskStack::COLOR_SENSOR,
                            nullptr, TaskPriority::COLOR_SENSOR, nullptr, 0);
    xTaskCreatePinnedToCore(TofSensorTask, "ToF", TaskStack::TOF_SENSOR,
                            nullptr, TaskPriority::TOF_SENSOR, nullptr, 0);
    xTaskCreatePinnedToCore(LedTask, "Led", TaskStack::LED_STATUS,
                            nullptr, TaskPriority::LED_STATUS, nullptr, 0);

    DEBUG_LINK.println("Todas las tareas lanzadas. Mision en marcha.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
