// ===========================================================================
//  Athena Rover 2026 — Firmware ESP32-S3 AUTÓNOMO (sin Raspberry Pi)
//  Retos del Rover H07 · INTEC · Reymildo & Montse
//
//  VARIANTE v3-color-delantero: prueba acotada para validar el sensor de
//  color DELANTERO recién recalibrado (ver calibracion/color/), en un
//  contexto de misión real y no solo de banco. Secuencia: asegura la llave,
//  avanza en LÍNEA RECTA (sin QTR, sin evasión de borde -- a propósito, ver
//  más abajo), y cuando el sensor de color ve AMARILLO se detiene y suelta
//  la llave ahí mismo. Durante TODO el recorrido, el LED RGB muestra en vivo
//  qué color está viendo -- blanco=GRIS/piso, azul=AZUL, amarillo=AMARILLO,
//  rojo=ROJO, apagado=NEGRO (o sin lectura).
//
//  ALCANCE DELIBERADAMENTE ACOTADO -- pedido explícito para esta prueba:
//  SOLO sensores de color + gripper + motores/drivers para avanzar recto.
//  NADA de QTR (sin ReflectanceTask, sin evasión de borde) ni de ToF (el
//  VL53L1X, si sigue físicamente conectado, se mantiene en reset todo el
//  tiempo -- ver Pins::TOF_XSHUT). Si esto funciona bien, el siguiente paso
//  es sumar los QTR (ver v1-confirmado/v2-merodeo para cómo se hizo antes).
//  CONSERVAR TAL CUAL -- no modificar este archivo salvo pedido explícito.
// ===========================================================================
//
//  POR QUÉ EL SENSOR DELANTERO Y NO SOLO EL TRASERO (como v1/v2)
//  ----------------------------------------------------------------------
//  v1-confirmado/v2-merodeo retiraron el bus I2C nº0 (TCS34725 delantero +
//  VL53L1X) porque nunca dieron una conexión física confiable en banco. El
//  28 de agosto se calibraron 9 umbrales de clasificación contra 970
//  muestras del delantero con ~14% de error; probado hoy (2026-09-07) bajo
//  luz de oficina + sol directo, AMARILLO clasificaba GRIS la mayoría de las
//  veces -- no porque el sensor estuviera dañado (ROJO y AZUL seguían
//  leyendo sólido), sino porque `AMARILLO_G_MIN` (0.350) quedó justo en
//  medio del rango real de g/c del delantero bajo esa luz (0.328-0.357).
//  Se recapturó AMARILLO (480 muestras, 8 puntos) y se reajustó ESE único
//  umbral a 0.200 -- ver calibracion/color/detector-tcs/src/main.cpp para
//  el detalle completo, con matriz de confusión. Esta variante existe para
//  confirmar que ese ajuste sirve de verdad en una misión, no solo en banco
//  suelto -- y por eso reintroduce el bus I2C nº0 que v1/v2 habían retirado.
//
//  PRIORIDAD: DELANTERO SOBRE TRASERO
//  ----------------------------------------------------------------------
//  Se leen los DOS sensores cada ciclo (el trasero sigue en el bus I2C nº1,
//  junto al PCA9685, igual que en v1/v2). Para el LED y para decidir "¿ya
//  llegué a la zona amarilla?", manda el delantero: si dio una lectura
//  válida este ciclo, se usa esa; solo si el delantero no respondió (cable
//  suelto, todavía reintentando tras un fallo) se cae al trasero. OJO: el
//  trasero usa EL MISMO juego de 9 umbrales que el delantero -- nunca se
//  caracterizó por separado con calibracion/color/ (mismo aviso que ya
//  tenía firmware-esp32/src/main.cpp) -- así que su clasificación aquí es
//  un respaldo de mejor esfuerzo, no algo validado.
//
//  SIN MANIOBRA DE RETROCESO AL LLEGAR AL AMARILLO
//  ----------------------------------------------------------------------
//  v1-confirmado necesitaba retroceder tras detectar amarillo porque su
//  ÚNICO sensor (trasero) ya había pasado la zona en el instante en que la
//  veía -- la llave, al frente, quedaba más allá. Con el sensor DELANTERO
//  como prioridad, el sensor que manda está literalmente donde está la
//  llave: en cuanto ve amarillo, el frente del robot (y la llave) YA están
//  sobre la zona. No hace falta retroceder -- solo un full stop antes de
//  soltar, para no abrir la pinza mientras el robot todavía se desliza.
//
//  SWITCH DE EQUIPO: se conserva SOLO como traba de seguridad (nada se
//  mueve mientras esté en el centro) -- esta prueba no depende del equipo
//  elegido para nada (no busca ninguna bandera), pero el switch ya está
//  cableado y no cuesta nada seguir exigiéndolo antes de energizar motores
//  cerca de alguien parado junto al robot.
//
//  HARDWARE:
//    · 2x L298N            -> 4 motores (cada driver mueve 2)
//    · 1x PCA9685 (I2C1)   -> 1 servo del gripper
//    · 2x TCS34725         -> delantero (I2C0) + trasero (I2C1)
//    · LED RGB (1x)        -> muestra el color detectado, en vivo
//    · Switch 3 posiciones -> traba de seguridad (0=nadie ha armado el robot)
//    · NADA de QTR, NADA de ToF activo (VL53L1X en reset permanente)
//
//  ÍNDICE
//    [1] Configuración: pines, prioridades, stacks, periodos
//    [2] Tipos compartidos entre tareas
//    [3] Colas y mutex del bus I2C nº1
//    [4] Watchdog cooperativo (heartbeats)
//    [5] Driver PCA9685 (servo por I2C)
//    [6] Driver TCS34725 (los dos sensores, por I2C)
//    [7] Clasificación de color -> etiqueta
//    [8] Tareas de hardware (motores, gripper, color, LED)
//    [9] MissionTask — el cerebro
//    [10] SupervisorTask, setup() / loop()
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
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

    // -------- Bus I2C nº0: TCS34725 DELANTERO -------------------------------
    // Reintroducido en esta variante -- v1/v2 lo habían retirado, ver el
    // aviso grande al principio del archivo. Comparte bus con el XSHUT del
    // VL53L1X (misma dirección fija 0x29): hay que mantenerlo en reset, no
    // hace falta usarlo para nada más.
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;
    constexpr uint8_t TOF_XSHUT = 3;        // en reset (LOW) todo el tiempo
    constexpr uint8_t TCS_LED_FRONT = 18;   // LED de iluminación, activo en alto

    // -------- Bus I2C nº1: TCS34725 TRASERO + PCA9685 (servos) --------------
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    // -------- LED RGB: muestra el color detectado, en vivo ------------------
    constexpr uint8_t LED_RGB_R = 39;
    constexpr uint8_t LED_RGB_G = 38;
    constexpr uint8_t LED_RGB_B = 41;

    // -------- Switch de 3 posiciones: SOLO traba de seguridad --------------
    // Mismo cableado físico que firmware-esp32/ y v1/v2 -- ver el aviso ahí:
    // el tiro "AZUL" quedó en GPIO 40 y el "ROJO" en GPIO 21.
    constexpr uint8_t TEAM_SWITCH_BLUE = 40;
    constexpr uint8_t TEAM_SWITCH_RED  = 21;
}

namespace I2CAddr {
    constexpr uint8_t PCA9685  = 0x40;
    constexpr uint8_t TCS34725 = 0x29;   // fija -- por eso los dos sensores van en buses separados
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

namespace TaskPriority {
    constexpr UBaseType_t SUPERVISOR      = 6;
    constexpr UBaseType_t MOTOR_CONTROL   = 5;
    constexpr UBaseType_t MISSION         = 4;
    constexpr UBaseType_t GRIPPER_CONTROL = 3;
    constexpr UBaseType_t COLOR_SENSOR    = 2;
    constexpr UBaseType_t LED_STATUS      = 1;
}

namespace TaskStack {
    constexpr uint32_t SUPERVISOR      = 3072;
    constexpr uint32_t MISSION         = 4096;
    constexpr uint32_t MOTOR_CONTROL   = 3072;
    constexpr uint32_t GRIPPER_CONTROL = 3072;
    constexpr uint32_t COLOR_SENSOR    = 3584;
    constexpr uint32_t LED_STATUS      = 2048;
}

namespace TaskPeriodMs {
    constexpr uint32_t SUPERVISOR    = 200;
    constexpr uint32_t MISSION       = 50;
    constexpr uint32_t MOTOR_CONTROL = 20;
    constexpr uint32_t GRIPPER       = 50;
    constexpr uint32_t COLOR_SENSOR  = 100;
    constexpr uint32_t LED_STATUS    = 100;
}

constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;
constexpr uint32_t MISSION_FAILSAFE_TIMEOUT_MS = 500;

enum class TaskId : uint8_t {
    MOTOR_CONTROL = 0,
    GRIPPER_CONTROL,
    COLOR_SENSOR,
    LED_STATUS,
    MISSION,
    COUNT
};

#define DEBUG_LINK Serial

// ===========================================================================
//  [2] TIPOS COMPARTIDOS ENTRE TAREAS
// ===========================================================================

enum class TeamColor     : uint8_t { NONE = 0, RED = 1, BLUE = 2 };
enum class MotorMode     : uint8_t { STOP = 0, DRIVE = 1 };
enum class GripperAction : uint8_t { OPEN = 0, CLOSE_LLAVE = 1, CLOSE_BANDERA = 2 };
enum class ColorLabel    : uint8_t { UNKNOWN = 0, BLACK, YELLOW, RED, BLUE, FLOOR };

struct MotorCommand {
    MotorMode mode = MotorMode::STOP;
    int8_t    left  = 0;
    int8_t    right = 0;
};

struct GripperCommand {
    GripperAction action = GripperAction::OPEN;
};

struct LedCommand {
    ColorLabel color = ColorLabel::UNKNOWN;
};

// Los DOS sensores en cada lectura -- ver "PRIORIDAD: DELANTERO SOBRE
// TRASERO" al principio del archivo para cómo se combinan.
struct ColorReading {
    uint32_t   timestamp_ms = 0;
    ColorLabel front        = ColorLabel::UNKNOWN;
    bool       front_valid  = false;
    ColorLabel back         = ColorLabel::UNKNOWN;
    bool       back_valid   = false;
};

struct HealthReport {
    uint32_t timestamp_ms          = 0;
    uint8_t  faulted_tasks_bitmask = 0;
};

// El delantero manda si dio lectura válida este ciclo; si no, se cae al
// trasero. UNKNOWN si ninguno de los dos respondió.
inline ColorLabel PriorityColor(const ColorReading &c) {
    if (c.front_valid) return c.front;
    if (c.back_valid)  return c.back;
    return ColorLabel::UNKNOWN;
}

// ===========================================================================
//  [3] COLAS Y MUTEX DEL BUS I2C Nº1
// ===========================================================================
//
//   MissionTask --> motorCmdQueue   (1, overwrite) --> MotorTask
//               --> gripperCmdQueue (4, FIFO)      --> GripperTask
//               --> ledCmdQueue     (1, overwrite) --> LedTask
//   ColorSensorTask --> colorQueue  (4, FIFO)       --> MissionTask
//   SupervisorTask  --> healthQueue (1, overwrite)  --> MissionTask (solo log)

static QueueHandle_t g_motorCmdQueue   = nullptr;
static QueueHandle_t g_gripperCmdQueue = nullptr;
static QueueHandle_t g_ledCmdQueue     = nullptr;
static QueueHandle_t g_colorQueue      = nullptr;
static QueueHandle_t g_healthQueue     = nullptr;

// Solo protege el bus Nº1 (Wire1): el TCS34725 trasero (ColorSensorTask,
// núcleo 0) y el PCA9685 (GripperTask, núcleo 1) lo comparten. El bus Nº0
// (Wire, delantero) lo toca ÚNICAMENTE ColorSensorTask -- nadie más -- así
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
//  [6] DRIVER TCS34725 — los DOS sensores, cada uno en su bus
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
//  [7] CLASIFICACIÓN DE COLOR -> ETIQUETA
// ===========================================================================
//
//  Umbrales del DELANTERO, reajustados 2026-09-07 (segunda vuelta: los 9
//  de nuevo, con AMARILLO y GRIS recapturados bajo la luz de hoy -- el
//  primer reajuste, que solo tocó AMARILLO_G_MIN, dejó falsos positivos de
//  AMARILLO sobre piso GRIS). Ver el detalle completo (matriz de
//  confusión, por qué GRIS necesitó recapturarse) en
//  calibracion/color/detector-tcs/src/main.cpp, que usa estos MISMOS
//  números. Si se recalibra de nuevo, actualizar los dos archivos.
//
//  ⚠️ Se usan también para el TRASERO a falta de una calibración propia --
//  ver "PRIORIDAD: DELANTERO SOBRE TRASERO" al principio del archivo.

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
//  [8] TAREAS DE HARDWARE
// ===========================================================================

namespace {

// ---------------------------------------------------------------------------
//  8.1  MotorTask — 4 motores a través de 2 drivers L298N
// ---------------------------------------------------------------------------

struct Motor {
    uint8_t in1, in2, en, ledc_channel;
};

// Rueda trasera izquierda con IN1/IN2 intercambiados por su cableado físico
// -- ver la nota larga en firmware-esp32/src/main.cpp.
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
// físico tal como está cableado HOY (con el gripper cerrando bien sobre la
// llave, el chasis iba hacia atrás con la convención "speed < 0 = avanza"
// copiada de v1-confirmado/firmware-esp32; se invirtió acá, en este
// archivo nada más). v1-confirmado quedó "CONSERVAR TAL CUAL" con la
// convención vieja porque se confirmó en su propia sesión de banco -- no
// se tocó ahí para no invalidar esa confirmación, pero evidentemente algo
// cambió físicamente desde entonces (un cable de motor re-sentado al
// tocar otras conexiones, lo más probable). Sin maniobra de retroceso en
// esta variante (ver el aviso al principio del archivo), así que no hace
// falta invertir_direccion aquí -- si el sentido vuelve a estar al revés
// en el futuro, es señal de que el cableado cambió otra vez, no de que
// este booleano esté mal elegido: medir en banco antes de voltearlo de
// nuevo (ver la nota larga sobre esto en v1-confirmado/src/main.cpp).
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
//  8.2  GripperTask — 1 servo vía PCA9685 (bus I2C nº1)
// ---------------------------------------------------------------------------

constexpr int kClawOpenDeg          = 0;
constexpr int kClawClosedLlaveDeg   = 128;
constexpr int kClawClosedBanderaDeg = 65;

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
//  8.3  ColorSensorTask — LOS DOS TCS34725, delantero (Wire) y trasero (Wire1)
// ---------------------------------------------------------------------------
//  Delantero: bus Nº0, sin mutex (nadie más lo toca). Trasero: bus Nº1,
//  protegido por g_i2c1Mutex (lo comparte con GripperTask/PCA9685).

void ColorSensorTask(void *) {
    // VL53L1X en reset ANTES de abrir el bus 0 -- comparte dirección fija
    // 0x29 con el TCS34725 delantero; si se deja flotando, corrompe sus
    // lecturas (ver el aviso grande al principio del archivo).
    pinMode(Pins::TOF_XSHUT, OUTPUT);
    digitalWrite(Pins::TOF_XSHUT, LOW);

    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);
    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);   // iluminación fija: no depender de la luz del salón

    bool front_ok = Tcs34725::Init(Wire);
    if (!front_ok) DEBUG_LINK.println("[Color] sensor DELANTERO no responde (bus I2C 0).");

    bool back_ok = false;
    if (I2c1Lock()) {
        back_ok = Tcs34725::Init(Wire1);
        I2c1Unlock();
    }
    if (!back_ok) DEBUG_LINK.println("[Color] sensor TRASERO no responde (bus I2C 1).");

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::COLOR_SENSOR);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        if ((!front_ok || !back_ok) && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            if (!front_ok) front_ok = Tcs34725::Init(Wire);
            if (!back_ok && I2c1Lock()) {
                back_ok = Tcs34725::Init(Wire1);
                I2c1Unlock();
            }
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
            bool read_ok = false;
            if (I2c1Lock()) {
                read_ok = Tcs34725::Read(Wire1, sample);
                I2c1Unlock();
            }
            if (read_ok) {
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
//  8.4  LedTask — muestra en vivo el color detectado (prioridad delantero)
// ---------------------------------------------------------------------------
//  blanco=GRIS/piso, azul=AZUL, amarillo=AMARILLO, rojo=ROJO, apagado=NEGRO
//  (o sin lectura de ningún sensor) -- pedido explícito para esta prueba.

namespace RgbLed {
    constexpr uint8_t CH_R = 4;
    constexpr uint8_t CH_G = 5;
    constexpr uint8_t CH_B = 6;

    // POLARIDAD CONFIRMADA: cátodo común, duty alto = canal más brillante.
    constexpr bool kCommonAnode = false;

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
            case ColorLabel::BLACK:                                // negro -> apagado, pedido explícito
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
//  [9] MISSIONTASK — el cerebro
// ===========================================================================

namespace Mission {

// Interruptor de emergencia: en false, no se mueve motor ni gripper -- solo
// se leen sensores y el LED sigue mostrando el color, en vivo. Útil para
// probar los sensores en banco sin que el robot se mueva.
constexpr bool kMotionEnabled = true;

constexpr int kVelocidadCrucero = 60;   // % de PWM al avanzar recto

constexpr uint32_t kStartupDelayMs  = 3000;  // tiempo para cargar la llave y ubicar el robot
constexpr uint32_t kGripperSettleMs = 400;   // tiempo mecánico para que el servo llegue

// Pausa ADICIONAL después de que el servo ya llegó a cerrado sobre la
// llave, antes de arrancar a avanzar -- pedido explícito: más margen que
// el simple asentamiento mecánico del servo (kGripperSettleMs), para que
// el agarre quede firme y estable antes de que el robot empiece a
// moverse. Se sigue esperando kGripperSettleMs primero (el servo tiene que
// llegar físicamente) y ESTA pausa corre después, no en paralelo.
constexpr uint32_t kDelayTrasAsegurarLlaveMs = 600;

// Full stop al detectar amarillo, antes de soltar -- para no abrir la
// pinza mientras el robot todavía se desliza por inercia (evita que la
// llave rebote al caer). "Pequeño tiempo", pedido explícito: se detiene
// del todo y suelta pronto, no una pausa larga.
constexpr uint32_t kFullStopMs      = 400;

enum class Phase : uint8_t {
    ARRANQUE = 0,
    ASEGURAR_LLAVE,
    // Pausa adicional tras el asentamiento del servo, ANTES de arrancar a
    // avanzar -- ver kDelayTrasAsegurarLlaveMs. Fase propia (no un segundo
    // timer dentro de ASEGURAR_LLAVE) para que el log de fases distinga
    // claramente "el servo ya llegó" de "ya se puede avanzar".
    ESPERAR_ANTES_DE_AVANZAR,
    BUSCAR_ZONA_NEUTRA,
    DETENER_ZONA_NEUTRA,
    DEPOSITAR_LLAVE,
    TERMINADO,
};

inline const char *PhaseName(Phase phase) {
    switch (phase) {
        case Phase::ARRANQUE:                  return "ARRANQUE";
        case Phase::ASEGURAR_LLAVE:             return "ASEGURAR_LLAVE";
        case Phase::ESPERAR_ANTES_DE_AVANZAR:   return "ESPERAR_ANTES_DE_AVANZAR";
        case Phase::BUSCAR_ZONA_NEUTRA:         return "BUSCAR_ZONA_NEUTRA";
        case Phase::DETENER_ZONA_NEUTRA:        return "DETENER_ZONA_NEUTRA";
        case Phase::DEPOSITAR_LLAVE:            return "DEPOSITAR_LLAVE";
        case Phase::TERMINADO:                  return "TERMINADO";
        default:                                return "DESCONOCIDA";
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

    ColorReading last_reading{};
    // Cerrojo: en cuanto se ve amarillo UNA vez en BUSCAR_ZONA_NEUTRA, esto
    // pasa a true y ya no vuelve a false -- mismo criterio que v1-confirmado
    // (el robot no debe seguir de largo por una lectura suelta que cambió
    // un instante después).
    bool zona_neutra_detectada = false;

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::MISSION);
    TickType_t last_wake = xTaskGetTickCount();

    DEBUG_LINK.println("[Mission] v3-color-delantero -- sin QTR, sin ToF, prioridad al sensor delantero.");

    for (;;) {
        // --- 0. Refrescar la última lectura de color -----------------------
        ColorReading c;
        while (xQueueReceive(g_colorQueue, &c, 0) == pdTRUE) last_reading = c;
        HealthReport h;
        while (xQueueReceive(g_healthQueue, &h, 0) == pdTRUE) {
            DEBUG_LINK.printf("[Mission] tareas colgadas, bitmask=0x%02X\n", h.faulted_tasks_bitmask);
        }

        const ColorLabel color_activo = PriorityColor(last_reading);

        MotorCommand motor;               // por defecto: STOP
        GripperCommand gripper;
        bool send_gripper = false;

        if (Mission::kMotionEnabled) {

            // --- 0b. Cerrojo de zona neutra: detenerse YA, para siempre ----
            if (!zona_neutra_detectada && phase == Mission::Phase::BUSCAR_ZONA_NEUTRA &&
                color_activo == ColorLabel::YELLOW) {
                zona_neutra_detectada = true;
                phase = Mission::Phase::DETENER_ZONA_NEUTRA;
                phase_started_ms = millis();
            }

            if (phase != last_logged_phase) {
                DEBUG_LINK.printf("[Mission] fase -> %s\n", Mission::PhaseName(phase));
                last_logged_phase = phase;
            }

            switch (phase) {

                case Mission::Phase::ARRANQUE: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kStartupDelayMs) {
                        phase = Mission::Phase::ASEGURAR_LLAVE;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::ASEGURAR_LLAVE: {
                    gripper.action = GripperAction::CLOSE_LLAVE;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleMs) {
                        phase = Mission::Phase::ESPERAR_ANTES_DE_AVANZAR;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Servo ya asentado sobre la llave; espera un poco más
                // (kDelayTrasAsegurarLlaveMs) antes de arrancar a avanzar
                // -- pedido explícito, para que el agarre quede firme.
                // Quieto: motor se queda en su valor por defecto (STOP).
                case Mission::Phase::ESPERAR_ANTES_DE_AVANZAR: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kDelayTrasAsegurarLlaveMs) {
                        phase = Mission::Phase::BUSCAR_ZONA_NEUTRA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Sin QTR, sin evasión de borde -- avanza recto sin más
                // hasta que el cerrojo de arriba detecte amarillo. A
                // propósito para esta prueba acotada, ver el aviso grande
                // al principio del archivo.
                case Mission::Phase::BUSCAR_ZONA_NEUTRA: {
                    SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    break;
                }

                // Full stop antes de soltar -- el sensor que manda (el
                // delantero) está donde está la llave, así que a diferencia
                // de v1-confirmado NO hace falta retroceder: en cuanto vio
                // amarillo, el frente del robot ya está sobre la zona.
                case Mission::Phase::DETENER_ZONA_NEUTRA: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kFullStopMs) {
                        phase = Mission::Phase::DEPOSITAR_LLAVE;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::DEPOSITAR_LLAVE: {
                    gripper.action = GripperAction::OPEN;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleMs) {
                        phase = Mission::Phase::TERMINADO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::TERMINADO:
                default:
                    break;
            }

        } // if (Mission::kMotionEnabled)

        xQueueOverwrite(g_motorCmdQueue, &motor);
        if (send_gripper) xQueueSend(g_gripperCmdQueue, &gripper, 0);

        // LED: refleja el color activo SIEMPRE, sin importar la fase --
        // pedido explícito ("durante todo ese proceso").
        LedCommand led;
        led.color = color_activo;
        xQueueOverwrite(g_ledCmdQueue, &led);

        Heartbeat(TaskId::MISSION);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ===========================================================================
//  [10] SUPERVISORTASK, setup() / loop()
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
    g_healthQueue     = xQueueCreate(1, sizeof(HealthReport));

    return g_motorCmdQueue && g_gripperCmdQueue && g_ledCmdQueue &&
           g_colorQueue && g_healthQueue;
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

    DEBUG_LINK.println("\nAthena Rover 2026 - v3-color-delantero (sin Raspberry Pi, sin QTR, sin ToF)");
    DEBUG_LINK.printf("[Setup] Motivo del ultimo reinicio: %s\n",
                       ResetReasonToString(esp_reset_reason()));

    // -- Switch de equipo: SOLO traba de seguridad ---------------------------
    // Esta variante no usa el equipo para nada (no busca ninguna bandera),
    // pero se sigue exigiendo salir del centro antes de mover un motor --
    // ver el aviso al principio del archivo.
    pinMode(Pins::TEAM_SWITCH_BLUE, INPUT_PULLUP);
    pinMode(Pins::TEAM_SWITCH_RED,  INPUT_PULLUP);
    delay(5);

    // Bus 1 (PCA9685) abierto temprano para dejar el gripper en 0 (abierto)
    // mientras se espera el switch -- mismo criterio que v1-confirmado.
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL, 400000);
    Wire1.setTimeOut(25);
    if (Pca9685::Init(Pwm::SERVO_FREQ_HZ)) {
        Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
        DEBUG_LINK.println("[Setup] Gripper a 0 (abierto) mientras se espera el switch.");
    } else {
        DEBUG_LINK.println("[Setup] PCA9685 no respondio al intentar abrir el gripper temprano "
                            "-- GripperTask lo reintentara despues.");
    }

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

    g_i2c1Mutex = xSemaphoreCreateMutex();
    if (g_i2c1Mutex == nullptr) {
        DEBUG_LINK.println("[FATAL] no se pudo crear el mutex del bus I2C. Arranque detenido.");
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
    xTaskCreatePinnedToCore(ColorSensorTask, "ColorSensors", TaskStack::COLOR_SENSOR,
                            nullptr, TaskPriority::COLOR_SENSOR, nullptr, 0);
    xTaskCreatePinnedToCore(LedTask, "ColorLed", TaskStack::LED_STATUS,
                            nullptr, TaskPriority::LED_STATUS, nullptr, 0);

    DEBUG_LINK.println("Todas las tareas lanzadas. Mision en marcha.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
