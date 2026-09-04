// ===========================================================================
//  Athena Rover 2026 — Firmware ESP32-S3 AUTÓNOMO (sin Raspberry Pi)
//  Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  Variante de firmware-esp32/src/main.cpp para la DEMOSTRACIÓN: aquí la
//  misión completa (percibir -> decidir -> mover) corre DENTRO del ESP32-S3,
//  en su propia tarea de FreeRTOS (MissionTask). No hay enlace serial con
//  ninguna computadora ni cámara: el robot arranca, hace su ronda solo, y se
//  detiene. Cada subsistema de hardware sigue en su propia tarea, igual que
//  en la versión con Raspberry Pi, comunicadas SOLO por colas explícitas.
//
//  QUÉ SE PUEDE HACER SIN RASPBERRY PI Y QUÉ NO
//  ----------------------------------------------------------------------
//  El ESP32-S3 por sí solo NO tiene cámara: no puede reconocer la forma ni
//  el color de un objeto lejano, solo leer sensores de contacto/proximidad
//  y el color del piso justo debajo de sí. Con eso alcanza para:
//    ✔ Identificar las líneas/zonas de piso negro, amarillo, rojo y azul
//      (TCS34725, igual que la versión con Raspberry Pi).
//    ✔ Cargar y depositar la llave: se asume que un operador la coloca a
//      mano en la pinza abierta antes de encender el robot (igual que
//      asume raspberry-pi/src/athena/decision.py en su fase INICIO) — el
//      firmware la sujeta, la transporta a la zona amarilla y la suelta.
//    ✔ Trasladar la bandera una vez agarrada: cerrar la pinza, volver a la
//      zona propia (reconocida por el color de piso) y soltarla ahí.
//    ✘ Encontrar Y VERIFICAR que lo que agarra es específicamente la
//      bandera DEL EQUIPO CONTRARIO. Sin cámara no hay forma de distinguir
//      "bandera roja" de "bandera azul" ni de "un objeto cualquiera" a
//      distancia. Este firmware usa el VL53L1X (telémetro delante del
//      gripper) como sustituto: durante la búsqueda, barre el área y
//      cierra la pinza sobre lo primero sólido que encuentra a corta
//      distancia. Es una heurística, no una detección real — documentado
//      a propósito, no es un descuido. Para "detectar la bandera del
//      oponente" de verdad hace falta la cámara + Edge Impulse de
//      raspberry-pi/src/athena/ei_flag_detector.py (ver firmware-esp32/).
//
//  LED RGB: INDICADOR PURO DE LA LÍNEA/ZONA DE PISO, nada más
//  ----------------------------------------------------------------------
//    · Piso gris (zona neutra) o sin lectura válida -> APAGADO.
//    · Amarillo / rojo / azul -> ese mismo color, fijo, mientras el sensor
//      delantero esté sobre esa zona.
//    · Negro (borde) -> destello alternando rojo/azul, como alerta.
//  El LED YA NO indica equipo ni "veo la bandera": esta variante lo dedica
//  por completo a decir en qué línea está parado el robot.
//
//  TEAM_SELECT: sin Raspberry Pi que le diga "--equipo rojo/azul" por
//  línea de comandos, el equipo se elige con un puente físico a GND en el
//  GPIO Pins::TEAM_SELECT (ver más abajo) — leído UNA vez al arrancar.
//
//  HARDWARE (idéntico a firmware-esp32/, más el puente TEAM_SELECT):
//    · 2x L298N            -> 4 motores (cada driver mueve 2)
//    · 1x PCA9685 (I2C)    -> 2 servos del gripper (pinza + elevación)
//    · 2x TCS34725 (I2C)   -> sensor de color delantero y trasero
//    · 1x VL53L1X (I2C)    -> telémetro delante del gripper
//    · 2x QTRX-HD-01A      -> reflectancia delantera izquierda y derecha
//    · LED RGB (1x)        -> indicador puro de la línea/zona de piso
//    · Puente TEAM_SELECT  -> GND = ROJO, sin conectar (pull-up) = AZUL
//
//  ÍNDICE
//    [1] Configuración: pines, prioridades, stacks, periodos
//    [2] Tipos compartidos entre tareas
//    [3] Colas
//    [4] Watchdog cooperativo (heartbeats)
//    [5] Driver PCA9685 (servos por I2C)
//    [6] Driver TCS34725 (sensores de color por I2C)
//    [7] Clasificación de color -> etiqueta de zona
//    [8] Tareas de hardware (motores, gripper, color, reflectancia, ToF, LED)
//    [9] MissionTask — el cerebro: percibir -> decidir -> mover, sin RPi
//    [10] setup() / loop()
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// ===========================================================================
//  [1] CONFIGURACIÓN
// ===========================================================================
//
//  PINES PROHIBIDOS en el ESP32-S3 DevKitC-1 — respetarlos no es opcional:
//    GPIO 19, 20    -> USB nativo (D-/D+).
//    GPIO 43, 44    -> U0TXD/U0RXD, puerto de programación y depuración.
//    GPIO 26..32    -> flash SPI interna. Tocarlos cuelga el chip.
//    GPIO 33..37    -> PSRAM Octal (módulos N8R8/N16R8). Igual de intocables.
//    GPIO 0, 45, 46 -> pines de strapping: su nivel al arranque decide el modo
//                      de boot. Evitarlos.
//    GPIO 3         -> strapping de JTAG. Se puede usar, pero mejor no.
//
//  Además: los sensores analógicos DEBEN ir en ADC1 (GPIO 1..10). El ADC2
//  queda inutilizable en cuanto se enciende el WiFi.

namespace Pins {
    // -------- Motores: 2x L298N, cada uno mueve 2 motores ------------------
    constexpr uint8_t L298N_L_IN1 = 4;    // FL sentido A
    constexpr uint8_t L298N_L_IN2 = 5;    // FL sentido B
    constexpr uint8_t L298N_L_ENA = 6;    // FL velocidad (PWM)
    constexpr uint8_t L298N_L_IN3 = 7;    // RL sentido A
    constexpr uint8_t L298N_L_IN4 = 15;   // RL sentido B
    constexpr uint8_t L298N_L_ENB = 16;   // RL velocidad (PWM)

    constexpr uint8_t L298N_R_IN1 = 10;   // FR sentido A
    constexpr uint8_t L298N_R_IN2 = 11;   // FR sentido B
    constexpr uint8_t L298N_R_ENA = 12;   // FR velocidad (PWM)
    constexpr uint8_t L298N_R_IN3 = 13;   // RR sentido A
    constexpr uint8_t L298N_R_IN4 = 14;   // RR sentido B
    constexpr uint8_t L298N_R_ENB = 17;   // RR velocidad (PWM)

    // -------- Bus I2C nº0: PCA9685 (servos) + TCS34725 DELANTERO -----------
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;

    // -------- Bus I2C nº1: TCS34725 TRASERO --------------------------------
    // Los dos TCS34725 comparten la misma dirección fija (0x29): van en buses
    // separados para no necesitar un multiplexor TCA9548A.
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    // -------- LED de iluminación de cada TCS34725 ---------------------------
    constexpr uint8_t TCS_LED_FRONT = 18;
    constexpr uint8_t TCS_LED_BACK  = 21;

    // -------- VL53L1X (ToF), delante del gripper ---------------------------
    // Comparte el bus I2C nº0 con el TCS34725 delantero (mismo 0x29 de
    // fábrica): XSHUT lo mantiene en reset hasta reasignarle dirección.
    constexpr uint8_t TOF_XSHUT = 3;

    // -------- Reflectancia QTRX-HD-01A (salida analógica) ------------------
    // ¡ALIMENTARLOS A 3.3 V! Ver la nota larga en ReflectanceTask.
    constexpr uint8_t QTR_LEFT_OUT  = 1;  // GPIO1 = ADC1_CH0
    constexpr uint8_t QTR_RIGHT_OUT = 2;  // GPIO2 = ADC1_CH1
    constexpr uint8_t QTR_EMITTER_CTRL = 42;

    // -------- LED RGB indicador de equipo / zona / bandera ------------------
    constexpr uint8_t LED_RGB_R = 39;
    constexpr uint8_t LED_RGB_G = 38;
    constexpr uint8_t LED_RGB_B = 41;

    // -------- Selector de equipo, SOLO en esta variante autónoma -----------
    // GPIO40 quedó libre en el robot (ver historial de firmware-esp32/: el
    // XSHUT del VL53L1X se corrió de aquí al 3, y el canal azul del LED al
    // 41). INPUT_PULLUP: puente a GND = ROJO, sin puente = ROJO también NO
    // — hace falta un nivel por defecto explícito, y se eligió que "sin
    // puente" (HIGH, el pull-up interno) sea AZUL. Leído una sola vez en
    // setup(): cambiarlo exige reiniciar el ESP32, que es exactamente lo que
    // ya se hace entre rondas para reflashear o solo repotenciar.
    constexpr uint8_t TEAM_SELECT = 40;
}

namespace I2CAddr {
    constexpr uint8_t PCA9685  = 0x40;
    constexpr uint8_t TCS34725 = 0x29;
    // Ver la nota larga en firmware-esp32/src/main.cpp junto a esta misma
    // constante: es una única escritura corta mientras el VL53L1X todavía
    // responde en 0x29, el mismo instante en que el TCS34725 delantero
    // también sigue vivo ahí. Riesgo acotado y aceptado a propósito.
    constexpr uint8_t VL53L1X_BOOT_ADDR = 0x29;
    constexpr uint8_t VL53L1X = 0x30;
}

namespace ServoChannel {
    constexpr uint8_t CLAW = 0;   // abre/cierra la pinza
    constexpr uint8_t LIFT = 1;   // sube/baja el gripper (no montado todavía)
}

namespace Pwm {
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;
    constexpr uint8_t  MOTOR_RESOLUTION = 8;     // duty 0..255

    constexpr uint32_t RGB_FREQ_HZ    = 5000;
    constexpr uint8_t  RGB_RESOLUTION = 8;       // duty 0..255

    constexpr uint32_t SERVO_FREQ_HZ  = 50;
    constexpr uint16_t SERVO_TICK_MIN = 205;   // pulso 1.0 ms ->   0 grados
    constexpr uint16_t SERVO_TICK_MAX = 410;   // pulso 2.0 ms -> 180 grados
}

// Prioridades FreeRTOS (mayor número = mayor prioridad).
namespace TaskPriority {
    constexpr UBaseType_t SUPERVISOR      = 6;
    constexpr UBaseType_t MOTOR_CONTROL   = 5;
    constexpr UBaseType_t MISSION         = 4;  // el cerebro: no debe acumular retraso
    constexpr UBaseType_t GRIPPER_CONTROL = 3;
    constexpr UBaseType_t REFLECTANCE     = 3;
    constexpr UBaseType_t COLOR_SENSOR    = 2;
    constexpr UBaseType_t TOF_SENSOR      = 2;
    constexpr UBaseType_t LED_STATUS      = 1;
}

namespace TaskStack {
    constexpr uint32_t SUPERVISOR      = 3072;
    constexpr uint32_t MISSION         = 4096;  // máquina de estados + logs
    constexpr uint32_t MOTOR_CONTROL   = 3072;
    constexpr uint32_t GRIPPER_CONTROL = 3072;
    constexpr uint32_t REFLECTANCE     = 2560;
    constexpr uint32_t COLOR_SENSOR    = 3584;
    constexpr uint32_t TOF_SENSOR      = 4096;
    constexpr uint32_t LED_STATUS      = 2048;
}

namespace TaskPeriodMs {
    constexpr uint32_t SUPERVISOR    = 200;
    constexpr uint32_t MISSION       = 50;   // 20 Hz, de sobra para decidir
    constexpr uint32_t MOTOR_CONTROL = 20;   // 50 Hz
    constexpr uint32_t GRIPPER       = 50;
    constexpr uint32_t REFLECTANCE   = 20;   // 50 Hz, seguimiento de línea
    constexpr uint32_t COLOR_SENSOR  = 100;  // 10 Hz
    constexpr uint32_t TOF_SENSOR    = 50;   // 20 Hz, igual al periodo de rango
    constexpr uint32_t LED_STATUS    = 100;  // más rápido que antes: hace falta para el destello de 5 Hz
}

// Sin heartbeat por más de este tiempo, el supervisor da la tarea por colgada.
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;

// Si MotorTask pasa este tiempo sin un comando fresco de MissionTask, frena
// por su cuenta. Ya no protege contra "se cayó el enlace con la Raspberry
// Pi" (no existe): protege contra "MissionTask se colgó" — mismo mecanismo,
// mismo valor, otro origen del riesgo.
constexpr uint32_t MISSION_FAILSAFE_TIMEOUT_MS = 500;

enum class TaskId : uint8_t {
    MOTOR_CONTROL = 0,
    GRIPPER_CONTROL,
    COLOR_SENSOR,
    REFLECTANCE,
    LED_STATUS,
    TOF_SENSOR,
    MISSION,
    COUNT   // siempre el último
};

#define DEBUG_LINK Serial   // USB nativo: sin Raspberry Pi, no hace falta reservarlo aparte

// ===========================================================================
//  [2] TIPOS COMPARTIDOS ENTRE TAREAS
// ===========================================================================

enum class TeamColor     : uint8_t { NONE = 0, RED = 1, BLUE = 2 };
enum class MotorMode     : uint8_t { STOP = 0, DRIVE = 1 };
enum class GripperAction : uint8_t { OPEN = 0, CLOSE_LLAVE = 1, CLOSE_BANDERA = 2, RAISE = 3, LOWER = 4 };
enum class ColorLabel    : uint8_t { UNKNOWN = 0, BLACK, YELLOW, RED, BLUE, FLOOR };

struct MotorCommand {
    MotorMode mode = MotorMode::STOP;
    int8_t    left  = 0;   // -100..100 (%), lado izquierdo (FL+RL)
    int8_t    right = 0;   // -100..100 (%), lado derecho  (FR+RR)
};

struct GripperCommand {
    GripperAction action = GripperAction::OPEN;
};

// El LED aquí tiene un solo trabajo: decir sobre qué línea/zona está el
// robot AHORA MISMO. MissionTask ya tiene la lectura de color a mano (la
// necesita para decidir), así que se la pasa a LedTask en vez de que
// LedTask vuelva a suscribirse a colorQueue por su cuenta — una sola tarea
// consume cada cola.
struct LedCommand {
    ColorLabel zone = ColorLabel::UNKNOWN;
};

struct ColorReading {
    uint32_t   timestamp_ms = 0;
    ColorLabel front        = ColorLabel::UNKNOWN;
    ColorLabel back         = ColorLabel::UNKNOWN;
    bool       front_valid  = false;
    bool       back_valid   = false;
};

struct ReflectanceReading {
    uint32_t timestamp_ms  = 0;
    uint16_t left_raw      = 0;
    uint16_t right_raw     = 0;
    bool     left_on_line  = false;
    bool     right_on_line = false;
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
//   MissionTask --> motorCmdQueue   (1, overwrite) --> MotorTask
//               --> gripperCmdQueue (4, FIFO)      --> GripperTask
//               --> ledCmdQueue     (1, overwrite) --> LedTask
//
//   ColorSensorTask --> colorQueue   (4, FIFO)      --> MissionTask
//   ReflectanceTask --> reflectQueue (4, FIFO)      --> MissionTask
//   TofSensorTask   --> tofQueue     (4, FIFO)      --> MissionTask
//   SupervisorTask  --> healthQueue  (1, overwrite) --> MissionTask (solo log)
//
//  Mismo criterio que en firmware-esp32/: colas "overwrite" para datos donde
//  solo importa el valor más reciente, FIFO para eventos que hay que ver
//  todos y en orden. Ninguna tarea usa portMAX_DELAY: todo timeout 0.

static QueueHandle_t g_motorCmdQueue   = nullptr;
static QueueHandle_t g_gripperCmdQueue = nullptr;
static QueueHandle_t g_ledCmdQueue     = nullptr;
static QueueHandle_t g_colorQueue      = nullptr;
static QueueHandle_t g_reflectQueue    = nullptr;
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
//  [5] DRIVER PCA9685 — servos del gripper por I2C
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
        Wire.beginTransmission(I2CAddr::PCA9685);
        Wire.write(reg);
        Wire.write(value);
        return Wire.endTransmission() == 0;
    }

    bool ReadReg(uint8_t reg, uint8_t &out) {
        Wire.beginTransmission(I2CAddr::PCA9685);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom((int)I2CAddr::PCA9685, 1) != 1) return false;
        out = (uint8_t)Wire.read();
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

        Wire.beginTransmission(I2CAddr::PCA9685);
        Wire.write(REG_LED0_ON_L + 4 * channel);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write((uint8_t)(ticks & 0xFF));
        Wire.write((uint8_t)(ticks >> 8));
        return Wire.endTransmission() == 0;
    }
}

static uint16_t ServoAngleToTicks(int angle_deg) {
    angle_deg = constrain(angle_deg, 0, 180);
    return (uint16_t)(Pwm::SERVO_TICK_MIN +
        ((uint32_t)(Pwm::SERVO_TICK_MAX - Pwm::SERVO_TICK_MIN) * (uint32_t)angle_deg) / 180UL);
}

// ===========================================================================
//  [6] DRIVER TCS34725 — sensores de color por I2C
// ===========================================================================

namespace Tcs34725 {
    constexpr uint8_t CMD_BIT   = 0x80;
    constexpr uint8_t CMD_AUTO_INC = 0x20;

    constexpr uint8_t REG_ENABLE  = 0x00;
    constexpr uint8_t REG_ATIME   = 0x01;
    constexpr uint8_t REG_CONTROL = 0x0F;
    constexpr uint8_t REG_ID      = 0x12;
    constexpr uint8_t REG_CDATAL  = 0x14;

    constexpr uint8_t ENABLE_PON = 0x01;
    constexpr uint8_t ENABLE_AEN = 0x02;

    constexpr uint8_t ATIME_24MS = 0xEB;
    constexpr uint8_t GAIN_4X = 0x01;

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
//  [7] CLASIFICACIÓN DE COLOR -> ETIQUETA DE ZONA
// ===========================================================================
//
//  MISMOS umbrales que firmware-esp32/src/main.cpp, calibrados contra 970
//  muestras reales del TCS34725 delantero (ver calibracion-color/). Si se
//  recalibra, actualizar los DOS firmwares — igual que ya advierte esa nota
//  allá, ahora con un tercer archivo (este) para no olvidar.

static ColorLabel ClassifyColor(const Tcs34725::Rgbc &s) {
    if (s.c < 392) return ColorLabel::BLACK;

    const float total = (float)s.c;
    const float r = (float)s.r / total;
    const float g = (float)s.g / total;
    const float b = (float)s.b / total;

    if (r > 0.450f && g < 0.312f && b < 0.300f) return ColorLabel::RED;
    if (b > 0.216f && r < 0.390f)               return ColorLabel::BLUE;
    if (r > 0.416f && g > 0.350f && b < 0.250f) return ColorLabel::YELLOW;

    return ColorLabel::FLOOR;
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

// Ver la nota larga en firmware-esp32/src/main.cpp: la rueda trasera
// izquierda gira al revés con el cableado físico actual, así que se
// intercambia el orden de IN1/IN2 solo para ese motor.
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

void MotorApply(const Motor &m, int speed) {
    speed = constrain(speed, -100, 100);
    const bool forward = (speed >= 0);
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

        // Failsafe: si MissionTask se cuelga (bug, excepción, deadlock en un
        // I2C), esto frena los motores igual que antes frenaba si se caía el
        // enlace con la Raspberry Pi. El origen del riesgo cambió; el
        // mecanismo de defensa, no.
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
//  8.2  GripperTask — 2 servos vía PCA9685
// ---------------------------------------------------------------------------
//  Ángulos calibrados con pruebas-platformio/06-calibracion-gripper/, los
//  mismos que usa firmware-esp32/. El servo de elevación (LIFT) no está
//  montado todavía: todo lo que lo toca queda comentado, igual que allá.

constexpr int kClawOpenDeg          = 0;
constexpr int kClawClosedLlaveDeg   = 120;
constexpr int kClawClosedBanderaDeg = 65;
// constexpr int kLiftUpDeg            = 160;
// constexpr int kLiftDownDeg          = 20;

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
        }

        GripperCommand cmd;
        if (xQueueReceive(g_gripperCmdQueue, &cmd, 0) == pdTRUE && pca_ok) {
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
                    break;   // RAISE/LOWER: sin servo de elevación montado
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
//  8.3  TofSensorTask — VL53L1X, telémetro delante del gripper
// ---------------------------------------------------------------------------
//  Aquí es donde este firmware hace más trabajo que en la versión con
//  Raspberry Pi: sin cámara, esta es la ÚNICA fuente de "algo delante" que
//  tiene MissionTask para intentar agarrar la bandera. Ver el aviso grande
//  al principio del archivo sobre esta limitación.

namespace Tof {
    constexpr uint32_t BOOT_DELAY_MS = 2;
    constexpr uint16_t TIMING_BUDGET_US = 50000;
    constexpr uint32_t RANGING_PERIOD_MS = 50;
}

VL53L1X g_tof;

bool TofBringUp() {
    g_tof.setBus(&Wire);
    g_tof.setTimeout(500);
    g_tof.setAddress(I2CAddr::VL53L1X);

    if (!g_tof.init()) return false;

    g_tof.setDistanceMode(VL53L1X::Long);
    g_tof.setMeasurementTimingBudget(Tof::TIMING_BUDGET_US);
    g_tof.startContinuous(Tof::RANGING_PERIOD_MS);
    return true;
}

void TofSensorTask(void *) {
    digitalWrite(Pins::TOF_XSHUT, HIGH);
    delay(Tof::BOOT_DELAY_MS);

    bool tof_ok = TofBringUp();
    if (!tof_ok) {
        DEBUG_LINK.println("[ToF] VL53L1X no responde. Reintentando en segundo plano.");
    }

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::TOF_SENSOR);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        if (!tof_ok && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            tof_ok = g_tof.init();
            if (tof_ok) {
                g_tof.setDistanceMode(VL53L1X::Long);
                g_tof.setMeasurementTimingBudget(Tof::TIMING_BUDGET_US);
                g_tof.startContinuous(Tof::RANGING_PERIOD_MS);
            }
        }

        TofReading reading;
        reading.timestamp_ms = millis();

        if (tof_ok && g_tof.dataReady()) {
            reading.distance_mm = g_tof.read(false);
            reading.valid = !g_tof.timeoutOccurred() &&
                            g_tof.ranging_data.range_status == VL53L1X::RangeValid;
            if (g_tof.timeoutOccurred()) tof_ok = false;
        }

        PushDropOldest(g_tofQueue, reading);

        Heartbeat(TaskId::TOF_SENSOR);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  8.4  ColorSensorTask — TCS34725 delantero (Wire) y trasero (Wire1)
// ---------------------------------------------------------------------------

void ColorSensorTask(void *) {
    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    pinMode(Pins::TCS_LED_BACK, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);
    digitalWrite(Pins::TCS_LED_BACK, HIGH);

    bool front_ok = Tcs34725::Init(Wire);
    bool back_ok  = Tcs34725::Init(Wire1);

    if (!front_ok) DEBUG_LINK.println("[Color] sensor DELANTERO no responde (bus I2C 0).");
    if (!back_ok)  DEBUG_LINK.println("[Color] sensor TRASERO no responde (bus I2C 1).");

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

        Tcs34725::Rgbc sample;
        if (front_ok && Tcs34725::Read(Wire, sample)) {
            reading.front = ClassifyColor(sample);
            reading.front_valid = true;
        } else {
            front_ok = false;
        }

        if (back_ok && Tcs34725::Read(Wire1, sample)) {
            reading.back = ClassifyColor(sample);
            reading.back_valid = true;
        } else {
            back_ok = false;
        }

        PushDropOldest(g_colorQueue, reading);

        Heartbeat(TaskId::COLOR_SENSOR);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  8.5  ReflectanceTask — 2x QTRX-HD-01A
// ---------------------------------------------------------------------------
//  Valor ADC ALTO = superficie OSCURA (cinta negra del borde). Ver la nota
//  larga en firmware-esp32/src/main.cpp si esto no es intuitivo.

constexpr uint16_t kDarkThreshold = 2048;   // TODO: calibrar en la pista real

void ReflectanceTask(void *) {
    analogReadResolution(12);
    analogSetPinAttenuation(Pins::QTR_LEFT_OUT, ADC_11db);
    analogSetPinAttenuation(Pins::QTR_RIGHT_OUT, ADC_11db);

    pinMode(Pins::QTR_EMITTER_CTRL, OUTPUT);
    digitalWrite(Pins::QTR_EMITTER_CTRL, HIGH);

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::REFLECTANCE);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        ReflectanceReading reading;
        reading.timestamp_ms  = millis();
        reading.left_raw      = (uint16_t)analogRead(Pins::QTR_LEFT_OUT);
        reading.right_raw     = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);
        reading.left_on_line  = reading.left_raw  > kDarkThreshold;
        reading.right_on_line = reading.right_raw > kDarkThreshold;

        PushDropOldest(g_reflectQueue, reading);

        Heartbeat(TaskId::REFLECTANCE);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  8.6  LedTask — LED RGB: indicador puro de la línea/zona de piso
// ---------------------------------------------------------------------------
//  Único trabajo del LED en esta variante: decir sobre qué está parado el
//  robot AHORA MISMO, nada más — ni equipo, ni "veo la bandera".
//    · Piso gris (FLOOR) o sin lectura válida -> apagado.
//    · Amarillo / rojo / azul                 -> ese mismo color, fijo.
//    · Negro (borde)                          -> destello alternando
//                                                 rojo/azul, como alerta.

namespace RgbLed {
    constexpr uint8_t CH_R = 4;
    constexpr uint8_t CH_G = 5;
    constexpr uint8_t CH_B = 6;

    // POLARIDAD SIN CONFIRMAR con el LED físico: se asume cátodo común. Si
    // los colores salen invertidos, cambiar a true (ver firmware-esp32/).
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

    // Color fijo de las zonas que NO parpadean. BLACK no aparece aquí a
    // propósito: LedTask lo maneja aparte, alternando rojo/azul.
    void ApplyZone(ColorLabel zone) {
        switch (zone) {
            case ColorLabel::YELLOW: SetRaw(255, 170, 0); break;
            case ColorLabel::RED:    SetRaw(255, 0, 0);   break;
            case ColorLabel::BLUE:   SetRaw(0, 0, 255);   break;
            case ColorLabel::BLACK:
            case ColorLabel::FLOOR:
            case ColorLabel::UNKNOWN:
            default:
                SetRaw(0, 0, 0);   // apagado: piso neutro o sin lectura
                break;
        }
    }
}

// Cuántas vueltas de LedTask dura cada mitad del destello rojo/azul sobre
// el borde negro. A TaskPeriodMs::LED_STATUS (100 ms) y 3 vueltas por
// mitad, el ciclo completo dura 600 ms (~1.7 Hz): lo bastante lento para
// distinguir los dos colores a simple vista, no un borrón.
constexpr uint32_t kBorderBlinkHalfPeriodTicks = 3;

void LedTask(void *) {
    RgbLed::Setup();
    RgbLed::SetRaw(0, 0, 0);

    ColorLabel zone = ColorLabel::UNKNOWN;
    uint32_t   blink_tick = 0;

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::LED_STATUS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        LedCommand cmd;
        if (xQueueReceive(g_ledCmdQueue, &cmd, 0) == pdTRUE) {
            zone = cmd.zone;
        }

        if (zone == ColorLabel::BLACK) {
            blink_tick = (blink_tick + 1) % (2 * kBorderBlinkHalfPeriodTicks);
            if (blink_tick < kBorderBlinkHalfPeriodTicks) {
                RgbLed::SetRaw(255, 0, 0);
            } else {
                RgbLed::SetRaw(0, 0, 255);
            }
        } else {
            blink_tick = 0;
            RgbLed::ApplyZone(zone);
        }

        Heartbeat(TaskId::LED_STATUS);
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace anónimo (tareas de hardware)

// ===========================================================================
//  [9] MISSIONTASK — el cerebro, sin Raspberry Pi
// ===========================================================================
//
//  Puerto directo de la lógica de raspberry-pi/src/athena/decision.py a
//  C++, adaptado a que aquí NO hay percepción visual de la bandera: donde
//  decision.py usaba ``perception.best(bandera_objetivo)`` (una detección
//  de la cámara con clase, ángulo y distancia), esta versión usa el
//  VL53L1X como sustituto ciego — ver el aviso grande al principio del
//  archivo. Las prioridades y el orden de fases son EXACTAMENTE los mismos
//  que allá, por la misma razón: salirse de la pista y adelantar la
//  secuencia pierden la ronda de inmediato, así que esas dos reglas no
//  pueden depender de que nada más ande bien.

namespace Mission {

// Velocidades y umbrales — mismo rol que ControlConfig en decision.py,
// pero constexpr porque aquí no hay archivo de configuración que cargar.
constexpr int kVelocidadCrucero     = 45;   // % de PWM al avanzar recto
constexpr int kVelocidadBusqueda    = 35;   // % al girar buscando
constexpr int kVelocidadAproximacion = 30;  // % al acercarse / evadir

constexpr uint16_t kDistanciaAgarreMm = 150;  // debajo de esto: cerrar la pinza
constexpr uint8_t  kTofDebounceHits   = 3;    // lecturas seguidas antes de agarrar

constexpr uint32_t kStartupDelayMs   = 3000;  // tiempo para cargar la llave y ubicar el robot
constexpr uint32_t kGripperSettleMs  = 400;   // tiempo mecánico para que el servo llegue
constexpr uint32_t kSpinBurstMs      = 900;   // barrido: cuánto gira sobre su eje
constexpr uint32_t kForwardBurstMs   = 700;   // barrido: cuánto avanza entre giros
constexpr uint32_t kReturnTurnMs     = 1500;  // giro aproximado de 180°, A CALIBRAR EN BANCO

enum class Phase : uint8_t {
    ARRANQUE = 0,
    ASEGURAR_LLAVE,
    BUSCAR_ZONA_NEUTRA,
    DEPOSITAR_LLAVE,
    BUSCAR_BANDERA,
    AGARRAR_BANDERA,
    RETORNAR_GIRANDO,
    RETORNAR_AVANZANDO,
    ENTREGAR,
    TERMINADO,
};

inline ColorLabel ZonaPropia(TeamColor team) {
    return (team == TeamColor::RED) ? ColorLabel::RED : ColorLabel::BLUE;
}

} // namespace Mission

// Evita depender de que el compilador trate a MotorCommand como agregado
// con inicializadores por defecto (necesita C++14+); igual que hace
// firmware-esp32/src/main.cpp, se asigna campo a campo.
inline void SetDrive(MotorCommand &m, int left, int right) {
    m.mode  = MotorMode::DRIVE;
    m.left  = (int8_t)constrain(left, -100, 100);
    m.right = (int8_t)constrain(right, -100, 100);
}

void MissionTask(void *pvTeam) {
    const TeamColor team = *reinterpret_cast<TeamColor *>(pvTeam);

    Mission::Phase phase = Mission::Phase::ARRANQUE;
    Mission::Phase last_logged_phase = phase;
    uint32_t phase_started_ms = millis();

    // Estado propio del barrido de búsqueda (fase BUSCAR_BANDERA).
    bool     buscando_gira = true;
    int8_t   sentido = 1;
    uint8_t  tof_hits = 0;

    ColorReading       last_color{};
    ReflectanceReading last_reflect{};
    TofReading         last_tof{};

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::MISSION);
    TickType_t last_wake = xTaskGetTickCount();

    DEBUG_LINK.printf("[Mission] equipo = %s\n", team == TeamColor::RED ? "ROJO" : "AZUL");

    for (;;) {
        // --- 0. Refrescar la última lectura de cada sensor -----------------
        // Igual que hacía run_rover.py: se drena toda la cola y solo importa
        // el dato más reciente de cada tipo.
        ColorReading c;
        while (xQueueReceive(g_colorQueue, &c, 0) == pdTRUE) last_color = c;
        ReflectanceReading r;
        while (xQueueReceive(g_reflectQueue, &r, 0) == pdTRUE) last_reflect = r;
        TofReading t;
        while (xQueueReceive(g_tofQueue, &t, 0) == pdTRUE) last_tof = t;
        HealthReport h;
        while (xQueueReceive(g_healthQueue, &h, 0) == pdTRUE) {
            DEBUG_LINK.printf("[Mission] tareas colgadas, bitmask=0x%02X\n", h.faulted_tasks_bitmask);
        }

        MotorCommand motor;               // por defecto: STOP
        GripperCommand gripper;
        bool send_gripper = false;

        // --- 1. PRIORIDAD MÁXIMA: no salirse de la pista -------------------
        // Idéntico a decision.py::_evadir_borde: pisa cualquier otra fase.
        bool evadiendo = false;
        if (last_reflect.left_on_line && last_reflect.right_on_line) {
            SetDrive(motor, -Mission::kVelocidadAproximacion, -Mission::kVelocidadAproximacion);
            evadiendo = true;
        } else if (last_reflect.left_on_line) {
            SetDrive(motor, -Mission::kVelocidadAproximacion, -Mission::kVelocidadAproximacion / 3);
            evadiendo = true;
        } else if (last_reflect.right_on_line) {
            SetDrive(motor, -Mission::kVelocidadAproximacion / 3, -Mission::kVelocidadAproximacion);
            evadiendo = true;
        }

        // Ayuda a depurar en banco: cambiar de fase se ve en el monitor
        // serial sin tener que instrumentar cada rama.
        if (phase != last_logged_phase) {
            DEBUG_LINK.printf("[Mission] fase -> %d\n", (int)phase);
            last_logged_phase = phase;
        }

        if (!evadiendo) {
            switch (phase) {

                // -- 2. Cuenta regresiva para cargar la llave a mano --------
                case Mission::Phase::ARRANQUE: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kStartupDelayMs) {
                        phase = Mission::Phase::ASEGURAR_LLAVE;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // -- 3. Asegurar la llave que el operador ya colocó ---------
                case Mission::Phase::ASEGURAR_LLAVE: {
                    gripper.action = GripperAction::CLOSE_LLAVE;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleMs) {
                        phase = Mission::Phase::BUSCAR_ZONA_NEUTRA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // -- 4. Avanzar hasta pisar la zona amarilla ----------------
                case Mission::Phase::BUSCAR_ZONA_NEUTRA: {
                    if (last_color.front_valid && last_color.front == ColorLabel::YELLOW) {
                        phase = Mission::Phase::DEPOSITAR_LLAVE;
                        phase_started_ms = millis();
                    } else {
                        SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    }
                    break;
                }

                // -- 5. Soltar la llave: única transición que habilita ------
                //       la búsqueda de bandera (igual que decision.py).
                case Mission::Phase::DEPOSITAR_LLAVE: {
                    gripper.action = GripperAction::OPEN;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleMs) {
                        phase = Mission::Phase::BUSCAR_BANDERA;
                        phase_started_ms = millis();
                        buscando_gira = true;
                        tof_hits = 0;
                    }
                    break;
                }

                // -- 6. Barrer el área; el VL53L1X hace de "ojos" ----------
                // LIMITACIÓN A PROPÓSITO (ver el aviso grande arriba): no
                // hay forma de confirmar que lo que se detecta es la
                // bandera del equipo contrario y no cualquier otro objeto
                // u obstáculo. Es la mejor aproximación posible sin cámara.
                case Mission::Phase::BUSCAR_BANDERA: {
                    if (last_tof.valid && last_tof.distance_mm <= Mission::kDistanciaAgarreMm) {
                        if (tof_hits < 255) tof_hits++;
                    } else {
                        tof_hits = 0;
                    }

                    if (tof_hits >= Mission::kTofDebounceHits) {
                        phase = Mission::Phase::AGARRAR_BANDERA;
                        phase_started_ms = millis();
                        break;
                    }

                    const uint32_t elapsed = millis() - phase_started_ms;
                    if (buscando_gira) {
                        SetDrive(motor, Mission::kVelocidadBusqueda * sentido,
                                        -Mission::kVelocidadBusqueda * sentido);
                        if (elapsed > Mission::kSpinBurstMs) {
                            buscando_gira = false;
                            phase_started_ms = millis();
                        }
                    } else {
                        // Avance con un leve arco, para barrer el ToF a
                        // ambos lados en vez de solo mirar de frente.
                        const int base = Mission::kVelocidadAproximacion;
                        const int sesgo = 10 * sentido;
                        SetDrive(motor, base + sesgo, base - sesgo);
                        if (elapsed > Mission::kForwardBurstMs) {
                            buscando_gira = true;
                            sentido = (int8_t)-sentido;   // alterna el lado del barrido
                            phase_started_ms = millis();
                        }
                    }
                    break;
                }

                // -- 7. Cerrar la pinza sobre lo que haya delante -----------
                case Mission::Phase::AGARRAR_BANDERA: {
                    gripper.action = GripperAction::CLOSE_BANDERA;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleMs) {
                        phase = Mission::Phase::RETORNAR_GIRANDO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // -- 8a. Girar ~180° antes de buscar la zona propia ---------
                // Sin odometría ni referencia visual (mismo hueco que anota
                // decision.py::_retornar): es un giro a tiempo fijo, A
                // CALIBRAR EN BANCO según el peso real del robot y la
                // fricción de la pista.
                case Mission::Phase::RETORNAR_GIRANDO: {
                    SetDrive(motor, Mission::kVelocidadBusqueda, -Mission::kVelocidadBusqueda);
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kReturnTurnMs) {
                        phase = Mission::Phase::RETORNAR_AVANZANDO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // -- 8b. Avanzar hasta pisar la zona del propio equipo ------
                case Mission::Phase::RETORNAR_AVANZANDO: {
                    if (last_color.front_valid && last_color.front == Mission::ZonaPropia(team)) {
                        phase = Mission::Phase::ENTREGAR;
                        phase_started_ms = millis();
                    } else {
                        SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    }
                    break;
                }

                // -- 9. Soltar la bandera en zona propia ---------------------
                case Mission::Phase::ENTREGAR: {
                    gripper.action = GripperAction::OPEN;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleMs) {
                        phase = Mission::Phase::TERMINADO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // -- 10. Misión completa: quieto -----------------------------
                case Mission::Phase::TERMINADO:
                default:
                    break;
            }
        }

        xQueueOverwrite(g_motorCmdQueue, &motor);
        if (send_gripper) xQueueSend(g_gripperCmdQueue, &gripper, 0);

        LedCommand led;
        led.zone = last_color.front_valid ? last_color.front : ColorLabel::UNKNOWN;
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
    g_reflectQueue    = xQueueCreate(4, sizeof(ReflectanceReading));
    g_tofQueue        = xQueueCreate(4, sizeof(TofReading));
    g_healthQueue     = xQueueCreate(1, sizeof(HealthReport));

    return g_motorCmdQueue && g_gripperCmdQueue && g_ledCmdQueue &&
           g_colorQueue && g_reflectQueue && g_tofQueue && g_healthQueue;
}

constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// Vive fuera de setup() porque MissionTask la recibe por puntero al crear
// la tarea (xTaskCreatePinnedToCore no admite pasar un TeamColor por valor
// directamente): tiene que sobrevivir a setup() retornando.
static TeamColor g_myTeam = TeamColor::BLUE;   // sobrescrito abajo por TEAM_SELECT

void setup() {
    DEBUG_LINK.begin(SERIAL_BAUD_RATE);
    delay(200);
    DEBUG_LINK.println("\nAthena Rover 2026 - firmware ESP32-S3 AUTONOMO (sin Raspberry Pi)");

    // -- Selector de equipo: puente físico a GND en Pins::TEAM_SELECT -------
    // Se lee UNA sola vez aquí, antes de crear ninguna tarea: MissionTask
    // recibe el resultado por puntero y no vuelve a tocar este pin.
    pinMode(Pins::TEAM_SELECT, INPUT_PULLUP);
    delay(5);   // deja asentar la lectura tras habilitar el pull-up
    g_myTeam = (digitalRead(Pins::TEAM_SELECT) == LOW) ? TeamColor::RED : TeamColor::BLUE;
    DEBUG_LINK.printf("[Setup] TEAM_SELECT=%d -> equipo %s\n",
                       digitalRead(Pins::TEAM_SELECT),
                       g_myTeam == TeamColor::RED ? "ROJO" : "AZUL");

    // Los dos buses I2C se abren aquí, ANTES de lanzar las tareas, igual que
    // en firmware-esp32/: así ninguna tarea tiene que inicializar hardware
    // compartido por su cuenta.
    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL, 400000);   // PCA9685 + TCS34725 delantero + VL53L1X
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL, 400000);  // TCS34725 trasero
    Wire.setTimeOut(25);
    Wire1.setTimeOut(25);

    // VL53L1X en reset desde ya — ver la nota larga en TofSensorTask (y su
    // gemela, más detallada, en firmware-esp32/src/main.cpp) sobre por qué
    // esto tiene que pasar ANTES de crear ninguna tarea.
    pinMode(Pins::TOF_XSHUT, OUTPUT);
    digitalWrite(Pins::TOF_XSHUT, LOW);

    if (!CreateQueues()) {
        DEBUG_LINK.println("[FATAL] no se pudieron crear las colas. Arranque detenido.");
        for (;;) delay(1000);
    }

    // Heartbeats inicializados antes de crear ninguna tarea, para que el
    // supervisor no vea como "recién nacida" a una tarea que ya arrancó.
    WatchdogInit();

    xTaskCreatePinnedToCore(SupervisorTask, "Supervisor", TaskStack::SUPERVISOR,
                            nullptr, TaskPriority::SUPERVISOR, nullptr, 0);
    xTaskCreatePinnedToCore(MissionTask, "Mission", TaskStack::MISSION,
                            &g_myTeam, TaskPriority::MISSION, nullptr, 1);
    xTaskCreatePinnedToCore(MotorTask, "Motors", TaskStack::MOTOR_CONTROL,
                            nullptr, TaskPriority::MOTOR_CONTROL, nullptr, 1);
    xTaskCreatePinnedToCore(GripperTask, "Gripper", TaskStack::GRIPPER_CONTROL,
                            nullptr, TaskPriority::GRIPPER_CONTROL, nullptr, 1);
    xTaskCreatePinnedToCore(ColorSensorTask, "ColorSensors", TaskStack::COLOR_SENSOR,
                            nullptr, TaskPriority::COLOR_SENSOR, nullptr, 0);
    xTaskCreatePinnedToCore(TofSensorTask, "TofSensor", TaskStack::TOF_SENSOR,
                            nullptr, TaskPriority::TOF_SENSOR, nullptr, 0);
    xTaskCreatePinnedToCore(ReflectanceTask, "Reflectance", TaskStack::REFLECTANCE,
                            nullptr, TaskPriority::REFLECTANCE, nullptr, 0);
    xTaskCreatePinnedToCore(LedTask, "TeamLed", TaskStack::LED_STATUS,
                            nullptr, TaskPriority::LED_STATUS, nullptr, 0);

    DEBUG_LINK.println("Todas las tareas lanzadas. Misión autónoma en marcha.");
}

void loop() {
    // Todo el trabajo ocurre en las tareas. loop() se queda vacío a propósito.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
