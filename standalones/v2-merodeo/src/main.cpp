// ===========================================================================
//  Athena Rover 2026 — Firmware ESP32-S3 AUTÓNOMO (sin Raspberry Pi)
//  Retos del Rover H07 · INTEC · Reymildo & Montse
//
//  VARIANTE v2-merodeo: parte de standalones/v1-confirmado/ (ver ese
//  archivo para la versión base ya confirmada) y le cambia el manejo del
//  borde negro: en vez de retroceder al instante mientras lo siga
//  detectando, ejecuta una maniobra fija de retroceso (kEvasionRetrocesoMs)
//  + giro de ~180° (kEvasionGiroMs) y retoma la búsqueda de la zona
//  amarilla -- "merodea" en vez de solo alejarse del borde.
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
//      (TCS34725 trasero, igual que la versión con Raspberry Pi).
//    ✔ Cargar y depositar la llave: se asume que un operador la coloca a
//      mano en la pinza abierta antes de encender el robot (igual que
//      asume raspberry-pi/src/athena/decision.py en su fase INICIO) — el
//      firmware la sujeta, la transporta a la zona amarilla y la suelta.
//    ✘ Buscar y agarrar la bandera del equipo contrario. Sin cámara ni
//      telémetro (el VL53L1X y el TCS34725 delantero, ambos en el bus I2C
//      nº0, se retiraron de esta variante — daban problemas físicos
//      persistentes en banco) no queda ningún sensor capaz de detectar
//      "hay algo delante": esta demo se limita a depositar la llave y
//      termina ahí. Para la misión completa (agarrar y devolver la
//      bandera) hace falta la cámara + Edge Impulse de
//      raspberry-pi/src/athena/ei_flag_detector.py (ver firmware-esp32/).
//
//  LED RGB: INDICADOR PURO DE LA LÍNEA/ZONA DE PISO, nada más
//  ----------------------------------------------------------------------
//    · Piso gris (zona neutra) o sin lectura válida -> APAGADO.
//    · Amarillo / rojo / azul -> ese mismo color, fijo, mientras el sensor
//      trasero esté sobre esa zona.
//    · Negro (borde) -> destello alternando rojo/azul, como alerta.
//  El LED YA NO indica equipo ni "veo la bandera": esta variante lo dedica
//  por completo a decir en qué línea está parado el robot.
//
//  SWITCH DE EQUIPO: sin Raspberry Pi que le diga "--equipo rojo/azul" por
//  línea de comandos, el equipo se elige con un switch físico de 3
//  posiciones (ON-OFF-ON) en Pins::TEAM_SWITCH_BLUE/RED (ver más abajo).
//  A diferencia del viejo puente de un solo pin, setup() se QUEDA ESPERANDO
//  en la posición central (0, nadie ha elegido) antes de arrancar
//  MissionTask — el procedimiento normal es encender el robot con el switch
//  en 0 y recién ahí elegir equipo, sin que el robot se mueva mientras tanto.
//
//  HARDWARE (idéntico a firmware-esp32/, más el switch de equipo, MENOS el
//  bus I2C nº0 -- ver más abajo por qué se retiró por completo):
//    · 2x L298N            -> 4 motores (cada driver mueve 2)
//    · 1x PCA9685 (I2C)    -> 1 servo del gripper (la pinza que abre/cierra)
//    · 1x TCS34725 (I2C)   -> sensor de color trasero (único que queda)
//    · 2x QTRX-HD-01A      -> reflectancia delantera izquierda y derecha
//    · LED RGB (1x)        -> indicador puro de la línea/zona de piso
//    · Switch 3 posiciones -> 0=nadie ha elegido, 1=azul, 2=rojo
//
//  BUS I2C Nº0 RETIRADO POR COMPLETO (TCS34725 delantero + VL53L1X)
//  ----------------------------------------------------------------------
//  El sensor de color delantero nunca dio una conexión física confiable en
//  banco (SDA/SCL intermitentes, ver el historial en calibracion/color/), y
//  el VL53L1X dependía de compartir bus con él (mismo 0x29 de fábrica,
//  coreografía de XSHUT). Decisión del equipo: en vez de seguir
//  depurando un bus que solo daba problemas, se elimina TODO lo asociado a
//  él -- pines, mutex, tarea del ToF, lado delantero de ColorSensorTask --
//  y la misión se apoya solo en el TCS34725 trasero (bus nº1) y en tiempos
//  fijos donde antes hacía falta el ToF. Ver el índice: ya no hay sección
//  de ToF ni de sensor delantero.
//
//  ÍNDICE
//    [1] Configuración: pines, prioridades, stacks, periodos
//    [2] Tipos compartidos entre tareas
//    [3] Colas
//    [4] Watchdog cooperativo (heartbeats)
//    [5] Driver PCA9685 (servos por I2C)
//    [6] Driver TCS34725 (sensor de color trasero, por I2C)
//    [7] Clasificación de color -> etiqueta de zona
//    [8] Tareas de hardware (motores, gripper, color, reflectancia, LED)
//    [9] MissionTask — el cerebro: percibir -> decidir -> mover, sin RPi
//    [10] setup() / loop()
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <freertos/semphr.h>   // SemaphoreHandle_t / mutex del bus I2C (ver g_i2c1Mutex)
#include <esp_system.h>        // esp_reset_reason() — ver el log de motivo de reinicio en setup()

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

    // -------- Bus I2C nº1: TCS34725 TRASERO + PCA9685 (servos) --------------
    // El bus I2C nº0 (TCS34725 delantero + VL53L1X) se retiró por completo
    // -- ver la nota grande al principio del archivo. Este es ahora el
    // ÚNICO bus I2C del firmware: el TCS34725 trasero y el PCA9685
    // conviven aquí, protegidos por g_i2c1Mutex por el riesgo de
    // concurrencia entre tareas de distinto núcleo (GripperTask en el
    // núcleo 1, ColorSensorTask en el núcleo 0).
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    // -------- Reflectancia QTRX-HD-01A (salida analógica) ------------------
    // ¡ALIMENTARLOS A 3.3 V! Ver la nota larga en ReflectanceTask.
    constexpr uint8_t QTR_LEFT_OUT  = 1;  // GPIO1 = ADC1_CH0
    constexpr uint8_t QTR_RIGHT_OUT = 2;  // GPIO2 = ADC1_CH1
    constexpr uint8_t QTR_EMITTER_CTRL = 42;

    // -------- LED RGB indicador de equipo / zona / bandera ------------------
    constexpr uint8_t LED_RGB_R = 39;
    constexpr uint8_t LED_RGB_G = 38;
    constexpr uint8_t LED_RGB_B = 41;

    // -------- Switch de 3 posiciones (ON-OFF-ON): selección de equipo ------
    // Reemplaza al viejo puente TEAM_SELECT de un solo pin (GND=ROJO,
    // abierto=AZUL, sin reposo real): ahora hay una posición central de
    // verdad (0 = nadie ha elegido todavía), igual que en firmware-esp32/.
    // El común del switch va a GND; cada tiro cierra a GND uno de estos 2
    // GPIO, leídos con pull-up interno (INPUT_PULLUP): HIGH = tiro abierto,
    // LOW = tiro cerrado. GPIO21 se liberó del LED trasero del TCS34725 (ver
    // arriba); GPIO40 es el mismo pin que ya usaba TEAM_SELECT.
    // Mismo cableado físico que firmware-esp32/ -- ver el comentario ahí:
    // el tiro "AZUL" quedó en GPIO 40 y el "ROJO" en GPIO 21, al revés del
    // borrador original de hardware/conexiones-esp32-s3.md.
    constexpr uint8_t TEAM_SWITCH_BLUE = 40;  // tiro "AZUL" cerrado a GND = equipo azul
    constexpr uint8_t TEAM_SWITCH_RED  = 21;  // tiro "ROJO" cerrado a GND = equipo rojo
}

namespace I2CAddr {
    constexpr uint8_t PCA9685  = 0x40;
    constexpr uint8_t TCS34725 = 0x29;
}

namespace ServoChannel {
    constexpr uint8_t CLAW = 0;   // abre/cierra la pinza
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
    constexpr UBaseType_t LED_STATUS      = 1;
}

namespace TaskStack {
    constexpr uint32_t SUPERVISOR      = 3072;
    constexpr uint32_t MISSION         = 4096;  // máquina de estados + logs
    constexpr uint32_t MOTOR_CONTROL   = 3072;
    constexpr uint32_t GRIPPER_CONTROL = 3072;
    constexpr uint32_t REFLECTANCE     = 2560;
    constexpr uint32_t COLOR_SENSOR    = 3584;
    constexpr uint32_t LED_STATUS      = 2048;
}

namespace TaskPeriodMs {
    constexpr uint32_t SUPERVISOR    = 200;
    constexpr uint32_t MISSION       = 50;   // 20 Hz, de sobra para decidir
    constexpr uint32_t MOTOR_CONTROL = 20;   // 50 Hz
    constexpr uint32_t GRIPPER       = 50;
    constexpr uint32_t REFLECTANCE   = 20;   // 50 Hz, seguimiento de línea
    constexpr uint32_t COLOR_SENSOR  = 100;  // 10 Hz
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
    MISSION,
    COUNT   // siempre el último
};

#define DEBUG_LINK Serial   // USB nativo: sin Raspberry Pi, no hace falta reservarlo aparte

// ===========================================================================
//  [2] TIPOS COMPARTIDOS ENTRE TAREAS
// ===========================================================================

enum class TeamColor     : uint8_t { NONE = 0, RED = 1, BLUE = 2 };
enum class MotorMode     : uint8_t { STOP = 0, DRIVE = 1 };
enum class GripperAction : uint8_t { OPEN = 0, CLOSE_LLAVE = 1, CLOSE_BANDERA = 2 };
enum class ColorLabel    : uint8_t { UNKNOWN = 0, BLACK, YELLOW, RED, BLUE, FLOOR };

struct MotorCommand {
    MotorMode mode = MotorMode::STOP;
    int8_t    left  = 0;   // -100..100 (%), lado izquierdo (FL+RL)
    int8_t    right = 0;   // -100..100 (%), lado derecho  (FR+RR)
    // Fuerza la combinación de IN1/IN2 CONTRARIA a la que `left`/`right`
    // producirían normalmente en MotorApply -- ver la nota grande junto a
    // MotorApply sobre por qué esto existe en vez de voltear el signo
    // global de nuevo: RETROCEDER_A_ZONA_NEUTRA es la única fase que lo usa
    // hasta ahora, precisamente porque volteando la convención global se
    // rompían las demás fases.
    bool      invertir_direccion = false;
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
    ColorLabel back         = ColorLabel::UNKNOWN;
    bool       back_valid   = false;
};

struct ReflectanceReading {
    uint32_t timestamp_ms  = 0;
    uint16_t left_raw      = 0;   // "crudo": ADC con el emisor prendido
    uint16_t right_raw     = 0;
    // con_luz - ambiente (ver la nota grande de rechazo de luz ambiente
    // junto a ReflectanceTask): |restado| chico = negro (absorbe casi
    // toda la luz del emisor), |restado| grande = superficie clara
    // (rebota mucho más). left_restado no se usa para decidir on_line
    // (ver left_on_line) -- se reporta solo para diagnóstico.
    int16_t  left_restado  = 0;
    int16_t  right_restado = 0;
    bool     left_on_line  = false;
    bool     right_on_line = false;
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
static QueueHandle_t g_healthQueue     = nullptr;

// ---------------------------------------------------------------------------
//  Mutex del bus I2C nº1 (Wire1) — TCS34725 trasero + PCA9685 (servos).
// ---------------------------------------------------------------------------
//  Único bus I2C de este firmware: el bus nº0 (TCS34725 delantero + ToF) se
//  retiró por completo, ver la nota grande al principio del archivo.
//
//  GripperTask (el PCA9685) corre en el núcleo 1, ColorSensorTask (el
//  TCS34725 trasero) en el núcleo 0 — pueden estar ejecutando una
//  transacción I2C cada una AL MISMO TIEMPO sobre el mismo objeto `Wire1`,
//  y ninguna deshabilita el planificador durante beginTransmission()/
//  endTransmission(). Este mutex serializa toda transacción sobre el bus:
//  cada tarea lo toma antes de tocar `Wire1` y lo suelta apenas termina.
//
//  Con timeout corto (no portMAX_DELAY, por la misma regla que las colas):
//  si no se consigue el bus en I2C1_LOCK_TIMEOUT_MS, la operación se da por
//  fallida esta vuelta y se reintenta en la siguiente.
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
//  [5] DRIVER PCA9685 — servos del gripper por I2C
// ===========================================================================
//
//  Vive en Wire1 (bus I2C nº1), no en Wire — decisión del equipo para no
//  sumar un tercer dispositivo al bus 0 (ver Pins::I2C1_SDA/SCL y
//  g_i2c1Mutex). Solo hay un PCA9685 y siempre va a estar en el mismo bus,
//  así que no hace falta recibirlo como parámetro (a diferencia de
//  Tcs34725::, que sí atiende dos sensores en dos buses distintos).

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
//  muestras reales del TCS34725 delantero (ver calibracion/color/). Si se
//  recalibra, actualizar los DOS firmwares — igual que ya advierte esa nota
//  allá, ahora con un tercer archivo (este) para no olvidar.

static ColorLabel ClassifyColor(const Tcs34725::Rgbc &s) {
    // BLACK se conserva como resultado del clasificador para diagnóstico,
    // pero NO participa en la detección del borde. La maniobra de evasión y
    // su indicación morada dependen exclusivamente de los QTR; ver
    // MissionTask. El TCS34725 solo decide zonas amarilla/roja/azul.
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
//  (numeración de subsecciones: 8.1 Motor, 8.2 Gripper, 8.3 Color,
//  8.4 Reflectancia, 8.5 LED — sin ToF, retirado del bus I2C nº0)
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

// CÓMO CONTROLA DIRECCIÓN UN L298N (para no seguir adivinando el signo a
// ciegas): cada canal tiene DOS entradas lógicas (IN1/IN2 aquí) más una de
// habilitación PWM (EN, aquí `m.en`). La tabla de verdad real del puente H
// es:
//   IN1=HIGH, IN2=LOW  -> gira en un sentido      (a esto llamamos "forward")
//   IN1=LOW,  IN2=HIGH -> gira en el sentido contrario ("no forward")
//   IN1=IN2=LOW        -> rueda libre (freno suave, sin corriente)
//   IN1=IN2=HIGH       -> frenado activo (cortocircuita el motor)
//   EN en 0% de duty   -> el motor NO gira sin importar IN1/IN2 -- por eso
//                         `PwmWrite` recibe abs(speed): la magnitud siempre
//                         es un duty positivo, el signo de `speed` SOLO
//                         decide cuál de las dos combinaciones de IN1/IN2
//                         se manda, nunca "un PWM negativo" (eso no existe).
//
// Con esto claro: el código de abajo YA implementa la tabla de verdad
// correctamente (nunca fue el bug). Lo que puede estar mal es cuál de las
// dos combinaciones corresponde a "avanza" en la realidad -- y eso NO se
// resuelve volteando este booleano cada vez que una sola fase se ve mal:
// la última prueba (invertir para arreglar RETROCEDER_A_ZONA_NEUTRA) hizo
// que el robot arrancara mal en TODAS las fases, así que el booleano se
// revirtió a su valor anterior. Antes de tocarlo de nuevo, hay que medir
// en banco, RUEDA POR RUEDA (no las 4 a la vez), qué combinación de IN1/IN2
// corresponde a qué sentido físico -- kMotorFL ya tiene IN1/IN2
// intercambiados a propósito por su cableado físico (ver el comentario
// junto a su declaración), así que puede que no todas las ruedas compartan
// la misma convención y el problema no sea este booleano global en
// absoluto, sino cuál motor específico está mal cableado o mal
// compensado.
//
// `invertir_direccion`: en vez de seguir tocando la convención global de
// arriba (que ya demostró romper otras fases al voltearla), esto manda
// EXPLÍCITAMENTE la combinación de IN1/IN2 contraria a la que el signo de
// `speed` produciría solo -- pedido explícito para usarlo ÚNICAMENTE en
// RETROCEDER_A_ZONA_NEUTRA, sin afectar cómo se mueve el robot en
// cualquier otra fase.
void MotorApply(const Motor &m, int speed, bool invertir_direccion = false) {
    speed = constrain(speed, -100, 100);
    bool forward = (speed < 0);
    if (invertir_direccion) forward = !forward;
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
            MotorApply(kMotorFL, current.left,  current.invertir_direccion);
            MotorApply(kMotorRL, current.left,  current.invertir_direccion);
            MotorApply(kMotorFR, current.right, current.invertir_direccion);
            MotorApply(kMotorRR, current.right, current.invertir_direccion);
        }

        Heartbeat(TaskId::MOTOR_CONTROL);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  8.2  GripperTask — 1 servo vía PCA9685
// ---------------------------------------------------------------------------
//  El robot tiene UN SOLO servo de gripper: agarra o suelta. Ángulos
//  calibrados con pruebas-platformio/06-calibracion-gripper/, los mismos que
//  usa firmware-esp32/.

constexpr int kClawOpenDeg          = 0;
constexpr int kClawClosedLlaveDeg   = 120;
constexpr int kClawClosedBanderaDeg = 65;

void GripperTask(void *) {
    // El PCA9685 vive en el bus I2C nº1, junto con el TCS34725 trasero (ver
    // la nota larga junto a g_i2c1Mutex, más arriba): cada bloque que toca
    // el bus toma el mutex y lo suelta apenas termina, nunca a mitad de una
    // secuencia de varias llamadas.
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
                // Si el PCA9685 no respondió al arrancar (arriba se saltó el
                // SetChannel inicial) y recién ahora reaparece, hay que
                // mandar la pinza a 0° aquí también — si no, se queda
                // esperando un GripperCommand que puede no llegar nunca (por
                // ejemplo, con Mission::kMotionEnabled en false, que no
                // manda ninguno).
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
                        break;   // acción desconocida: se ignora
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
//  8.3  ColorSensorTask — TCS34725 trasero (Wire1), único sensor de color
// ---------------------------------------------------------------------------

void ColorSensorTask(void *) {
    // Único sensor y único bus desde que se retiró el bus I2C nº0 (ver la
    // nota grande al principio del archivo): sus llamadas van protegidas
    // por g_i2c1Mutex, que comparte con el PCA9685 (GripperTask corre en el
    // otro núcleo).
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
        if (!back_ok && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            if (I2c1Lock()) {
                back_ok = Tcs34725::Init(Wire1);
                I2c1Unlock();
            }
        }

        ColorReading reading;
        reading.timestamp_ms = millis();

        Tcs34725::Rgbc sample;
        if (back_ok) {
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
//  8.4  ReflectanceTask — 2x QTRX-HD-01A, con rechazo de luz ambiente
// ---------------------------------------------------------------------------
//  CALIBRADO EN BANCO (robot físicamente quieto sobre cada superficie,
//  ver el historial de esta sesión de depuración) comparando una lectura
//  con el emisor IR prendido contra una con el emisor apagado de verdad:
//
//    restado = (ADC con emisor prendido) - (ADC con emisor apagado)
//
//  Con el sensor DERECHO (único que funciona -- ver la nota sobre el
//  izquierdo más abajo):
//    negro (absorbe casi toda la luz del emisor): |restado| ~= 1 a 8
//    gris  (rebota mucho más):                    |restado| ~= 76 a 87
//
//  |restado| CHICO = negro, GRANDE = superficie clara. Antes se comparaba
//  el ADC crudo (con el emisor siempre prendido) contra un umbral fijo,
//  pero esa lectura mezcla el reflejo del piso CON cualquier luz ambiente
//  que le llegue al fototransistor -- con suficiente luz ambiente, esa
//  componente domina y aplana la diferencia entre negro y gris (eso es
//  justo lo que pasaba: crudo salía casi igual en las dos superficies).
//  Restar una lectura con el emisor apagado aísla la parte que de verdad
//  depende del color del piso.
//
//  IMPORTANTE sobre el pulso de apagado: según el datasheet del QTRX-HD,
//  un LOW de 0.5-300 us en CTRL NO apaga el emisor -- lo interpreta como
//  un PULSO DE ATENUACIÓN (baja un escalón de 32 en el brillo, ~3.33%) y
//  lo deja prendido. Solo un LOW sostenido >= 1 ms apaga los LEDs de
//  verdad; ese mismo LOW seguido de HIGH reinicia el emisor a corriente
//  completa (100%), así que no hace falta re-sincronizar ningún estado de
//  atenuación entre lecturas.

// Umbral sobre |restado|: a la mitad entre el peor caso de negro (~8) y
// el mejor caso de gris (~76), con margen generoso a los dos lados.
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
        delayMicroseconds(200);   // asentar el fototransistor con luz IR ya estable
        const uint16_t left_raw  = (uint16_t)analogRead(Pins::QTR_LEFT_OUT);
        const uint16_t right_raw = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);

        ReflectanceReading reading;
        reading.timestamp_ms  = millis();
        reading.left_raw       = left_raw;
        reading.right_raw      = right_raw;
        reading.left_restado   = (int16_t)((int32_t)left_raw  - (int32_t)left_ambiente);
        reading.right_restado  = (int16_t)((int32_t)right_raw - (int32_t)right_ambiente);

        reading.right_on_line = abs(reading.right_restado) < kBordeRestadoUmbral;

        // IZQUIERDO: sensor confirmado roto en banco -- pegado en 4095 sin
        // importar superficie NI estado del emisor (restado ronda 0
        // constante, que bajo este criterio se leería como "negro"). Si
        // se dejara participar, el lado izquierdo dispararía la evasión
        // todo el tiempo. Forzado en false hasta reparar el hardware;
        // left_raw/left_restado se siguen reportando para diagnóstico.
        reading.left_on_line = false;

        PushDropOldest(g_reflectQueue, reading);

        Heartbeat(TaskId::REFLECTANCE);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  8.5  LedTask — LED RGB: indicador puro de la línea/zona de piso
// ---------------------------------------------------------------------------
//  Único trabajo del LED en esta variante: decir sobre qué está parado el
//  robot AHORA MISMO, nada más — ni equipo, ni "veo la bandera".
//    · Piso gris (FLOOR) o sin lectura válida -> apagado.
//    · Amarillo / rojo / azul                 -> ese mismo color, fijo.
//    · Negro (borde)                          -> morado, fijo.

namespace RgbLed {
    constexpr uint8_t CH_R = 4;
    constexpr uint8_t CH_G = 5;
    constexpr uint8_t CH_B = 6;

    // POLARIDAD CONFIRMADA con el LED físico: es CÁTODO COMÚN, o sea duty
    // alto = canal más brillante (mismo caso que firmware-esp32/). La
    // constante se conserva por si algún día se cambia el LED por uno de
    // ánodo común.
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

    // Color fijo de cada zona -- BLACK incluido, ya no parpadea.
    void ApplyZone(ColorLabel zone) {
        switch (zone) {
            case ColorLabel::YELLOW: SetRaw(255, 170, 0); break;
            case ColorLabel::RED:    SetRaw(255, 0, 0);   break;
            case ColorLabel::BLUE:   SetRaw(0, 0, 255);   break;
            case ColorLabel::BLACK:  SetRaw(160, 0, 200); break;   // morado
            case ColorLabel::FLOOR:
            case ColorLabel::UNKNOWN:
            default:
                SetRaw(0, 0, 0);   // apagado: piso neutro o sin lectura
                break;
        }
    }
}

void LedTask(void *) {
    RgbLed::Setup();
    RgbLed::SetRaw(0, 0, 0);

    ColorLabel zone = ColorLabel::UNKNOWN;

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::LED_STATUS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        LedCommand cmd;
        if (xQueueReceive(g_ledCmdQueue, &cmd, 0) == pdTRUE) {
            zone = cmd.zone;
        }

        RgbLed::ApplyZone(zone);

        Heartbeat(TaskId::LED_STATUS);
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace anónimo (tareas de hardware)

// ===========================================================================
//  [9] MISSIONTASK — el cerebro, sin Raspberry Pi
// ===========================================================================
//
//  Puerto parcial de la lógica de raspberry-pi/src/athena/decision.py a
//  C++: solo la parte de "cargar y depositar la llave". Esta variante NO
//  intenta buscar ni agarrar la bandera del oponente — ver el aviso grande
//  al principio del archivo sobre por qué (se retiró el bus I2C nº0, único
//  sensor con el que esta demo podía intentarlo). La prioridad de no
//  salirse de la pista sí es EXACTAMENTE la misma que allá: salirse de la
//  pista pierde la ronda de inmediato, así que esa regla no puede depender
//  de que nada más ande bien.

namespace Mission {

// Interruptor de emergencia: en false, MissionTask NO mueve motores ni el
// gripper — solo lee los sensores y deja que el LED (LedTask) siga mostrando
// la zona de piso. Sirve para probar sensores en el banco, o para desactivar
// el robot sin desconectarle nada.
//
// Estuvo en false durante unas pruebas en las que el robot se reiniciaba
// solo. La causa resultó ser un problema de CONEXIÓN, no de batería baja
// como se sospechó al principio, así que ya no hay motivo para dejarlo
// apagado: la misión completa está habilitada.
constexpr bool kMotionEnabled = true;

// Velocidades y umbrales — mismo rol que ControlConfig en decision.py,
// pero constexpr porque aquí no hay archivo de configuración que cargar.
constexpr int kVelocidadCrucero     = 70;   // % de PWM al avanzar recto
constexpr int kVelocidadAproximacion = 60;  // % al acercarse / evadir

constexpr uint32_t kStartupDelayMs   = 3000;  // tiempo para cargar la llave y ubicar el robot
constexpr uint32_t kGripperSettleMs  = 400;   // tiempo mecánico para que el servo llegue
// Full stop al llegar a la zona amarilla, antes de retroceder: pedido
// explícito para que el robot se detenga por completo (no una frenada
// suave a mitad de un SetDrive) antes de invertir el sentido de marcha.
constexpr uint32_t kFullStopMs       = 400;
// Retrocede a tiempo fijo justo DESPUÉS de que el sensor trasero detecta la
// zona amarilla, para que el frente (y la llave) quede dentro de la zona
// segura y no más allá de ella. Variable fácil de ajustar: solo este
// número, en milisegundos.
constexpr uint32_t kRetrocesoZonaNeutraMs = 700;
// Evasión de borde negro, en CUATRO tiempos fijos (sin sensor, igual
// criterio que kRetrocesoZonaNeutraMs): full stop, retroceder, full stop
// otra vez, y luego girar ~180° antes de retomar la búsqueda. Números
// fáciles de ajustar en banco según cuánto se pasa/gira de verdad.
//
// Los dos full stop (antes de retroceder, y antes de girar) son pedido
// explícito: detenerse por completo (motor en STOP, no una frenada a
// mitad de un SetDrive) antes de cada cambio de sentido de marcha, mismo
// criterio que kFullStopMs para la zona amarilla.
// Pedido explícito: corto a propósito (no como kDetenerAntesGiroMs) -- con
// el motor sin energía el robot sigue deslizándose por inercia igual,
// dure lo que dure la parada; entre más tiempo se quede sin corregir, más
// se puede seguir saliendo de la pista antes de que el retroceso empiece
// a traerlo de vuelta.
constexpr uint32_t kDetenerAntesRetrocesoMs = 200;
// Valores recuperados de la calibracion sobre el chasis real hecha en
// pruebas-platformio/01-mantente-en-cuadro (commit b435181): 1.5 s para
// alejar las cuatro ruedas del borde y 2 s para completar el pivote. Los
// 500 ms originales eran solo una estimacion y el giro quedaba incompleto.
constexpr uint32_t kEvasionRetrocesoMs      = 1500;
constexpr uint32_t kDetenerAntesGiroMs      = 1000;
constexpr uint32_t kEvasionGiroMs           = 2000;
// Velocidad del giro de evasión -- pedido explícito: máxima velocidad
// (100%), no kVelocidadAproximacion como el resto de las maniobras de
// borde.
constexpr int kVelocidadGiroMax = 100;
// Debounce del borde negro: los QTR tienen que reportar on_line de forma
// CONTINUA durante al menos este tiempo antes de disparar la maniobra de
// evasión -- pedido explícito para filtrar falsos positivos (ruido de un
// instante, no la calibración en sí: kBordeRestadoUmbral ya está calibrado
// en banco, ver ReflectanceTask). Un solo instante con on_line=true ya no
// alcanza: si en cualquier momento se deja de detectar, el conteo se
// reinicia desde cero.
constexpr uint32_t kBordeDebounceMs = 50;
// Mismo criterio de debounce que kBordeDebounceMs, pero para la zona
// amarilla (sensor de color trasero) en vez del borde negro (QTR) -- ver
// el cerrojo zona_neutra_detectada, más abajo en MissionTask.
constexpr uint32_t kZonaNeutraDebounceMs = 50;
// Gracia al ENTRAR a BUSCAR_ZONA_NEUTRA (arranque de la misión, y también
// cada vez que se vuelve aquí tras un giro de evasión): durante este tiempo
// se IGNORA por completo la lectura de los QTR y el robot avanza sí o sí.
// Pedido explícito -- la pista es gris con manchas de suciedad más oscuras
// que aunque kBordeRestadoUmbral ya esté calibrado (ver ReflectanceTask)
// podrían seguir leyendo parecido al borde real, y sin esto el robot podía
// ponerse a retroceder desde el segundo 0 sin haber avanzado nunca.
// kBordeDebounceMs (arriba) filtra ruido de un instante; esto filtra "la
// mancha bajo el sensor justo en este momento", que puede sostenerse mucho
// más que 50 ms si el robot no se ha movido todavía.
constexpr uint32_t kIgnorarBordeAlEntrarMs = 1000;

enum class Phase : uint8_t {
    ARRANQUE = 0,
    ASEGURAR_LLAVE,
    BUSCAR_ZONA_NEUTRA,
    // Borde negro detectado durante BUSCAR_ZONA_NEUTRA: full stop,
    // retroceder, full stop otra vez, girar ~180°, y volver a
    // BUSCAR_ZONA_NEUTRA a seguir merodeando.
    EVADIR_BORDE_DETENER_ANTES_RETROCESO,
    EVADIR_BORDE_RETROCESO,
    EVADIR_BORDE_DETENER_ANTES_GIRO,
    EVADIR_BORDE_GIRO,
    // Full stop al detectar la zona amarilla, antes de retroceder -- ver
    // kFullStopMs.
    DETENER_ZONA_NEUTRA,
    // Retrocede a tiempo fijo tras detectar la zona amarilla: el sensor
    // TRASERO ya la pasó de largo en el instante en que la detecta, así
    // que sin este paso el frente (y la llave) quedarían más allá de la
    // zona segura, no dentro.
    RETROCEDER_A_ZONA_NEUTRA,
    DEPOSITAR_LLAVE,
    TERMINADO,
};

// Nombre legible de cada fase, para el log — sin esto, el monitor serial
// solo mostraba el número crudo del enum y había que contar a mano.
inline const char *PhaseName(Phase phase) {
    switch (phase) {
        case Phase::ARRANQUE:            return "ARRANQUE";
        case Phase::ASEGURAR_LLAVE:      return "ASEGURAR_LLAVE";
        case Phase::BUSCAR_ZONA_NEUTRA:  return "BUSCAR_ZONA_NEUTRA";
        case Phase::EVADIR_BORDE_DETENER_ANTES_RETROCESO: return "EVADIR_BORDE_DETENER_ANTES_RETROCESO";
        case Phase::EVADIR_BORDE_RETROCESO: return "EVADIR_BORDE_RETROCESO";
        case Phase::EVADIR_BORDE_DETENER_ANTES_GIRO: return "EVADIR_BORDE_DETENER_ANTES_GIRO";
        case Phase::EVADIR_BORDE_GIRO:   return "EVADIR_BORDE_GIRO";
        case Phase::DETENER_ZONA_NEUTRA: return "DETENER_ZONA_NEUTRA";
        case Phase::RETROCEDER_A_ZONA_NEUTRA: return "RETROCEDER_A_ZONA_NEUTRA";
        case Phase::DEPOSITAR_LLAVE:     return "DEPOSITAR_LLAVE";
        case Phase::TERMINADO:           return "TERMINADO";
        default:                         return "DESCONOCIDA";
    }
}

} // namespace Mission

// Evita depender de que el compilador trate a MotorCommand como agregado
// con inicializadores por defecto (necesita C++14+); igual que hace
// firmware-esp32/src/main.cpp, se asigna campo a campo.
inline void SetDrive(MotorCommand &m, int left, int right, bool invertir_direccion = false) {
    m.mode  = MotorMode::DRIVE;
    m.left  = (int8_t)constrain(left, -100, 100);
    m.right = (int8_t)constrain(right, -100, 100);
    m.invertir_direccion = invertir_direccion;
}

void MissionTask(void *pvTeam) {
    const TeamColor team = *reinterpret_cast<TeamColor *>(pvTeam);

    Mission::Phase phase = Mission::Phase::ARRANQUE;
    Mission::Phase last_logged_phase = phase;
    uint32_t phase_started_ms = millis();

    ColorReading       last_color{};
    ReflectanceReading last_reflect{};
    // Cerrojo: en cuanto el sensor trasero ve amarillo UNA vez estando en
    // BUSCAR_ZONA_NEUTRA, esto pasa a true y ya NUNCA vuelve a false --
    // pedido explícito: "que se detenga por completo al detectar la zona,
    // sin importar si luego empezaste a leer otra cosa" (ej. por inercia
    // el robot sigue deslizándose un poco y el sensor deja de ver amarillo
    // un instante después). Solo importa mientras phase==BUSCAR_ZONA_NEUTRA
    // (ver el switch más abajo): una vez en DETENER_ZONA_NEUTRA en adelante
    // el estado de este cerrojo ya no se vuelve a consultar.
    bool zona_neutra_detectada = false;

    // Debounce del borde negro (ver Mission::kBordeDebounceMs): 0 = no se
    // está detectando borde ahora mismo; en cuanto on_line se pone true,
    // guarda el millis() de ESE instante y no se reinicia hasta que
    // on_line vuelva a false -- así (millis() - esto) mide cuánto lleva
    // detectándose SIN INTERRUPCIÓN, no cuántas veces se detectó.
    uint32_t borde_detectado_desde_ms = 0;

    // Mismo mecanismo de debounce, para la zona amarilla (ver el cerrojo
    // zona_neutra_detectada más abajo y Mission::kZonaNeutraDebounceMs).
    uint32_t amarillo_detectado_desde_ms = 0;

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
        HealthReport h;
        while (xQueueReceive(g_healthQueue, &h, 0) == pdTRUE) {
            DEBUG_LINK.printf("[Mission] tareas colgadas, bitmask=0x%02X\n", h.faulted_tasks_bitmask);
        }

        MotorCommand motor;               // por defecto: STOP
        GripperCommand gripper;
        bool send_gripper = false;

        // Interruptor de batería baja (Mission::kMotionEnabled): si está en
        // false, todo lo de aquí abajo se salta por completo. motor se queda
        // en su valor por defecto (STOP) y gripper nunca se envía — el
        // robot no mueve nada, solo sigue leyendo sensores y mostrando la
        // zona por el LED (eso pasa siempre, fuera de este bloque).
        if (Mission::kMotionEnabled) {

        // --- 0b. Cerrojo de zona neutra: detenerse YA, para siempre -------
        // Se evalúa ANTES del switch(phase) de abajo: en cuanto el sensor
        // trasero ve amarillo de forma CONTINUA durante al menos
        // Mission::kZonaNeutraDebounceMs (mismo criterio de debounce que
        // el borde negro, ver kBordeDebounceMs -- un instante suelto no
        // cuenta, y cualquier lectura distinta reinicia el conteo), esto
        // cambia de fase inmediatamente y zona_neutra_detectada nunca
        // vuelve a false -- así el resto de la secuencia (parada,
        // retroceso, soltar la llave) no se puede reabrir por una lectura
        // posterior, sin importar qué color se lea después.
        //
        // Mission::kIgnorarBordeAlEntrarMs también se aplica AQUÍ, no solo
        // a la evasión de borde: pedido explícito, porque el piso FUERA de
        // la pista (la casa) también es amarillo. Si el robot cruza el
        // borde negro (se sale de la pista) y termina retrocediendo sobre
        // ese piso amarillo real, sin esta gracia el sensor trasero vería
        // amarillo genuino ahí mismo y dispararía un falso "zona neutra
        // encontrada" justo cuando en realidad el robot está afuera de la
        // pista, no sobre la zona segura. Mantenerse DENTRO de la pista es
        // imperativo -- no se vuelve a evaluar esto hasta pasado el mismo
        // tiempo que ya se le da al borde para asentarse tras la maniobra
        // de evasión completa (parada+retroceso+parada+giro).
        if (!zona_neutra_detectada && phase == Mission::Phase::BUSCAR_ZONA_NEUTRA &&
            (uint32_t)(millis() - phase_started_ms) > Mission::kIgnorarBordeAlEntrarMs) {
            const bool ve_amarillo = last_color.back_valid && last_color.back == ColorLabel::YELLOW;
            if (ve_amarillo) {
                if (amarillo_detectado_desde_ms == 0) {
                    amarillo_detectado_desde_ms = millis();
                } else if ((uint32_t)(millis() - amarillo_detectado_desde_ms) >= Mission::kZonaNeutraDebounceMs) {
                    zona_neutra_detectada = true;
                    phase = Mission::Phase::DETENER_ZONA_NEUTRA;
                    phase_started_ms = millis();
                }
            } else {
                amarillo_detectado_desde_ms = 0;
            }
        }

        // Log de reflectancia: valores crudos + restado (ver la nota
        // grande de rechazo de luz ambiente en ReflectanceTask) contra
        // kBordeRestadoUmbral. izq siempre sale on_line=0 a propósito
        // (sensor roto, forzado en ReflectanceTask) -- su raw/restado se
        // imprime solo para seguir monitoreando si algún día se repara.
        {
            static uint32_t last_reflect_log_ms = 0;
            if ((uint32_t)(millis() - last_reflect_log_ms) > 500) {
                last_reflect_log_ms = millis();
                DEBUG_LINK.printf(
                    "[Reflect] crudo: izq=%u der=%u | restado: izq=%d der=%d (umbral=|%d|) "
                    "on_line: izq=%d der=%d -> zona_neutra_detectada=%d\n",
                    last_reflect.left_raw, last_reflect.right_raw,
                    last_reflect.left_restado, last_reflect.right_restado, kBordeRestadoUmbral,
                    last_reflect.left_on_line, last_reflect.right_on_line,
                    zona_neutra_detectada);
            }
        }

        // Ayuda a depurar en banco: cambiar de fase se ve en el monitor
        // serial sin tener que instrumentar cada rama.
        if (phase != last_logged_phase) {
            DEBUG_LINK.printf("[Mission] fase -> %s\n", Mission::PhaseName(phase));
            last_logged_phase = phase;
        }

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

                // -- 4. Avanzar/merodear hasta pisar la zona amarilla -------
                // Usa el sensor TRASERO: el delantero se retiró por
                // completo junto con el bus I2C nº0 (ver aviso al
                // principio del archivo). La transición a
                // DETENER_ZONA_NEUTRA ya no se decide aquí: la maneja el
                // cerrojo zona_neutra_detectada, más arriba. Si seguimos
                // viendo esta fase, es que todavía no se detectó el
                // amarillo: seguir avanzando, salvo que el borde negro
                // interrumpa con la maniobra de evasión de abajo.
                //
                // OJO CON EL SIGNO: antes se mandaba kVelocidadCrucero en
                // POSITIVO, pero nunca se probó de verdad en banco porque
                // la evasión de borde (ahora reemplazada por las dos fases
                // de abajo) tapaba este case el 100% del tiempo en toda
                // prueba hecha hasta ahora. La evasión vieja usaba
                // velocidad NEGATIVA sin invertir_direccion y eso sí se
                // confirmó en banco que avanza físicamente -- por eso este
                // avance usa el mismo signo negativo ahora, para no volver
                // a mandar el robot para atrás en cuanto esta fase por fin
                // se ejecute de verdad.
                case Mission::Phase::BUSCAR_ZONA_NEUTRA: {
                    // Gracia de arranque (Mission::kIgnorarBordeAlEntrarMs,
                    // ver su declaración): recién entrando a esta fase (al
                    // arrancar la misión, o al volver aquí tras un giro de
                    // evasión) se ignora el borde por completo -- la pista
                    // gris con manchas de suciedad puede leer "negro" bajo
                    // el sensor sin que el robot se haya movido nunca, y
                    // eso no es un borde real que evadir. Ni siquiera se
                    // toca borde_detectado_desde_ms aquí: el debounce de
                    // abajo arranca limpio en cuanto termine esta gracia.
                    if ((uint32_t)(millis() - phase_started_ms) <= Mission::kIgnorarBordeAlEntrarMs) {
                        SetDrive(motor, -Mission::kVelocidadCrucero, -Mission::kVelocidadCrucero);
                        break;
                    }

                    // Debounce (Mission::kBordeDebounceMs, ver su
                    // declaración): un solo instante con on_line=true no
                    // dispara la evasión -- tiene que sostenerse SIN
                    // INTERRUPCIÓN al menos ese tiempo. Mientras se está
                    // confirmando (o si no hay borde en absoluto) el robot
                    // sigue avanzando normalmente.
                    const bool en_borde = last_reflect.left_on_line || last_reflect.right_on_line;
                    if (en_borde) {
                        if (borde_detectado_desde_ms == 0) {
                            borde_detectado_desde_ms = millis();
                        } else if ((uint32_t)(millis() - borde_detectado_desde_ms) >= Mission::kBordeDebounceMs) {
                            phase = Mission::Phase::EVADIR_BORDE_DETENER_ANTES_RETROCESO;
                            phase_started_ms = millis();
                            borde_detectado_desde_ms = 0;
                            break;
                        }
                    } else {
                        borde_detectado_desde_ms = 0;
                    }
                    SetDrive(motor, -Mission::kVelocidadCrucero, -Mission::kVelocidadCrucero);
                    break;
                }

                // -- 4·i. Borde negro: full stop antes de retroceder --------
                // Pedido explícito: detenerse por completo (motor en STOP,
                // no una frenada a mitad de un SetDrive) antes de invertir
                // el sentido de marcha. motor se queda en su valor por
                // defecto (STOP): no hace falta llamar a SetDrive aquí.
                case Mission::Phase::EVADIR_BORDE_DETENER_ANTES_RETROCESO: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kDetenerAntesRetrocesoMs) {
                        phase = Mission::Phase::EVADIR_BORDE_RETROCESO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // -- 4·ii. Borde negro: retroceder a tiempo fijo ------------
                // Mismo mecanismo (signo negativo + invertir_direccion) ya
                // confirmado en banco para RETROCEDER_A_ZONA_NEUTRA, más
                // abajo -- reutilizado tal cual, sin inventar una tercera
                // combinación de signo/flag.
                case Mission::Phase::EVADIR_BORDE_RETROCESO: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kEvasionRetrocesoMs) {
                        phase = Mission::Phase::EVADIR_BORDE_DETENER_ANTES_GIRO;
                        phase_started_ms = millis();
                    } else {
                        SetDrive(motor, -Mission::kVelocidadAproximacion, -Mission::kVelocidadAproximacion,
                                 /*invertir_direccion=*/true);
                    }
                    break;
                }

                // -- 4·iii. Borde negro: full stop antes de girar -----------
                // Mismo criterio que EVADIR_BORDE_DETENER_ANTES_RETROCESO,
                // arriba: parar por completo antes del siguiente cambio de
                // sentido (esta vez, antes de girar).
                case Mission::Phase::EVADIR_BORDE_DETENER_ANTES_GIRO: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kDetenerAntesGiroMs) {
                        phase = Mission::Phase::EVADIR_BORDE_GIRO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // -- 4·iv. Borde negro: gira ~180° SIEMPRE A LA DERECHA -----
                // Pedido explícito: siempre gira hacia la derecha (nunca
                // hacia la izquierda, sin importar de qué lado vino el
                // borde) y a velocidad MÁXIMA en ambos lados
                // (kVelocidadGiroMax = 100, no kVelocidadAproximacion).
                // Lado IZQUIERDO en sentido "avanza" (negativo) + lado
                // DERECHO en sentido "retrocede" (positivo, sin necesitar
                // invertir_direccion): con un lado empujando hacia adelante
                // y el otro hacia atrás, el robot rota sobre su propio eje
                // -- izquierdo adelante + derecho atrás rota hacia la
                // derecha (mismo principio que un tanque girando: la oruga
                // izquierda avanza, la derecha retrocede, y el frente
                // "apunta" hacia la derecha). Al terminar, vuelve a
                // BUSCAR_ZONA_NEUTRA a seguir merodeando.
                case Mission::Phase::EVADIR_BORDE_GIRO: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kEvasionGiroMs) {
                        phase = Mission::Phase::BUSCAR_ZONA_NEUTRA;
                        phase_started_ms = millis();
                    } else {
                        SetDrive(motor, -Mission::kVelocidadGiroMax, Mission::kVelocidadGiroMax);
                    }
                    break;
                }

                // -- 4a. Full stop antes de invertir el sentido de marcha ---
                // Pedido explícito: detenerse por completo (motor en STOP,
                // no una frenada a mitad de un SetDrive) antes de
                // retroceder, en vez de pasar directo de avanzar a
                // retroceder. motor se queda en su valor por defecto
                // (STOP): no hace falta llamar a SetDrive aquí.
                case Mission::Phase::DETENER_ZONA_NEUTRA: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kFullStopMs) {
                        phase = Mission::Phase::RETROCEDER_A_ZONA_NEUTRA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // -- 4b. Retroceder para que el FRENTE (no solo el sensor
                // trasero) quede dentro de la zona amarilla --------------
                // El trasero recién detecta el amarillo cuando ya lo pasó
                // de largo -- sin este paso, la llave (al frente) quedaría
                // más allá de la zona segura, no dentro. Tiempo fijo, sin
                // sensor: Mission::kRetrocesoZonaNeutraMs es el único número
                // que hay que ajustar en banco según cuánto se pasa.
                //
                // invertir_direccion=true: pedido explícito para NO volver
                // a tocar la convención global de MotorApply (voltearla
                // rompió las demás fases, ver la nota grande junto a
                // MotorApply) y en su lugar forzar la combinación de
                // IN1/IN2 contraria SOLO en esta fase, la única que
                // necesita moverse hacia atrás de verdad.
                case Mission::Phase::RETROCEDER_A_ZONA_NEUTRA: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kRetrocesoZonaNeutraMs) {
                        phase = Mission::Phase::DEPOSITAR_LLAVE;
                        phase_started_ms = millis();
                    } else {
                        // OJO: el signo aquí SÍ importa incluso con
                        // invertir_direccion=true -- son dos inversiones
                        // independientes (signo de `speed`, y el flag) y
                        // hay que combinarlas bien. La evasión de borde ya
                        // demostró en banco que speed NEGATIVO sin invertir
                        // produce "avanza" físicamente; para obtener el
                        // estado contrario hay que partir de ESE mismo
                        // signo negativo y sí invertirlo -- usar positivo
                        // aquí (como se hizo antes) cancela las dos
                        // inversiones y vuelve a producir "avanza".
                        SetDrive(motor, -Mission::kVelocidadAproximacion, -Mission::kVelocidadAproximacion,
                                 /*invertir_direccion=*/true);
                    }
                    break;
                }

                // -- 5. Soltar la llave: fin de la misión de esta demo ------
                // (sin bus I2C nº0 no hay forma de buscar la bandera del
                // oponente -- ver el aviso grande al principio del
                // archivo -- así que la secuencia termina aquí).
                case Mission::Phase::DEPOSITAR_LLAVE: {
                    gripper.action = GripperAction::OPEN;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleMs) {
                        phase = Mission::Phase::TERMINADO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // -- 6. Misión completa: quieto -------------------------------
                case Mission::Phase::TERMINADO:
                default:
                    break;
        }

        } // if (Mission::kMotionEnabled)

        xQueueOverwrite(g_motorCmdQueue, &motor);
        if (send_gripper) xQueueSend(g_gripperCmdQueue, &gripper, 0);

        // BLACK/morado pertenece EXCLUSIVAMENTE a los QTR. El TCS34725 puede
        // clasificar una lectura oscura como BLACK para diagnóstico, pero se
        // ignora aquí para que un falso positivo de color nunca parezca una
        // detección de borde ni dispare ninguna reacción. El sensor de color
        // sigue indicando únicamente las zonas amarilla, roja y azul.
        LedCommand led;
        const bool en_borde_qtr = last_reflect.left_on_line || last_reflect.right_on_line;
        if (en_borde_qtr) {
            led.zone = ColorLabel::BLACK;
        } else if (last_color.back_valid && last_color.back != ColorLabel::BLACK) {
            led.zone = last_color.back;
        } else {
            led.zone = ColorLabel::UNKNOWN;
        }
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
    g_healthQueue     = xQueueCreate(1, sizeof(HealthReport));

    return g_motorCmdQueue && g_gripperCmdQueue && g_ledCmdQueue &&
           g_colorQueue && g_reflectQueue && g_healthQueue;
}

constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// Vive fuera de setup() porque MissionTask la recibe por puntero al crear
// la tarea (xTaskCreatePinnedToCore no admite pasar un TeamColor por valor
// directamente): tiene que sobrevivir a setup() retornando.
static TeamColor g_myTeam = TeamColor::BLUE;   // sobrescrito abajo por el switch de equipo

// Traduce el motivo del último reinicio a algo legible. Existe porque el
// USB nativo se reenumera solo con cada reset del ESP32-S3: un monitor
// serial conectado pierde los primeros logs mientras se reconecta, así que
// "¿por qué se reinició?" es justo la pregunta que más se pierde sin este
// mensaje explícito (ver el bloqueo de reconexión más abajo en setup()).
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

    // Da tiempo a que un monitor serial conectado por USB nativo termine de
    // reconectarse tras un reset (el dispositivo USB se reenumera solo, con
    // este modo, en cada reinicio). Sin esto se pierden justo los primeros
    // logs — los que dicen POR QUÉ se reinició. Acotado: si no hay ningún
    // monitor conectado (el caso normal en competencia, sin laptop), no
    // bloquea más de kSerialWaitMs.
    constexpr uint32_t kSerialWaitMs = 1500;
    const uint32_t wait_start = millis();
    while (!DEBUG_LINK && (millis() - wait_start) < kSerialWaitMs) {
        delay(10);
    }
    delay(200);

    DEBUG_LINK.println("\nAthena Rover 2026 - firmware ESP32-S3 AUTONOMO (sin Raspberry Pi)");
    DEBUG_LINK.printf("[Setup] Motivo del ultimo reinicio: %s\n",
                       ResetReasonToString(esp_reset_reason()));

    // -- Selector de equipo: switch físico de 3 posiciones -------------------
    // Se queda ESPERANDO AQUÍ, antes de crear ninguna tarea (así que nada
    // puede mover motores todavía), hasta que el switch salga de la posición
    // central. Es el procedimiento normal: encender el robot con el switch
    // en 0 y recién ahí elegir equipo -- MissionTask recibe el resultado por
    // puntero y no vuelve a tocar estos pines.
    pinMode(Pins::TEAM_SWITCH_BLUE, INPUT_PULLUP);
    pinMode(Pins::TEAM_SWITCH_RED,  INPUT_PULLUP);
    delay(5);   // deja asentar la lectura tras habilitar los pull-up

    // Gripper a 0 (abierto) apenas se pueda, ANTES de elegir equipo. Pedido
    // explícito: mientras el switch siga en el centro, el gripper no debe
    // quedar en lo que sea que haya dejado un ciclo de energía anterior.
    // Se abre aquí el bus 1 (donde vive el PCA9685) con un intento único,
    // sin bloquear si falla: no hay ninguna otra tarea corriendo todavía
    // (nada se crea hasta después del switch), así que no hace falta el
    // mutex de g_i2c1Mutex para esto — no hay con quién competir por el bus
    // en este punto. GripperTask, más abajo, vuelve a abrir el bus y a
    // reafirmar "abierto" como su primer gesto de todas formas.
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

        // Blanco tenue parpadeando: "esperando instrucción" -- ni un color de
        // equipo (todavía no hay ninguno elegido) ni apagado del todo (para
        // distinguirlo de un ESP32 que no ha arrancado).
        if ((uint32_t)(millis() - blink_ms) > 300) {
            blink_ms = millis();
            blink_on = !blink_on;
            RgbLed::SetRaw(blink_on ? 40 : 0, blink_on ? 40 : 0, blink_on ? 40 : 0);
        }
        delay(20);
    }
    RgbLed::SetRaw(0, 0, 0);
    DEBUG_LINK.printf("[Setup] Switch de equipo -> %s\n",
                       g_myTeam == TeamColor::RED ? "ROJO" : "AZUL");

    // Único bus I2C del firmware: se abre aquí, ANTES de lanzar las tareas,
    // igual que en firmware-esp32/, así ninguna tarea tiene que
    // inicializar hardware compartido por su cuenta. El bus nº0 (TCS34725
    // delantero + VL53L1X) se retiró por completo -- ver el aviso grande
    // al principio del archivo.
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL, 400000);  // TCS34725 trasero + PCA9685
    Wire1.setTimeOut(25);

    if (!CreateQueues()) {
        DEBUG_LINK.println("[FATAL] no se pudieron crear las colas. Arranque detenido.");
        for (;;) delay(1000);
    }

    // Mutex del bus I2C, creado ANTES de lanzar ninguna tarea: ColorSensors
    // y Gripper lo usan desde el primer momento que corren (ver la nota
    // larga junto a su declaración).
    g_i2c1Mutex = xSemaphoreCreateMutex();
    if (g_i2c1Mutex == nullptr) {
        DEBUG_LINK.println("[FATAL] no se pudo crear el mutex del bus I2C. Arranque detenido.");
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
