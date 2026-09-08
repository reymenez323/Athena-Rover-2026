// ===========================================================================
//  Athena Rover 2026 — Firmware ESP32-S3
//  Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  TODO el firmware vive en este archivo, a propósito. Cada subsistema del
//  robot corre en su propia tarea de FreeRTOS, independiente de las demás,
//  comunicadas SOLO por colas explícitas. Un fallo en una tarea no bloquea
//  ni tumba a las otras.
//
//  Dependencias externas: NINGUNA. Solo el framework Arduino-ESP32 (que ya
//  trae FreeRTOS) y su librería Wire. Los drivers del PCA9685 y del TCS34725
//  están escritos aquí mismo: son un puñado de registros I2C cada uno, y
//  escribirlos nos da control total sobre tiempos y ganancia.
//
//  HARDWARE
//    · 2x L298N            -> 4 motores (cada driver mueve 2)
//    · 1x PCA9685 (I2C)    -> 1 servo del gripper (la pinza que abre/cierra)
//    · 1x TCS34725 (I2C)   -> sensor de color TRASERO (el delantero se retiró
//                             junto con el bus I2C nº0, ver más abajo)
//    · 2x QTRX-HD-01A      -> reflectancia delantera izquierda y derecha
//    · LED RGB (1x)        -> identificación de equipo (lo exige el reglamento);
//                             antes eran 2 LED discretos rojo/azul, ya no existen
//    · Switch de 3 posiciones (ON-OFF-ON) -> quién es "nuestro equipo":
//                             0=nadie ha elegido, 1=azul, 2=rojo (ver TeamSwitch)
//    · Enlace con la Raspberry Pi por USB (CDC nativo)
//
//  BUS I2C Nº0 RETIRADO POR COMPLETO (TCS34725 delantero + VL53L1X)
//  ----------------------------------------------------------------------
//  Hasta nuevo aviso: sin ToF (VL53L1X) para medir distancia a la bandera,
//  y sin el TCS34725 delantero. Mismo criterio y mismo hallazgo que en
//  firmware-esp32-standalone/ (ver su historial de commits): el sensor de
//  color delantero nunca dio una conexión física confiable en banco, y el
//  VL53L1X dependía de compartir ese mismo bus con él. En vez de seguir
//  depurando un bus que solo daba problemas, se elimina TODO lo asociado
//  -- pines, mutex, tarea del ToF, telemetría TLM_TOF, lado delantero de
//  ColorSensorTask.
//
//  CONSECUENCIA PARA LA DECISIÓN DE AGARRAR LA BANDERA: sin ToF, la
//  Raspberry Pi (raspberry-pi/src/athena/decision.py) ya no tiene una
//  medición física real de "qué tan cerca está la bandera" -- depende
//  por completo de la distancia estimada por tamaño aparente en la
//  cámara (GeometryConfig.focal_px). Ver el comentario en
//  decision.py::_aproximar_bandera y el docstring de GeometryConfig en
//  config.py para el detalle.
//
//  ÍNDICE
//    [1] Configuración: pines, prioridades, stacks, periodos
//    [2] Tipos compartidos entre tareas
//    [3] Protocolo serial con la Raspberry Pi (formato de cable exacto)
//    [4] Colas
//    [5] Watchdog cooperativo (heartbeats)
//    [6] Driver PCA9685 (servos por I2C)
//    [7] Driver TCS34725 (sensores de color por I2C)
//    [8] Tareas
//    [9] setup() / loop()
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <freertos/semphr.h>   // SemaphoreHandle_t / mutex del bus I2C (ver g_i2c1Mutex)
#include <esp_system.h>        // esp_reset_reason() — ver el log de arranque en setup()

// ===========================================================================
//  [1] CONFIGURACIÓN
// ===========================================================================
//
//  PINES PROHIBIDOS en el ESP32-S3 DevKitC-1 — respetarlos no es opcional:
//    GPIO 19, 20    -> USB nativo (D-/D+). Es por donde habla la Raspberry Pi.
//    GPIO 43, 44    -> U0TXD/U0RXD, puerto de programación y depuración.
//    GPIO 26..32    -> flash SPI interna. Tocarlos cuelga el chip.
//    GPIO 33..37    -> PSRAM Octal (módulos N8R8/N16R8). Igual de intocables.
//    GPIO 0, 45, 46 -> pines de strapping: su nivel al arranque decide el modo
//                      de boot. Si algo los tira a un nivel raro, el ESP32 no
//                      arranca. Evitarlos.
//    GPIO 3         -> strapping de JTAG. Se puede usar, pero mejor no.
//
//  Además: los sensores analógicos DEBEN ir en ADC1 (GPIO 1..10). El ADC2
//  queda inutilizable en cuanto se enciende el WiFi.

namespace Pins {
    // -------- Motores: 2x L298N, cada uno mueve 2 motores ------------------
    // Driver IZQUIERDO -> motor delantero izq. (FL) y trasero izq. (RL)
    constexpr uint8_t L298N_L_IN1 = 4;    // FL sentido A
    constexpr uint8_t L298N_L_IN2 = 5;    // FL sentido B
    constexpr uint8_t L298N_L_ENA = 6;    // FL velocidad (PWM)
    constexpr uint8_t L298N_L_IN3 = 7;    // RL sentido A
    constexpr uint8_t L298N_L_IN4 = 15;   // RL sentido B
    constexpr uint8_t L298N_L_ENB = 16;   // RL velocidad (PWM)

    // Driver DERECHO -> motor delantero der. (FR) y trasero der. (RR)
    constexpr uint8_t L298N_R_IN1 = 10;   // FR sentido A
    constexpr uint8_t L298N_R_IN2 = 11;   // FR sentido B
    constexpr uint8_t L298N_R_ENA = 12;   // FR velocidad (PWM)
    constexpr uint8_t L298N_R_IN3 = 13;   // RR sentido A
    constexpr uint8_t L298N_R_IN4 = 14;   // RR sentido B
    constexpr uint8_t L298N_R_ENB = 17;   // RR velocidad (PWM)

    // -------- Bus I2C nº1: TCS34725 TRASERO + PCA9685 (servos) --------------
    // El bus I2C nº0 (TCS34725 delantero + VL53L1X) se retiró por completo
    // -- ver la nota grande al principio del archivo. Este es ahora el
    // ÚNICO bus I2C del firmware: el TCS34725 trasero y el PCA9685 conviven
    // aquí, protegidos por g_i2c1Mutex por el riesgo de concurrencia entre
    // tareas de distinto núcleo (GripperTask en el núcleo 1, ColorSensorTask
    // en el núcleo 0).
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    // -------- Switch de 3 posiciones (ON-OFF-ON): selección de equipo -------
    // El común del switch va a GND; cada tiro cierra a GND uno de estos 2
    // GPIO. Se leen con pull-up interno (INPUT_PULLUP): HIGH = tiro abierto,
    // LOW = tiro cerrado. En la posición central (0, idle) NINGÚN tiro está
    // cerrado, así que los 2 GPIO leen HIGH — ver el namespace TeamSwitch.
    //
    // GPIO 21 se liberó del LED de iluminación del TCS34725 trasero (ver
    // arriba). GPIO 40 ya se usaba con el mismo propósito, pero como un solo
    // puente a GND de 2 estados, en firmware-esp32-standalone/ (Pins::
    // TEAM_SELECT) — aquí se suma el segundo tiro para tener una posición de
    // reposo real (0 = nadie ha elegido equipo todavía) en vez de arrancar
    // siempre decidido a AZUL u ROJO.
    // Cableado real del robot (confirmado en banco, no el borrador original de
    // hardware/conexiones-esp32-s3.md): el tiro que cablearon como "AZUL"
    // terminó en GPIO 40, y el "ROJO" en GPIO 21 -- al revés de como se había
    // documentado antes de instalar el switch. Los nombres de acá siguen al
    // COMPORTAMIENTO real, no al GPIO original: total, lo único que importa
    // es que TEAM_SWITCH_BLUE apunte al pin que de verdad selecciona azul.
    constexpr uint8_t TEAM_SWITCH_BLUE = 40;  // tiro "AZUL": cerrado a GND = equipo azul
    constexpr uint8_t TEAM_SWITCH_RED  = 21;  // tiro "ROJO": cerrado a GND = equipo rojo

    // -------- Reflectancia QTRX-HD-01A (salida analógica) ------------------
    // ¡ALIMENTARLOS A 3.3 V! La salida del QTRX es proporcional a su VIN: a
    // 5 V entregaría hasta 5 V y eso quema la entrada ADC del ESP32-S3.
    constexpr uint8_t QTR_LEFT_OUT  = 1;  // GPIO1 = ADC1_CH0
    constexpr uint8_t QTR_RIGHT_OUT = 2;  // GPIO2 = ADC1_CH1
    // Pin CTRL de los emisores IR, compartido por ambos sensores. Permite
    // apagar los LED infrarrojos para medir la luz ambiente y restarla.
    constexpr uint8_t QTR_EMITTER_CTRL = 42;

    // -------- LED RGB indicador de equipo -----------------------------------
    // Reemplaza a los 2 LED discretos rojo/azul que documentaba antes esta
    // sección (GPIO 40/41): el equipo ya no los tiene montados, solo queda
    // este único LED RGB. El canal B vivía en GPIO 3 (strapping de JTAG);
    // se corrió a GPIO 41 cuando ese pin todavía tenía otro uso (el XSHUT
    // del VL53L1X, ya retirado -- ver el aviso grande al principio del
    // archivo); se queda en 41 para no recablear sin necesidad.
    constexpr uint8_t LED_RGB_R = 39;
    constexpr uint8_t LED_RGB_G = 38;
    constexpr uint8_t LED_RGB_B = 41;
}

namespace I2CAddr {
    constexpr uint8_t PCA9685  = 0x40;   // dirección por defecto (A0..A5 sin puentear)
    constexpr uint8_t TCS34725 = 0x29;   // fija, no se puede cambiar
}

// Canal del PCA9685 usado por el gripper. El robot tiene UN SOLO servo:
// abre y cierra la pinza, nada más. No hay servo de elevación.
namespace ServoChannel {
    constexpr uint8_t CLAW = 0;   // abre/cierra la pinza
}

namespace Pwm {
    // Motores. El L298N es un driver bipolar antiguo y no conmuta bien a
    // frecuencias altas: 20 kHz lo hace calentar y perder par. 1 kHz es el
    // punto sano, aunque queda dentro del rango audible (se oirá un zumbido).
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;
    constexpr uint8_t  MOTOR_RESOLUTION = 8;     // duty 0..255

    // LED RGB. Fuera del rango audible y sin parpadeo visible a simple vista.
    constexpr uint32_t RGB_FREQ_HZ    = 5000;
    constexpr uint8_t  RGB_RESOLUTION = 8;       // duty 0..255

    // Servos por PCA9685: 50 Hz, 12 bits de resolución (0..4095 por periodo).
    // Un periodo de 20 ms en 4096 pasos -> 1.0 ms = 205 y 2.0 ms = 410.
    constexpr uint32_t SERVO_FREQ_HZ  = 50;
    constexpr uint16_t SERVO_TICK_MIN = 205;   // pulso 1.0 ms ->   0 grados
    constexpr uint16_t SERVO_TICK_MAX = 410;   // pulso 2.0 ms -> 180 grados
}

// Prioridades FreeRTOS (mayor número = mayor prioridad). La loopTask de
// Arduino corre en prioridad 1, así que todo lo que deba ganarle va encima.
namespace TaskPriority {
    constexpr UBaseType_t SUPERVISOR      = 6;  // debe seguir vivo pase lo que pase
    constexpr UBaseType_t MOTOR_CONTROL   = 5;  // crítico: parada de emergencia
    constexpr UBaseType_t SERIAL_COMM     = 4;  // el enlace no debe acumular retraso
    constexpr UBaseType_t GRIPPER_CONTROL = 3;
    constexpr UBaseType_t REFLECTANCE     = 3;  // realimentación rápida de línea
    constexpr UBaseType_t COLOR_SENSOR    = 2;
    constexpr UBaseType_t LED_STATUS      = 1;
}

// OJO: en ESP-IDF (y por tanto en Arduino-ESP32) el parámetro de stack de
// xTaskCreate va en BYTES, no en palabras como en el FreeRTOS de referencia.
// Valores de partida conservadores: medirlos con uxTaskGetStackHighWaterMark()
// en las pruebas de banco y recortarlos ahí.
namespace TaskStack {
    constexpr uint32_t SUPERVISOR      = 3072;
    constexpr uint32_t SERIAL_COMM     = 4096;
    constexpr uint32_t MOTOR_CONTROL   = 3072;
    constexpr uint32_t GRIPPER_CONTROL = 3072;
    constexpr uint32_t REFLECTANCE     = 2560;
    constexpr uint32_t COLOR_SENSOR    = 3584;
    constexpr uint32_t LED_STATUS      = 2048;
}

namespace TaskPeriodMs {
    constexpr uint32_t SUPERVISOR    = 200;
    constexpr uint32_t SERIAL_COMM   = 5;    // 200 Hz
    constexpr uint32_t MOTOR_CONTROL = 20;   // 50 Hz
    constexpr uint32_t GRIPPER       = 50;
    constexpr uint32_t REFLECTANCE   = 20;   // 50 Hz, seguimiento de línea
    constexpr uint32_t COLOR_SENSOR  = 100;  // 10 Hz, de sobra para las zonas
    constexpr uint32_t LED_STATUS    = 250;
}

constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// Sin heartbeat por más de este tiempo, el supervisor da la tarea por colgada.
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;

// Si MotorTask pasa este tiempo sin un comando válido, frena por su cuenta.
constexpr uint32_t COMMS_FAILSAFE_TIMEOUT_MS = 500;

// Identificadores de tarea, usados como índice del arreglo de heartbeats.
// SUPERVISOR no está aquí a propósito: es quien vigila y no se vigila a sí
// mismo (de eso se encarga el Task Watchdog Timer del propio ESP-IDF).
enum class TaskId : uint8_t {
    SERIAL_COMM = 0,
    MOTOR_CONTROL,
    GRIPPER_CONTROL,
    COLOR_SENSOR,
    REFLECTANCE,
    LED_STATUS,
    COUNT   // siempre el último
};

// ---------------------------------------------------------------------------
//  Enlace con la Raspberry Pi.
//
//  Se usa el puerto USB NATIVO del ESP32-S3 (los GPIO 19/20), que aparece en
//  la Raspberry Pi como /dev/ttyACM0. Ventaja frente a usar el puerto UART:
//  deja libre el puerto de programación para depurar con un segundo cable
//  mientras el robot conversa con la Pi.
//
//  Requiere estos build_flags en platformio.ini (ya están puestos):
//      -D ARDUINO_USB_CDC_ON_BOOT=1
//      -D ARDUINO_USB_MODE=1
//
//  Con esos flags, `Serial` es el USB nativo y `Serial0` es el UART0 del
//  puerto de programación.
// ---------------------------------------------------------------------------
#define RPI_LINK   Serial     // datos hacia/desde la Raspberry Pi
#define DEBUG_LINK Serial0    // consola de depuración (puerto UART del DevKit)

// ===========================================================================
//  [2] TIPOS COMPARTIDOS ENTRE TAREAS
// ===========================================================================
//
//  Regla de diseño: no existe un "estado global del robot". Cada tarea es
//  dueña de sus datos y los publica por cola. Esto evita mutex compartidos y,
//  con ellos, la clase de bloqueos donde un subsistema lento congela a otro.

enum class TeamColor     : uint8_t { NONE = 0, RED = 1, BLUE = 2 };
enum class MotorMode     : uint8_t { STOP = 0, DRIVE = 1 };
// Con UN SOLO servo, el gripper solo sabe hacer tres cosas. CLOSE está
// separado en dos porque la llave (cubo) y el asta cilíndrica de la bandera
// tienen grosores distintos y necesitan ángulos de cierre distintos (ver
// kClawClosedLlaveDeg/kClawClosedBanderaDeg más abajo, en GripperTask).
enum class GripperAction : uint8_t { OPEN = 0, CLOSE_LLAVE = 1, CLOSE_BANDERA = 2 };
enum class ColorLabel    : uint8_t { UNKNOWN = 0, BLACK, YELLOW, RED, BLUE, FLOOR };

struct MotorCommand {
    MotorMode mode = MotorMode::STOP;
    int8_t    left  = 0;   // -100..100 (%), lado izquierdo (FL+RL)
    int8_t    right = 0;   // -100..100 (%), lado derecho  (FR+RR)
};

struct GripperCommand {
    GripperAction action = GripperAction::OPEN;
};

struct LedCommand {
    TeamColor team = TeamColor::NONE;
};

// La cámara de la Raspberry Pi es quien ve la bandera del equipo contrario
// (raspberry-pi/src/athena/ei_flag_detector.py); el ESP32 no tiene forma de
// verla por su cuenta. Este comando es solo la señal de "la estoy viendo
// AHORA MISMO", para que el LED de equipo la anuncie mientras el robot sigue
// recorriendo la pista — es la parte de la demo que exige "señalizar su
// detección", separada de LedCommand porque el color de equipo no cambia
// mientras que esta señal sí, cuadro a cuadro.
struct FlagSignalCommand {
    bool detected = false;
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

struct TeamSwitchReading {
    uint32_t  timestamp_ms = 0;
    TeamColor team         = TeamColor::NONE;
};

struct HealthReport {
    uint32_t timestamp_ms          = 0;
    uint8_t  faulted_tasks_bitmask = 0;  // bit i = TaskId i encontrada colgada
};

// ===========================================================================
//  [3] PROTOCOLO SERIAL CON LA RASPBERRY PI
// ===========================================================================
//
//  Trama:  [0xAA][TYPE][LEN][PAYLOAD ... LEN bytes][CHECKSUM]
//
//    TYPE     : 1 byte, ver PacketType.
//    LEN      : 1 byte, tamaño del payload.
//    CHECKSUM : 1 byte, XOR de TYPE, LEN y todos los bytes del payload.
//
//  DECISIÓN IMPORTANTE: el payload se arma y se lee BYTE A BYTE, en
//  little-endian, nunca con memcpy de un struct. Mandar structs crudos por el
//  cable es una trampa clásica: el compilador inserta padding invisible entre
//  campos, y ese padding no tiene por qué coincidir con lo que Python asuma
//  del otro lado. Serializando a mano, el contrato queda fijo y verificable.
//  Los booleanos viajan empacados en bits por la misma razón (sizeof(bool) no
//  está garantizado).
//
//  ---------------------------------------------------------------------
//   RPi -> ESP32
//  ---------------------------------------------------------------------
//   0x01 CMD_MOTOR    len=3   [0]=mode(0=STOP,1=DRIVE) [1]=left i8 [2]=right i8
//   0x02 CMD_GRIPPER  len=1   [0]=action(0=OPEN,1=CLOSE_LLAVE,2=CLOSE_BANDERA)
//   0x03 CMD_LED      len=1   [0]=team(0=NONE,1=RED,2=BLUE)
//   0x04 CMD_FLAG_SIGNAL len=1  [0]=detected(0=no, 1=sí, la cámara ve la bandera contraria)
//
//  ---------------------------------------------------------------------
//   ESP32 -> RPi
//  ---------------------------------------------------------------------
//   0x10 TLM_COLOR    len=7   [0..3]=timestamp_ms u32 LE
//                             [4]=front_label  [5]=back_label
//                             [6]=flags: bit0=front_valid, bit1=back_valid
//   0x11 TLM_REFLECT  len=9   [0..3]=timestamp_ms u32 LE
//                             [4..5]=left_raw u16 LE  [6..7]=right_raw u16 LE
//                             [8]=flags: bit0=left_on_line, bit1=right_on_line
//   0x12 TLM_HEALTH   len=5   [0..3]=timestamp_ms u32 LE
//                             [4]=faulted_tasks_bitmask
//   0x14 TLM_TEAM_SWITCH len=5 [0..3]=timestamp_ms u32 LE
//                             [4]=team(0=NONE,1=RED,2=BLUE), leído del switch
//                             físico de 3 posiciones (Pins::TEAM_SWITCH_*),
//                             NONE = posición central (nadie ha elegido)
//
//  El lado Python vive en raspberry-pi/src/athena/protocol.py y DEBE
//  mantenerse sincronizado con este bloque. El test
//  raspberry-pi/tests/test_protocol.py lee ESTE archivo y compara los valores,
//  así que si alguien cambia un número aquí y no allá, el test falla.

namespace Proto {
    constexpr uint8_t START_BYTE  = 0xAA;
    constexpr uint8_t MAX_PAYLOAD = 32;

    enum PacketType : uint8_t {
        CMD_MOTOR       = 0x01,
        CMD_GRIPPER     = 0x02,
        CMD_LED         = 0x03,
        CMD_FLAG_SIGNAL = 0x04,
        TLM_COLOR   = 0x10,
        TLM_REFLECT = 0x11,
        TLM_HEALTH  = 0x12,
        TLM_TEAM_SWITCH = 0x14,
    };

    constexpr uint8_t LEN_CMD_MOTOR       = 3;
    constexpr uint8_t LEN_CMD_GRIPPER     = 1;
    constexpr uint8_t LEN_CMD_LED         = 1;
    constexpr uint8_t LEN_CMD_FLAG_SIGNAL = 1;
    constexpr uint8_t LEN_TLM_COLOR   = 7;
    constexpr uint8_t LEN_TLM_REFLECT = 9;
    constexpr uint8_t LEN_TLM_HEALTH  = 5;
    constexpr uint8_t LEN_TLM_TEAM_SWITCH = 5;

    // Tipos que el ESP32 puede RECIBIR. Solo comandos: la telemetria va en la
    // otra direccion. Validar el byte de tipo contra esta lista es lo que le
    // permite al parser resincronizar cuando llega basura por el cable: sin
    // esto, un 0xAA suelto justo antes de una trama valida se traga esa trama.
    inline bool IsReceivableType(uint8_t type) {
        return type == CMD_MOTOR || type == CMD_GRIPPER || type == CMD_LED ||
               type == CMD_FLAG_SIGNAL;
    }

    inline uint8_t Checksum(uint8_t type, uint8_t len, const uint8_t *payload) {
        uint8_t c = type ^ len;
        for (uint8_t i = 0; i < len; ++i) c ^= payload[i];
        return c;
    }

    inline void WriteU16LE(uint8_t *dst, uint16_t v) {
        dst[0] = (uint8_t)(v & 0xFF);
        dst[1] = (uint8_t)((v >> 8) & 0xFF);
    }

    inline void WriteU32LE(uint8_t *dst, uint32_t v) {
        dst[0] = (uint8_t)(v & 0xFF);
        dst[1] = (uint8_t)((v >> 8) & 0xFF);
        dst[2] = (uint8_t)((v >> 16) & 0xFF);
        dst[3] = (uint8_t)((v >> 24) & 0xFF);
    }
}

// ===========================================================================
//  [4] COLAS
// ===========================================================================
//
//   RPi --USB--> SerialTask --> motorCmdQueue      (len 1, overwrite) --> MotorTask
//                           --> gripperCmdQueue    (len 4, FIFO)      --> GripperTask
//                           --> ledCmdQueue        (len 1, overwrite) --> LedTask
//                           --> flagSignalCmdQueue (len 1, overwrite) --> LedTask
//
//   ColorSensorTask --> colorQueue   (len 4, FIFO)      --> SerialTask --USB--> RPi
//   ReflectanceTask --> reflectQueue (len 4, FIFO)      --> SerialTask --USB--> RPi
//   SupervisorTask  --> healthQueue  (len 1, overwrite) --> SerialTask --USB--> RPi
//   LedTask         --> teamSwitchQueue (len 1, overwrite) --> SerialTask --USB--> RPi
//
//  Las colas "overwrite" (largo 1) son para datos donde solo importa el valor
//  más reciente: un comando de motores viejo no debe ejecutarse nunca. Las
//  FIFO son para eventos que deben procesarse todos y en orden (el gripper
//  tiene que abrir ANTES de bajar, no al revés).
//
//  Regla que se respeta en todo el archivo: ninguna tarea usa portMAX_DELAY
//  dentro de su bucle principal. Todas las operaciones de cola llevan timeout
//  0, de forma que un productor lento jamás cuelgue al consumidor.

static QueueHandle_t g_motorCmdQueue      = nullptr;
static QueueHandle_t g_gripperCmdQueue    = nullptr;
static QueueHandle_t g_ledCmdQueue        = nullptr;
static QueueHandle_t g_flagSignalCmdQueue = nullptr;
static QueueHandle_t g_colorQueue         = nullptr;
static QueueHandle_t g_reflectQueue       = nullptr;
static QueueHandle_t g_healthQueue        = nullptr;
static QueueHandle_t g_teamSwitchQueue    = nullptr;

// Última lectura del switch físico de equipo, publicada por LedTask en cada
// vuelta. MotorTask la lee para negarse a mover el robot mientras nadie haya
// elegido equipo (posición 0/central) — igual que ya se niega a moverse si
// el enlace con la Raspberry Pi se cae (COMMS_FAILSAFE_TIMEOUT_MS): es un
// segundo freno de seguridad, independiente del primero, y no depende de que
// el software de la Raspberry Pi haga lo correcto. Un uint8_t/enum de un
// byte se lee y escribe atómicamente en el ESP32-S3, así que no hace falta
// mutex para esto (no es el "estado global" que evita el resto del diseño:
// es una sola bandera de solo lectura para todos menos LedTask).
static volatile TeamColor g_switchTeam = TeamColor::NONE;

// ---------------------------------------------------------------------------
//  Mutex del bus I2C nº1 (Wire1) — TCS34725 trasero + PCA9685 (servos).
// ---------------------------------------------------------------------------
//  Único bus I2C de este firmware: el bus nº0 (TCS34725 delantero + ToF) se
//  retiró por completo, ver la nota grande al principio del archivo.
//
//  GripperTask (el PCA9685) corre en el núcleo 1, ColorSensorTask (el
//  TCS34725 trasero) en el núcleo 0 — con dos núcleos de por medio, pueden
//  estar ejecutando una transacción I2C cada una AL MISMO TIEMPO sobre el
//  mismo objeto `Wire1`. Este mutex serializa toda transacción sobre el bus:
//  cada tarea lo toma antes de tocarlo y lo suelta apenas termina.
//
//  Con timeout corto (no portMAX_DELAY, por la misma regla que las colas): si
//  no se consigue el bus en I2C1_LOCK_TIMEOUT_MS, la operación se da por
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
//  [5] WATCHDOG COOPERATIVO
// ===========================================================================
//
//  Cada tarea marca su heartbeat en cada vuelta de su bucle. El supervisor
//  revisa quién dejó de marcar.
//
//  Cada tarea escribe SOLO su propia celda del arreglo y el supervisor solo
//  lee, así que no hace falta mutex: no hay dos escritores para la misma
//  celda, y un uint32_t se escribe de forma atómica en esta arquitectura, de
//  modo que el supervisor nunca puede leer un valor a medias.

static volatile uint32_t g_lastHeartbeatMs[(size_t)TaskId::COUNT];

static void WatchdogInit() {
    const uint32_t now = millis();
    for (size_t i = 0; i < (size_t)TaskId::COUNT; ++i) g_lastHeartbeatMs[i] = now;
}

static inline void Heartbeat(TaskId id) {
    g_lastHeartbeatMs[(size_t)id] = millis();
}

// Devuelve un bitmask con las tareas que llevan demasiado tiempo sin marcar.
static uint8_t WatchdogCheck() {
    const uint32_t now = millis();
    uint8_t faulted = 0;
    for (size_t i = 0; i < (size_t)TaskId::COUNT; ++i) {
        // Resta sin signo: se comporta bien aunque millis() desborde (49 días).
        if ((uint32_t)(now - g_lastHeartbeatMs[i]) > WATCHDOG_TIMEOUT_MS) {
            faulted |= (uint8_t)(1u << i);
        }
    }
    return faulted;
}

// ===========================================================================
//  [6] DRIVER PCA9685 — servos del gripper por I2C
// ===========================================================================
//
//  El PCA9685 es un generador de PWM de 16 canales. Solo hay que configurarle
//  el prescaler una vez y luego escribir 4 bytes por canal. Escribirlo a mano
//  en vez de traer una librería nos cuesta unas 40 líneas y nos deja el
//  control fino del pulso, que es justo lo que hay que ajustar al calibrar
//  un gripper.
//
//  TODAS las funciones devuelven bool. Si el PCA9685 se desconecta, devuelven
//  false y la tarea del gripper sigue viva: un servo mudo no puede colgar al
//  robot entero.
//
//  Vive en Wire1 (bus I2C nº1), no en Wire — decisión del equipo para no
//  sumar un tercer dispositivo al bus 0 (ver Pins::I2C1_SDA/SCL y
//  g_i2c1Mutex). A diferencia de Tcs34725:: (que sí necesita recibir el bus
//  por parámetro porque el mismo driver atiende los dos TCS34725, uno en
//  cada bus), aquí hay un solo PCA9685 y siempre va a estar en el mismo
//  bus, así que no hace falta esa generalización.

namespace Pca9685 {
    constexpr uint8_t REG_MODE1    = 0x00;
    constexpr uint8_t REG_MODE2    = 0x01;
    constexpr uint8_t REG_LED0_ON_L = 0x06;
    constexpr uint8_t REG_PRESCALE = 0xFE;

    constexpr uint8_t MODE1_RESTART = 0x80;
    constexpr uint8_t MODE1_AI      = 0x20;   // auto-incremento de registro
    constexpr uint8_t MODE1_SLEEP   = 0x10;
    constexpr uint8_t MODE2_OUTDRV  = 0x04;   // salida totem-pole

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
        // El prescaler solo se puede tocar con el chip dormido.
        if (!WriteReg(REG_MODE1, MODE1_SLEEP)) return false;

        // Fórmula de la hoja de datos: oscilador interno de 25 MHz, 4096 pasos.
        const uint32_t prescale = (25000000UL / (4096UL * freq_hz)) - 1UL;
        if (!WriteReg(REG_PRESCALE, (uint8_t)prescale)) return false;

        // Despertar y habilitar el auto-incremento (para escribir 4 bytes seguidos).
        if (!WriteReg(REG_MODE1, MODE1_AI)) return false;
        delayMicroseconds(500);              // el oscilador tarda en estabilizarse
        if (!WriteReg(REG_MODE1, MODE1_AI | MODE1_RESTART)) return false;
        if (!WriteReg(REG_MODE2, MODE2_OUTDRV)) return false;

        uint8_t check = 0;
        return ReadReg(REG_MODE1, check);    // confirma que sigue respondiendo
    }

    // ticks: 0..4095, ancho del pulso dentro del periodo de 20 ms.
    bool SetChannel(uint8_t channel, uint16_t ticks) {
        if (channel > 15) return false;
        if (ticks > 4095) ticks = 4095;

        Wire1.beginTransmission(I2CAddr::PCA9685);
        Wire1.write(REG_LED0_ON_L + 4 * channel);
        Wire1.write(0x00);                      // ON  low   -> el pulso empieza en 0
        Wire1.write(0x00);                      // ON  high
        Wire1.write((uint8_t)(ticks & 0xFF));   // OFF low
        Wire1.write((uint8_t)(ticks >> 8));     // OFF high
        return Wire1.endTransmission() == 0;
    }
}

// Convierte un ángulo 0..180 al número de ticks del PCA9685.
static uint16_t ServoAngleToTicks(int angle_deg) {
    angle_deg = constrain(angle_deg, 0, 180);
    return (uint16_t)(Pwm::SERVO_TICK_MIN +
        ((uint32_t)(Pwm::SERVO_TICK_MAX - Pwm::SERVO_TICK_MIN) * (uint32_t)angle_deg) / 180UL);
}

// ===========================================================================
//  [7] DRIVER TCS34725 — sensores de color por I2C
// ===========================================================================
//
//  Los dos sensores son idénticos y comparten la dirección 0x29, que es fija.
//  Por eso cada uno vive en su propio bus (Wire y Wire1) y este driver recibe
//  el bus como parámetro.

namespace Tcs34725 {
    constexpr uint8_t CMD_BIT   = 0x80;   // todo acceso a registro lleva este bit
    constexpr uint8_t CMD_AUTO_INC = 0x20;

    constexpr uint8_t REG_ENABLE  = 0x00;
    constexpr uint8_t REG_ATIME   = 0x01;
    constexpr uint8_t REG_CONTROL = 0x0F;
    constexpr uint8_t REG_ID      = 0x12;
    constexpr uint8_t REG_CDATAL  = 0x14;   // luego R, G, B consecutivos

    constexpr uint8_t ENABLE_PON = 0x01;    // enciende el oscilador interno
    constexpr uint8_t ENABLE_AEN = 0x02;    // habilita el conversor RGBC

    // Tiempo de integración. 0xEB = 24 ms: suficientemente rápido para leer a
    // 10 Hz y suficientemente largo para no quedarse corto de luz.
    constexpr uint8_t ATIME_24MS = 0xEB;
    // Ganancia: 0=1x, 1=4x, 2=16x, 3=60x. A pocos milímetros del piso, 4x sobra.
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

// ---------------------------------------------------------------------------
//  Clasificación de color -> etiqueta de la pista.
//
//  Se normaliza cada canal contra el canal "clear" (luz total). Así la
//  decisión deja de depender del brillo absoluto, que cambia muchísimo con la
//  luz del salón, y pasa a depender solo del tono.
//
//  RECALIBRADO 2026-08-28 contra 970 muestras reales del TCS34725
//  DELANTERO (calibracion/color/data_logs/, generadas con
//  calibracion/color/calibrar_color.py) usando
//  calibracion/color/analizar_umbrales_tcs.py — descenso de coordenadas
//  sobre esta MISMA estructura de reglas. Error total 14.3% sobre esas 970
//  muestras (era 58.1% con los umbrales anteriores, sin calibrar). El
//  detalle completo —incluida la matriz de confusión por color— vive en
//  calibracion/color/detector-tcs/src/main.cpp, que usa exactamente estos
//  mismos números: si se recalibra, actualizar LOS DOS archivos o quedan
//  desincronizados (pasó antes con el umbral IR, commit c4f9b47).
//
//  ⚠️ NEGRO es la clase que peor acierta (58.5%, confundido sobre todo con
//  AZUL y GRIS): el rango de `clear` de NEGRO se solapa fuerte con el de
//  AZUL y GRIS, y esta clasificación secuencial por umbrales no separa bien
//  "oscuro" de "oscuro y además azul/gris". Es un límite estructural, no
//  algo que otra vuelta de ajuste vaya a arreglar — el clasificador K-NN de
//  pruebas-platformio/05-evitador-linea/ (mismo dataset, 4 dimensiones
//  juntas con distancia real en vez de reglas con AND encadenados) le
//  acierta bastante mejor a NEGRO. Si NEGRO/BLACK necesita ser confiable
//  para la lógica de vuelo, vale la pena portar ese enfoque acá en vez de
//  seguir afinando estos 9 umbrales.
//
//  ⚠️ EL SENSOR DELANTERO SE RETIRÓ (bus I2C nº0, ver el aviso grande al
//  principio del archivo): el TRASERO es ahora el ÚNICO sensor de color
//  del robot, pero estos umbrales solo se probaron contra datos del
//  DELANTERO — sigue siendo, sin verificar, el mismo supuesto que ya
//  advertía esta nota antes de que el delantero se quitara (ver
//  calibracion/color/README.md). Antes solo degradaba una lectura
//  secundaria; ahora TODA la detección de zona (BUSCAR_ZONA_NEUTRA,
//  RETORNAR_A_ZONA en decision.py) depende de que este juego de umbrales
//  también sirva para el trasero. Recalibrar contra capturas reales del
//  trasero en cuanto se pueda.
// ---------------------------------------------------------------------------
static ColorLabel ClassifyColor(const Tcs34725::Rgbc &s) {
    // Muy poca luz reflejada = cinta negra (o el sensor mirando al vacío).
    if (s.c < 392) return ColorLabel::BLACK;

    const float total = (float)s.c;
    const float r = (float)s.r / total;
    const float g = (float)s.g / total;
    const float b = (float)s.b / total;

    if (r > 0.450f && g < 0.312f && b < 0.300f) return ColorLabel::RED;
    if (b > 0.216f && r < 0.390f)               return ColorLabel::BLUE;
    if (r > 0.416f && g > 0.350f && b < 0.250f) return ColorLabel::YELLOW;

    // Canales parejos = superficie gris: el tapete de la pista.
    return ColorLabel::FLOOR;
}

// ===========================================================================
//  [8] TAREAS
// ===========================================================================

namespace {

// ---------------------------------------------------------------------------
//  8.1  MotorTask — 4 motores a través de 2 drivers L298N
// ---------------------------------------------------------------------------
//  Es la tarea más crítica para la seguridad: si el enlace con la Raspberry Pi
//  se cae, ésta frena por su cuenta sin esperar a que nadie se lo ordene.
//
//  RECORDATORIO DE MONTAJE: hay que QUITAR los jumpers de ENA y ENB en ambos
//  L298N. Si se dejan puestos, el enable queda fijo a 5 V y el PWM no hace
//  absolutamente nada: los motores girarán siempre a full o nada.

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

// La API de LEDC cambió entre el core 2.x y el 3.x de Arduino-ESP32. Estas
// dos funciones absorben la diferencia para que el firmware compile en ambos.
// Reciben freq/resolución como parámetro porque no todos los canales PWM del
// robot usan los mismos: los motores van a 1 kHz/8 bits, el LED RGB a 5 kHz.
void PwmAttach(uint8_t pin, uint8_t channel, uint32_t freqHz, uint8_t resolution) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;                       // en 3.x el canal se asigna solo
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

// speed: -100..100. El signo define el sentido, la magnitud el duty.
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

    MotorCommand current{};                  // por defecto: STOP
    uint32_t last_cmd_ms = millis();

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::MOTOR_CONTROL);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        MotorCommand incoming;
        if (xQueueReceive(g_motorCmdQueue, &incoming, 0) == pdTRUE) {
            current = incoming;
            last_cmd_ms = millis();
        }

        // Failsafe propio: no depende de que otra tarea lo detecte por nosotros.
        const bool comms_stale =
            (uint32_t)(millis() - last_cmd_ms) > COMMS_FAILSAFE_TIMEOUT_MS;
        // Segundo failsafe, independiente del primero: mientras el switch
        // físico de equipo siga en la posición central (nadie lo ha movido
        // todavía), el robot se queda quieto pase lo que mande la Raspberry
        // Pi. Ver g_switchTeam.
        const bool team_undecided = (g_switchTeam == TeamColor::NONE);

        if (comms_stale || team_undecided || current.mode == MotorMode::STOP) {
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
//  8.2  GripperTask — 1 servo vía PCA9685
// ---------------------------------------------------------------------------
//  El robot tiene UN SOLO servo de gripper: agarra o suelta. No sube ni baja
//  nada. Los ángulos están calibrados a mano con pruebas-platformio/
//  06-calibracion-gripper/ (ver su README) — el mismo "abierta" sirve para
//  los dos objetos, pero "cerrada" NO: la llave (cubo) y el asta cilíndrica
//  de la bandera tienen grosores distintos, de ahí los dos closed distintos
//  y por qué GripperAction no tiene un CLOSE genérico.

constexpr int kClawOpenDeg          = 0;
constexpr int kClawClosedLlaveDeg   = 128;
constexpr int kClawClosedBanderaDeg = 65;

void GripperTask(void *) {
    // El bus I2C ya fue inicializado en setup(); aquí solo se configura el
    // chip. El PCA9685 vive en el bus 1 (Wire1), junto con el TCS34725
    // trasero: se toma el mutex de ESE bus antes y se suelta al terminar
    // (ver la nota larga junto a g_i2c1Mutex).
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
        // Si el PCA9685 no arrancó (cable flojo, alimentación ausente), se
        // reintenta cada segundo sin bloquear nada. El robot puede seguir
        // moviéndose y compitiendo mientras tanto.
        if (!pca_ok && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            if (I2c1Lock()) {
                pca_ok = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
                // Si el PCA9685 no respondió al arrancar (arriba se saltó el
                // SetChannel inicial) y recién ahora reaparece, hay que
                // mandar la pinza a 0° también acá: si no, se queda esperando
                // un GripperCommand que puede tardar en llegar.
                if (pca_ok) {
                    pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
                }
                I2c1Unlock();
            }
        }

        GripperCommand cmd;
        if (xQueueReceive(g_gripperCmdQueue, &cmd, 0) == pdTRUE && pca_ok) {
            // Si el mecanismo se traba, el servo simplemente no llega al
            // ángulo pedido. Esta tarea no se queda esperándolo: manda la
            // orden y sigue, así que un gripper atascado nunca congela a los
            // motores ni al enlace serial.
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

// Empuja un elemento a una cola FIFO descartando el más viejo si está llena.
// Preferimos telemetría fresca a telemetría completa: un dato de sensor de
// hace medio segundo no le sirve de nada a la Raspberry Pi.
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

    if (!back_ok)  DEBUG_LINK.println("[Color] sensor TRASERO no responde (bus I2C 1).");

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::COLOR_SENSOR);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        // Reintento perezoso del sensor caído, sin bloquear el resto.
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
        // Se distingue "no se consiguió el bus a tiempo" de "el sensor no
        // respondió". Lo primero es normal bajo contención y NO debe marcar
        // el sensor como caído: simplemente no hay lectura esta vuelta. Solo
        // un Read() que falla de verdad lo manda a reintento.
        bool back_read = false;
        bool back_intentado = false;
        if (back_ok && I2c1Lock()) {
            back_intentado = true;
            back_read = Tcs34725::Read(Wire1, sample);
            I2c1Unlock();
        }
        if (back_read) {
            reading.back = ClassifyColor(sample);
            reading.back_valid = true;
        } else if (back_intentado) {
            back_ok = false;
        }

        PushDropOldest(g_colorQueue, reading);

        Heartbeat(TaskId::COLOR_SENSOR);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  8.4  ReflectanceTask — 2x QTRX-HD-01A, con rechazo de luz ambiente
// ---------------------------------------------------------------------------
//
//  CALIBRADO EN BANCO (robot físicamente quieto sobre cada superficie —
//  ver el historial de commits de firmware-esp32-standalone/, donde se
//  diagnosticó primero) comparando una lectura con el emisor IR prendido
//  contra una con el emisor apagado de verdad:
//
//    restado = (ADC con emisor prendido) - (ADC con emisor apagado)
//
//  Con el sensor DERECHO (el único que funciona -- ver la nota sobre el
//  izquierdo más abajo):
//    negro (absorbe casi toda la luz del emisor): |restado| ~= 1 a 8
//    gris/pista (rebota mucho más):               |restado| ~= 76 a 87
//
//  |restado| CHICO = negro, GRANDE = superficie clara. Comparar el ADC
//  crudo (con el emisor siempre prendido) contra un umbral fijo NO
//  funciona: esa lectura mezcla el reflejo del piso CON cualquier luz
//  ambiente que le llegue al fototransistor, y con suficiente luz ambiente
//  esa componente domina y aplana la diferencia entre negro y gris. Restar
//  una lectura con el emisor apagado aísla la parte que de verdad depende
//  del color del piso.
//
//  IMPORTANTE sobre el pulso de apagado: según el datasheet del QTRX-HD,
//  un LOW de 0.5-300 us en CTRL NO apaga el emisor -- lo interpreta como
//  un PULSO DE ATENUACIÓN (baja un escalón de 32 en el brillo, ~3.33%) y
//  lo deja prendido. Solo un LOW sostenido >= 1 ms apaga los LEDs de
//  verdad; ese mismo LOW seguido de HIGH reinicia el emisor a corriente
//  completa (100%), así que no hace falta re-sincronizar ningún estado de
//  atenuación entre lecturas.
//
//  ¡ALIMENTAR EL QTRX A 3.3 V! Su salida es proporcional a su VIN: alimentado
//  a 5 V entregaría hasta 5 V en un pin de ADC que solo tolera 3.3 V.

// Umbral sobre |restado|: a la mitad entre el peor caso de negro (~8) y
// el mejor caso de gris (~76), con margen generoso a los dos lados.
constexpr int16_t kBordeRestadoUmbral = 40;

void ReflectanceTask(void *) {
    analogReadResolution(12);

    // Atenuación de 11 dB: extiende el rango útil del ADC a ~0..3.1 V, que es
    // lo que necesitamos para leer la excursión completa del QTR a 3.3 V.
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
        reading.timestamp_ms = millis();
        reading.left_raw     = left_raw;
        reading.right_raw    = right_raw;

        const int16_t right_restado = (int16_t)((int32_t)right_raw - (int32_t)right_ambiente);
        reading.right_on_line = abs(right_restado) < kBordeRestadoUmbral;

        // IZQUIERDO: sensor confirmado roto en banco -- pegado en 4095 sin
        // importar superficie NI estado del emisor (restado ronda 0
        // constante, que bajo este criterio se leería como "negro"). Si se
        // dejara participar, el lado izquierdo dispararía la evasión de
        // borde todo el tiempo. Forzado en false hasta reparar el
        // hardware; left_raw se sigue reportando (vía TLM_REFLECT) para
        // diagnóstico desde la Raspberry Pi.
        reading.left_on_line = false;

        PushDropOldest(g_reflectQueue, reading);

        Heartbeat(TaskId::REFLECTANCE);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  8.5  LedTask — LED RGB indicador de equipo (lo exige el reglamento)
// ---------------------------------------------------------------------------
//  Hasta hace poco esto eran 2 LED discretos (rojo/azul) con digitalWrite.
//  El equipo los reemplazó por un único LED RGB (validado primero en
//  pruebas-platformio/02-cuadro-color-rgb/), así que ahora se maneja por PWM.
//  Queda con margen para lo que haga falta más adelante: cualquier otra señal
//  visual se resuelve mezclando los 3 canales, sin cablear nada nuevo.

namespace RgbLed {
    // Canales LEDC 4..6: los motores ya ocupan 0..3 (ver 8.1).
    constexpr uint8_t CH_R = 4;
    constexpr uint8_t CH_G = 5;
    constexpr uint8_t CH_B = 6;

    // POLARIDAD CONFIRMADA con el LED físico: es CÁTODO COMÚN, o sea duty
    // alto = canal más brillante, que es el sentido natural del código. La
    // constante se conserva porque documenta esa decisión y deja la vuelta
    // preparada si algún día se cambia el LED por uno de ánodo común.
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

    void ApplyTeam(TeamColor team) {
        switch (team) {
            case TeamColor::RED:  SetRaw(255, 0, 0); break;
            case TeamColor::BLUE: SetRaw(0, 0, 255); break;
            case TeamColor::NONE:
            default:              SetRaw(0, 0, 0);   break;
        }
    }
}

// ---------------------------------------------------------------------------
//  Switch de 3 posiciones (ON-OFF-ON): quién es "nuestro equipo" al arrancar.
// ---------------------------------------------------------------------------
//  Ver Pins::TEAM_SWITCH_BLUE / TEAM_SWITCH_RED y hardware/conexiones-esp32-
//  s3.md. Se lee en cada vuelta de LedTask (250 ms) en vez de una sola vez en
//  setup(): así, si alguien mueve el switch después de energizar el robot
//  (el procedimiento normal: encender con el switch en 0 y recién ahí elegir
//  equipo), el cambio se recoge sin tener que reiniciar el ESP32.
namespace TeamSwitch {
    void Setup() {
        pinMode(Pins::TEAM_SWITCH_BLUE, INPUT_PULLUP);
        pinMode(Pins::TEAM_SWITCH_RED,  INPUT_PULLUP);
    }

    // LOW = tiro cerrado a GND = esa posición está seleccionada. Si algún día
    // el switch fallara y los 2 tiros leyeran cerrados a la vez (no debería
    // pasar con un ON-OFF-ON sano), se trata igual que la posición central:
    // más seguro reportar "nadie ha elegido" que adivinar un color.
    TeamColor Read() {
        const bool blue_closed = digitalRead(Pins::TEAM_SWITCH_BLUE) == LOW;
        const bool red_closed  = digitalRead(Pins::TEAM_SWITCH_RED)  == LOW;
        if (blue_closed && !red_closed) return TeamColor::BLUE;
        if (red_closed  && !blue_closed) return TeamColor::RED;
        return TeamColor::NONE;
    }
}

// Cuántas vueltas de LedTask dura cada mitad del destello (encendido/apagado)
// cuando se está señalizando la bandera contraria. A TaskPeriodMs::LED_STATUS
// (250 ms) esto da un destello de ~2 Hz: rápido de notar a simple vista desde
// la mesa de jueces, sin parpadear tan rápido que parezca un LED fundido.
constexpr uint32_t kFlagBlinkHalfPeriodTicks = 1;

void LedTask(void *) {
    RgbLed::Setup();
    RgbLed::SetRaw(0, 0, 0);
    TeamSwitch::Setup();

    TeamColor commanded_team = TeamColor::NONE;   // último CMD_LED recibido
    bool flag_detected = false;
    uint32_t blink_tick = 0;

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::LED_STATUS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        LedCommand cmd;
        if (xQueueReceive(g_ledCmdQueue, &cmd, 0) == pdTRUE) commanded_team = cmd.team;

        // El switch físico es la fuente de verdad: si alguien lo movió de la
        // posición central, manda sobre lo que haya dicho la Raspberry Pi por
        // CMD_LED. Si sigue en el centro (NONE, el reposo normal al
        // encender), se respeta el último CMD_LED — así el LED sigue
        // funcionando en banco sin el switch instalado (por ejemplo en
        // pruebas-platformio/02-cuadro-color-rgb/).
        const TeamColor switch_team = TeamSwitch::Read();
        g_switchTeam = switch_team;   // MotorTask la lee para no moverse en NONE

        TeamSwitchReading switch_reading;
        switch_reading.timestamp_ms = millis();
        switch_reading.team = switch_team;
        xQueueOverwrite(g_teamSwitchQueue, &switch_reading);

        const TeamColor team = (switch_team != TeamColor::NONE) ? switch_team : commanded_team;

        // Cola "overwrite": solo importa si la cámara ve la bandera AHORA, no
        // un fotograma viejo. Si la Raspberry Pi deja de mandar esta señal
        // (se cae el enlace, por ejemplo), el LED simplemente vuelve a quedar
        // en el color de equipo fijo — no hay estado "pegado" en destello.
        FlagSignalCommand flag_cmd;
        if (xQueueReceive(g_flagSignalCmdQueue, &flag_cmd, 0) == pdTRUE) {
            flag_detected = flag_cmd.detected;
        }

        if (flag_detected) {
            // Alterna entre BLANCO (destello) y el color de equipo: sigue
            // identificando al equipo ante los jueces mientras además avisa
            // que ve la bandera contraria — ninguna señal tapa a la otra.
            blink_tick = (blink_tick + 1) % (2 * kFlagBlinkHalfPeriodTicks);
            if (blink_tick < kFlagBlinkHalfPeriodTicks) {
                RgbLed::SetRaw(255, 255, 255);
            } else {
                RgbLed::ApplyTeam(team);
            }
        } else {
            blink_tick = 0;
            RgbLed::ApplyTeam(team);
        }

        Heartbeat(TaskId::LED_STATUS);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  8.6  SerialTask — único punto de contacto con la Raspberry Pi
// ---------------------------------------------------------------------------
//  Es la ÚNICA tarea que toca el enlace con la RPi. Ninguna otra escribe en
//  él, así que no hace falta un mutex para el puerto.
//
//  El receptor es una máquina de estados que consume byte a byte. Nunca
//  bloquea esperando el resto de una trama: si el paquete llega partido en
//  varios pedazos (cosa normal), simplemente retoma donde quedó en la vuelta
//  siguiente. Cualquier byte suelto o checksum malo lo devuelve a WAIT_START,
//  de modo que una trama corrupta no puede trabar el enlace.

enum class RxState : uint8_t { WAIT_START, READ_TYPE, READ_LEN, READ_PAYLOAD, READ_CHECKSUM };

void SerialSendPacket(uint8_t type, const uint8_t *payload, uint8_t len) {
    if (len > Proto::MAX_PAYLOAD) return;

    // 4 bytes de sobrecarga: START, TYPE, LEN y CHECKSUM.
    uint8_t buf[4 + Proto::MAX_PAYLOAD];
    buf[0] = Proto::START_BYTE;
    buf[1] = type;
    buf[2] = len;
    memcpy(&buf[3], payload, len);
    buf[3 + len] = Proto::Checksum(type, len, payload);
    RPI_LINK.write(buf, (size_t)(4 + len));
}

// Despacha un paquete ya validado hacia la cola que le toca.
void SerialDispatch(uint8_t type, const uint8_t *payload, uint8_t len) {
    switch (type) {
        case Proto::CMD_MOTOR: {
            if (len != Proto::LEN_CMD_MOTOR) return;
            MotorCommand cmd;
            cmd.mode  = (payload[0] == 1) ? MotorMode::DRIVE : MotorMode::STOP;
            cmd.left  = (int8_t)payload[1];
            cmd.right = (int8_t)payload[2];
            xQueueOverwrite(g_motorCmdQueue, &cmd);
            break;
        }
        case Proto::CMD_GRIPPER: {
            if (len != Proto::LEN_CMD_GRIPPER) return;
            if (payload[0] > (uint8_t)GripperAction::CLOSE_BANDERA) return;
            GripperCommand cmd;
            cmd.action = (GripperAction)payload[0];
            xQueueSend(g_gripperCmdQueue, &cmd, 0);
            break;
        }
        case Proto::CMD_LED: {
            if (len != Proto::LEN_CMD_LED) return;
            if (payload[0] > (uint8_t)TeamColor::BLUE) return;
            LedCommand cmd;
            cmd.team = (TeamColor)payload[0];
            xQueueOverwrite(g_ledCmdQueue, &cmd);
            break;
        }
        case Proto::CMD_FLAG_SIGNAL: {
            if (len != Proto::LEN_CMD_FLAG_SIGNAL) return;
            FlagSignalCommand cmd;
            cmd.detected = (payload[0] != 0);
            xQueueOverwrite(g_flagSignalCmdQueue, &cmd);
            break;
        }
        default:
            break;   // tipo desconocido: se ignora, el enlace sigue vivo
    }
}

void SerialTask(void *) {
    RxState state = RxState::WAIT_START;
    uint8_t rx_type = 0, rx_len = 0, rx_index = 0;
    uint8_t rx_payload[Proto::MAX_PAYLOAD];

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::SERIAL_COMM);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // --- Recepción: consume lo que haya llegado, sin bloquear nunca ---
        // El tope por vuelta evita que una ráfaga de basura acapare la CPU.
        int budget = 256;
        while (RPI_LINK.available() > 0 && budget-- > 0) {
            const uint8_t b = (uint8_t)RPI_LINK.read();
            switch (state) {
                case RxState::WAIT_START:
                    if (b == Proto::START_BYTE) state = RxState::READ_TYPE;
                    break;

                case RxState::READ_TYPE:
                    if (Proto::IsReceivableType(b)) {
                        rx_type = b;
                        state = RxState::READ_LEN;
                    } else if (b != Proto::START_BYTE) {
                        state = RxState::WAIT_START;
                    }
                    // Si b ES el byte de inicio, nos quedamos en READ_TYPE:
                    // era un 0xAA de relleno y el tipo real viene ahora.
                    break;

                case RxState::READ_LEN:
                    rx_len = b;
                    if (rx_len > Proto::MAX_PAYLOAD) {
                        state = RxState::WAIT_START;   // largo imposible: resincronizar
                    } else {
                        rx_index = 0;
                        state = (rx_len == 0) ? RxState::READ_CHECKSUM : RxState::READ_PAYLOAD;
                    }
                    break;

                case RxState::READ_PAYLOAD:
                    rx_payload[rx_index++] = b;
                    if (rx_index >= rx_len) state = RxState::READ_CHECKSUM;
                    break;

                case RxState::READ_CHECKSUM:
                    if (b == Proto::Checksum(rx_type, rx_len, rx_payload)) {
                        SerialDispatch(rx_type, rx_payload, rx_len);
                    }
                    // Calce o no el checksum, siempre volvemos al inicio.
                    state = RxState::WAIT_START;
                    break;
            }
        }

        // --- Envío de telemetría ---
        uint8_t payload[Proto::MAX_PAYLOAD];

        ColorReading color;
        while (xQueueReceive(g_colorQueue, &color, 0) == pdTRUE) {
            Proto::WriteU32LE(&payload[0], color.timestamp_ms);
            payload[4] = (uint8_t)color.front;
            payload[5] = (uint8_t)color.back;
            payload[6] = (uint8_t)((color.front_valid ? 0x01 : 0) |
                                   (color.back_valid  ? 0x02 : 0));
            SerialSendPacket(Proto::TLM_COLOR, payload, Proto::LEN_TLM_COLOR);
        }

        ReflectanceReading reflect;
        while (xQueueReceive(g_reflectQueue, &reflect, 0) == pdTRUE) {
            Proto::WriteU32LE(&payload[0], reflect.timestamp_ms);
            Proto::WriteU16LE(&payload[4], reflect.left_raw);
            Proto::WriteU16LE(&payload[6], reflect.right_raw);
            payload[8] = (uint8_t)((reflect.left_on_line  ? 0x01 : 0) |
                                   (reflect.right_on_line ? 0x02 : 0));
            SerialSendPacket(Proto::TLM_REFLECT, payload, Proto::LEN_TLM_REFLECT);
        }

        HealthReport health;
        while (xQueueReceive(g_healthQueue, &health, 0) == pdTRUE) {
            Proto::WriteU32LE(&payload[0], health.timestamp_ms);
            payload[4] = health.faulted_tasks_bitmask;
            SerialSendPacket(Proto::TLM_HEALTH, payload, Proto::LEN_TLM_HEALTH);
        }

        TeamSwitchReading switch_reading;
        while (xQueueReceive(g_teamSwitchQueue, &switch_reading, 0) == pdTRUE) {
            Proto::WriteU32LE(&payload[0], switch_reading.timestamp_ms);
            payload[4] = (uint8_t)switch_reading.team;
            SerialSendPacket(Proto::TLM_TEAM_SWITCH, payload, Proto::LEN_TLM_TEAM_SWITCH);
        }

        Heartbeat(TaskId::SERIAL_COMM);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  8.7  SupervisorTask — vigila que las demás tareas sigan vivas
// ---------------------------------------------------------------------------
//
//  ESTRATEGIA DE TOLERANCIA A FALLOS
//
//  1. Cada tarea es dueña de su propio hardware y su propia cola. Un fallo al
//     leer el TCS34725 no puede bloquear la cola de los motores.
//  2. Ninguna tarea bloquea indefinidamente. Todo lleva timeout.
//  3. Los drivers I2C devuelven bool y las tareas reintentan en segundo plano
//     en vez de quedarse esperando a un chip que no contesta.
//  4. MotorTask tiene su propio failsafe si se cae el enlace con la RPi.
//  5. Cada tarea marca un heartbeat; esta tarea detecta a la que dejó de
//     marcar (colgada en un I2C que no responde, un bucle sin salida, etc.).
//
//  Cuando detecta una tarea colgada NO reinicia el ESP32: registra el fallo y
//  se lo reporta a la Raspberry Pi, que es donde vive la lógica de decisión.
//  Así la RPi puede, por ejemplo, dejar de confiar en el sensor de color y
//  seguir compitiendo, mientras el resto de tareas siguen normales.
//
//  LÍMITE CONOCIDO, dicho sin adornos: el ESP32-S3 no tiene MMU, así que todas
//  las tareas comparten el mismo espacio de memoria. Este diseño aísla tareas
//  COLGADAS, no memoria corrupta: un desbordamiento de stack severo en una
//  tarea sí puede dañar a otra. Por eso hay que medir los stacks reales con
//  uxTaskGetStackHighWaterMark() durante las pruebas, y por eso ninguna tarea
//  reserva memoria dinámica dentro de su bucle.

void SupervisorTask(void *) {
    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::SUPERVISOR);
    TickType_t last_wake = xTaskGetTickCount();

    uint8_t previous_faults = 0;

    for (;;) {
        const uint8_t faulted = WatchdogCheck();

        // Se manda cada vuelta, con el bitmask en 0 cuando todo está sano:
        // así el silencio total en TLM_HEALTH es una señal real de que el
        // ESP32 dejó de hablar (o el propio SupervisorTask se colgó), no un
        // caso ambiguo con "todo va bien".
        HealthReport report;
        report.timestamp_ms = millis();
        report.faulted_tasks_bitmask = faulted;
        xQueueOverwrite(g_healthQueue, &report);

        // Se registra por consola solo cuando el conjunto de fallos CAMBIA,
        // para no inundar el log con la misma línea 5 veces por segundo.
        if (faulted != previous_faults) {
            DEBUG_LINK.printf("[Supervisor] tareas colgadas: 0x%02X\n", faulted);
            previous_faults = faulted;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace anónimo

// ===========================================================================
//  [9] setup() / loop()
// ===========================================================================

static bool CreateQueues() {
    g_motorCmdQueue      = xQueueCreate(1, sizeof(MotorCommand));
    g_gripperCmdQueue    = xQueueCreate(4, sizeof(GripperCommand));
    g_ledCmdQueue        = xQueueCreate(1, sizeof(LedCommand));
    g_flagSignalCmdQueue = xQueueCreate(1, sizeof(FlagSignalCommand));
    g_colorQueue         = xQueueCreate(4, sizeof(ColorReading));
    g_reflectQueue       = xQueueCreate(4, sizeof(ReflectanceReading));
    g_healthQueue        = xQueueCreate(1, sizeof(HealthReport));
    g_teamSwitchQueue    = xQueueCreate(1, sizeof(TeamSwitchReading));

    return g_motorCmdQueue && g_gripperCmdQueue && g_ledCmdQueue && g_flagSignalCmdQueue &&
           g_colorQueue && g_reflectQueue && g_healthQueue && g_teamSwitchQueue;
}

// Por qué se reinició el ESP32 la última vez. Distinguir un BROWNOUT (batería
// o alimentación floja) de un PANIC (bug del firmware) o de un simple POWERON
// ahorra horas de depuración a ciegas: los síntomas en la pista son los
// mismos —el robot "se queda"— pero la causa y el arreglo no tienen nada que
// ver. Se diagnosticó primero en firmware-esp32-standalone/ (commit 2db53fb).
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
    DEBUG_LINK.begin(SERIAL_BAUD_RATE);   // consola por el puerto UART del DevKit
    RPI_LINK.begin(SERIAL_BAUD_RATE);     // enlace USB con la Raspberry Pi
    delay(200);
    DEBUG_LINK.println("\nAthena Rover 2026 - firmware ESP32-S3");
    DEBUG_LINK.printf("[Setup] Motivo del ultimo reinicio: %s\n",
                       ResetReasonToString(esp_reset_reason()));

    // Único bus I2C del firmware: se abre aquí, ANTES de lanzar las tareas,
    // así ninguna tarea tiene que inicializar hardware compartido por su
    // cuenta. El bus nº0 (TCS34725 delantero + VL53L1X) se retiró por
    // completo -- ver el aviso grande al principio del archivo.
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL, 400000);  // TCS34725 trasero + PCA9685

    // Timeout corto: si el chip I2C se cuelga tirando SDA a masa, la
    // transacción falla rápido en vez de congelar la tarea que la pidió.
    Wire1.setTimeOut(25);

    // Mutex del bus I2C, creado ANTES de ninguna tarea: las que lo
    // comparten lo toman en su primera vuelta (ver g_i2c1Mutex).
    g_i2c1Mutex = xSemaphoreCreateMutex();
    if (g_i2c1Mutex == nullptr) {
        DEBUG_LINK.println("[FATAL] no se pudo crear el mutex del bus I2C. Arranque detenido.");
        for (;;) delay(1000);
    }

    if (!CreateQueues()) {
        // Sin colas no hay comunicación entre tareas: arrancar a medias sería
        // peor que no arrancar, porque el robot se movería sin poder frenar.
        DEBUG_LINK.println("[FATAL] no se pudieron crear las colas. Arranque detenido.");
        for (;;) delay(1000);
    }

    // Los heartbeats se inicializan ANTES de crear ninguna tarea. Si se
    // hiciera después, se borrarían las marcas de las tareas que ya arrancaron
    // y el supervisor las vería como recién nacidas.
    WatchdogInit();

    xTaskCreatePinnedToCore(SupervisorTask, "Supervisor", TaskStack::SUPERVISOR,
                            nullptr, TaskPriority::SUPERVISOR, nullptr, 0);
    xTaskCreatePinnedToCore(SerialTask, "SerialComm", TaskStack::SERIAL_COMM,
                            nullptr, TaskPriority::SERIAL_COMM, nullptr, 1);
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

    DEBUG_LINK.println("Todas las tareas lanzadas.");
}

void loop() {
    // Todo el trabajo ocurre en las tareas. loop() se queda vacío a propósito.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
