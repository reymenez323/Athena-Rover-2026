// ===========================================================================
//  Prueba 06 — Calibración interactiva del servo del gripper
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  OBJETIVO: mover el servo del gripper un grado a la vez desde el monitor
//  serial, para encontrar a mano los ángulos de:
//    - Pinza ABIERTA / CERRADA sobre el CILINDRO.
//    - Pinza ABIERTA / CERRADA sobre la LLAVE (cubo).
//
//  El canal del PCA9685 donde está cableado el servo se asume el 0 (ver
//  hardware/conexiones-esp32-s3.md), pero si no estás seguro de cuál es en
//  tu cableado real, el comando "scan" prueba los 16 canales uno por uno e
//  imprime cuál está activo en cada momento — mira cuál mueve el servo
//  físico y quédate con ese número (comando "canal <n>").
//
//  Este sketch NO decide nada por su cuenta ni recuerda los ángulos entre
//  sesiones: cada vez que encuentres el ángulo correcto, el firmware lo
//  imprime por consola y TÚ lo anotas a mano. Esos números ya están
//  integrados en firmware-esp32/src/main.cpp (sección [8.2] GripperTask):
//  kClawOpenDeg=0, kClawClosedLlaveDeg=120, kClawClosedBanderaDeg=65. Si
//  vuelves a calibrar (otro gripper, otro objeto), actualiza esas constantes
//  allá también.
//
//  ⚠️ ANTES DE ENCENDER: igual que en 04-servos-a-cero, monta el gripper
//  DESACOPLADO del mecanismo la primera vez. Al arrancar, este sketch manda
//  el servo a 0° (mismo cero mecánico que 04-servos-a-cero), y si el cuerno
//  ya está atornillado contra un tope, ese salto inicial lo fuerza. Una vez
//  confirmado que 0° es un punto seguro, sí conviene calibrar con el
//  mecanismo acoplado — de hecho hace falta, para sentir cuándo la pinza
//  realmente agarra el objeto y no solo se acerca.
//
//  CÓMO SE USA — por el monitor serial (115200 baudios, línea + Enter):
//    abrir           pinza al ángulo de "abierta" ya calibrado
//    cerrar llave    pinza al ángulo de "cerrada" ya calibrado para la llave
//    cerrar bandera  pinza al ángulo de "cerrada" ya calibrado para la bandera
//    c <grados>      servo al ángulo absoluto indicado (0-180), en el canal
//                    actualmente seleccionado — para seguir afinando a mano
//    c+ / c-         nudge de +paso / -paso grados
//    canal <n>       cambia el canal del PCA9685 que controlan c/c+/c-/0 (0-15)
//    scan            prueba los 16 canales en orden, moviendo cada uno para
//                    que puedas ver a simple vista cuál es el que mueve tu servo
//    paso <n>        cambia el tamaño del nudge (arranca en 5°)
//    0               el servo de vuelta a 0° (reset de seguridad)
//    ?               reimprime el menú de ayuda
//
//  Cada comando imprime el estado completo después de aplicarse, así que el
//  número que hay que anotar siempre es el último que se ve en pantalla.
//
//  FLUJO SUGERIDO para cada objeto (cilindro, luego llave/cubo):
//    1. Con nudges pequeños (paso 2-3°), cierra "c" hasta que la pinza
//       encierre el objeto sin tocarlo todavía -> anota como "abierta".
//    2. Sigue cerrando de a poco hasta sentir que agarra firme SIN forzar
//       el servo contra el objeto (un servo forzado calienta y puede
//       perder torque con el tiempo) -> anota como "cerrada".
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>

// ===========================================================================
//  [1] PINES / DIRECCIÓN I2C — idénticos a hardware/conexiones-esp32-s3.md
// ===========================================================================

namespace Pins {
    constexpr uint8_t I2C0_SDA = 8;   // bus I2C nº0: PCA9685 + TCS34725 delantero
    constexpr uint8_t I2C0_SCL = 9;
}

namespace I2CAddr {
    constexpr uint8_t PCA9685 = 0x40;   // dirección por defecto (A0..A5 sin puentear)
}

// Canal del PCA9685 donde debería estar el servo del gripper, según
// hardware/conexiones-esp32-s3.md. Es solo el punto de partida: el comando
// "canal <n>" lo puede cambiar en tiempo de ejecución si "scan" revela que
// el cableado real usa otro.
namespace ServoChannel {
    constexpr uint8_t CLAW = 0;   // abre/cierra la pinza
}

constexpr uint8_t PCA9685_CHANNEL_COUNT = 16;

// Ángulos ya calibrados con este mismo sketch e integrados en
// firmware-esp32/src/main.cpp (GripperTask: kClawOpenDeg,
// kClawClosedLlaveDeg, kClawClosedBanderaDeg). Si vuelves a calibrar,
// actualiza los dos lados.
namespace Calibrado {
    constexpr int ABIERTO         = 0;
    constexpr int CERRADO_LLAVE   = 120;
    constexpr int CERRADO_BANDERA = 65;
}

#define DEBUG_LINK Serial0   // consola + comandos por el puerto UART del DevKit

// ===========================================================================
//  [2] PWM DE SERVOS — mismos valores que firmware-esp32/
// ===========================================================================

namespace Pwm {
    // Servos por PCA9685: 50 Hz, 12 bits de resolución (0..4095 por periodo).
    // Un periodo de 20 ms en 4096 pasos -> 1.0 ms = 205 y 2.0 ms = 410.
    constexpr uint32_t SERVO_FREQ_HZ  = 50;
    constexpr uint16_t SERVO_TICK_MIN = 205;   // pulso 1.0 ms ->   0 grados
    constexpr uint16_t SERVO_TICK_MAX = 410;   // pulso 2.0 ms -> 180 grados
}

// Convierte un ángulo 0..180 al número de ticks del PCA9685.
uint16_t ServoAngleToTicks(int angle_deg) {
    angle_deg = constrain(angle_deg, 0, 180);
    return (uint16_t)(Pwm::SERVO_TICK_MIN +
        ((uint32_t)(Pwm::SERVO_TICK_MAX - Pwm::SERVO_TICK_MIN) * (uint32_t)angle_deg) / 180UL);
}

// ===========================================================================
//  [3] DRIVER PCA9685 — mismo driver escrito a mano que en firmware-esp32/
// ===========================================================================

namespace Pca9685 {
    constexpr uint8_t REG_MODE1     = 0x00;
    constexpr uint8_t REG_MODE2     = 0x01;
    constexpr uint8_t REG_LED0_ON_L = 0x06;
    constexpr uint8_t REG_PRESCALE  = 0xFE;

    constexpr uint8_t MODE1_RESTART = 0x80;
    constexpr uint8_t MODE1_AI      = 0x20;   // auto-incremento de registro
    constexpr uint8_t MODE1_SLEEP   = 0x10;
    constexpr uint8_t MODE2_OUTDRV  = 0x04;   // salida totem-pole

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

        Wire.beginTransmission(I2CAddr::PCA9685);
        Wire.write(REG_LED0_ON_L + 4 * channel);
        Wire.write(0x00);                      // ON  low  -> el pulso empieza en 0
        Wire.write(0x00);                      // ON  high
        Wire.write((uint8_t)(ticks & 0xFF));   // OFF low
        Wire.write((uint8_t)(ticks >> 8));     // OFF high
        return Wire.endTransmission() == 0;
    }
}

// ===========================================================================
//  [4] ESTADO DE CALIBRACIÓN + COMANDOS
// ===========================================================================

namespace Cal {
    uint8_t channel  = ServoChannel::CLAW;
    int     claw_deg = 0;
    int     step_deg = 5;
    bool    pca_ok   = false;
}

// Cal::pca_ok refleja el resultado de la ÚLTIMA escritura I2C (no un
// histórico acumulado): así, si el PCA9685 se reconecta a mitad de sesión,
// el siguiente comando lo vuelve a marcar OK sin necesidad de reiniciar.
void ApplyClaw(int deg) {
    Cal::claw_deg = constrain(deg, 0, 180);
    Cal::pca_ok = Pca9685::SetChannel(Cal::channel, ServoAngleToTicks(Cal::claw_deg));
}

void PrintStatus() {
    DEBUG_LINK.printf("[Estado] canal=%u   pinza=%d grados   paso=%d grados   PCA9685=%s\n",
        Cal::channel, Cal::claw_deg, Cal::step_deg, Cal::pca_ok ? "OK" : "FALLO");
}

void PrintHelp() {
    DEBUG_LINK.println("Comandos: abrir | cerrar llave | cerrar bandera | c <g> | c+ | c- | canal <n> | scan | paso <n> | 0 | ?");
}

// Prueba los 16 canales uno por uno: mueve cada uno 0 -> 180 -> 0 e imprime
// cuál está activo en cada momento, para identificar a simple vista en qué
// canal está cableado el servo. Es una operación bloqueante y a propósito:
// es un diagnóstico de una sola vez, no algo que deba correr en segundo
// plano mientras se procesan más comandos.
void ScanChannels() {
    const uint8_t previous_channel = Cal::channel;

    DEBUG_LINK.println("[Scan] Probando los 16 canales del PCA9685. Mira cual mueve tu servo.");
    for (uint8_t ch = 0; ch < PCA9685_CHANNEL_COUNT; ++ch) {
        DEBUG_LINK.printf("[Scan] Canal %u...\n", ch);
        Pca9685::SetChannel(ch, ServoAngleToTicks(0));
        delay(400);
        Pca9685::SetChannel(ch, ServoAngleToTicks(180));
        delay(700);
        Pca9685::SetChannel(ch, ServoAngleToTicks(0));
        delay(400);
    }
    DEBUG_LINK.println("[Scan] Listo. Usa \"canal <n>\" con el numero que viste moverse.");

    Cal::channel = previous_channel;   // el scan no cambia la selección activa
    PrintStatus();
}

// Procesa una línea de comando ya sin '\r'/'\n' en los extremos.
void HandleCommand(String line) {
    line.trim();
    if (line.length() == 0) {
        PrintStatus();
        return;
    }

    if (line == "?") {
        PrintHelp();
    } else if (line == "0") {
        ApplyClaw(0);
    } else if (line == "abrir") {
        ApplyClaw(Calibrado::ABIERTO);
    } else if (line == "cerrar llave") {
        ApplyClaw(Calibrado::CERRADO_LLAVE);
    } else if (line == "cerrar bandera") {
        ApplyClaw(Calibrado::CERRADO_BANDERA);
    } else if (line == "cerrar") {
        DEBUG_LINK.println("[Error] especifica el objeto: \"cerrar llave\" o \"cerrar bandera\"");
        return;
    } else if (line == "c+") {
        ApplyClaw(Cal::claw_deg + Cal::step_deg);
    } else if (line == "c-") {
        ApplyClaw(Cal::claw_deg - Cal::step_deg);
    } else if (line.startsWith("c ")) {
        ApplyClaw(line.substring(2).toInt());
    } else if (line == "scan") {
        ScanChannels();
        return;   // ScanChannels ya imprimió su propio estado final
    } else if (line.startsWith("canal ")) {
        Cal::channel = (uint8_t)constrain(line.substring(6).toInt(), 0, PCA9685_CHANNEL_COUNT - 1);
        ApplyClaw(Cal::claw_deg);   // reaplica el ángulo actual al canal nuevo
    } else if (line.startsWith("paso ")) {
        Cal::step_deg = constrain(line.substring(5).toInt(), 1, 90);
    } else {
        DEBUG_LINK.printf("[Error] comando no reconocido: \"%s\"\n", line.c_str());
        PrintHelp();
        return;
    }

    PrintStatus();
}

// ===========================================================================
//  [5] setup() / loop()
// ===========================================================================

void setup() {
    DEBUG_LINK.begin(115200);
    delay(200);
    DEBUG_LINK.println("\nPrueba 06 - Calibracion interactiva del servo del gripper");

    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);

    Cal::pca_ok = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
    if (!Cal::pca_ok) {
        DEBUG_LINK.println("[Setup] ERROR: el PCA9685 no responde. Revisa cableado I2C y V+.");
    }

    // Mismo cero mecánico que 04-servos-a-cero: punto de partida conocido.
    ApplyClaw(0);

    PrintHelp();
    PrintStatus();
}

// Algunos monitores seriales no mandan '\n'/'\r' al enviar texto (depende de
// su configuración de "line ending"). Sin este respaldo, un comando sin
// salto de línea se queda esperando en el buffer para siempre. Por eso
// también se ejecuta si pasa este tiempo sin llegar un byte nuevo.
constexpr uint32_t LINE_IDLE_TIMEOUT_MS = 200;

void loop() {
    static String line;
    static uint32_t last_byte_ms = 0;

    while (DEBUG_LINK.available() > 0) {
        char c = (char)DEBUG_LINK.read();
        if (c == '\n' || c == '\r') {
            if (line.length() > 0) {
                HandleCommand(line);
                line = "";
            }
        } else {
            line += c;
            last_byte_ms = millis();
        }
    }

    if (line.length() > 0 && (uint32_t)(millis() - last_byte_ms) > LINE_IDLE_TIMEOUT_MS) {
        HandleCommand(line);
        line = "";
    }
}
