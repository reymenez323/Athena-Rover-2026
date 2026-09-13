// ===========================================================================
//  Prueba 09 — ToF + cámara + gripper, aislado de todo lo demás
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  OBJETIVO: diagnosticar por qué el gripper nunca cerró en la corrida
//  completa de standalones/v7-mision-completa-camara/, quitando de en
//  medio todo lo que no sea ToF + gripper + cámara -- sin motores, sin
//  color, sin las fases de misión que tienen que pasar antes de llegar al
//  agarre. Le acercas la bandera a mano y el ESP32 solo:
//    1. Mide distancia con el ToF, todo el tiempo.
//    2. Escucha si la Raspberry Pi (cámara) confirma que ve la bandera
//       contraria -- mismo protocolo de un byte 'V' que
//       standalones/v7-mision-completa-camara/ (CAM_LINK). Corré
//       raspberry-pi/scripts/avisar_bandera_v7.py sin cambiarle nada.
//    3. Cuando AMBAS cosas se cumplen a la vez -- cámara confirmando Y ToF
//       en rango de agarre 3 veces seguidas -- cierra el gripper solo.
//
//  CÓMO USAR ESTO PARA DIAGNOSTICAR "cable suelto / falta de corriente /
//  algo más":
//    - Si acá el gripper cierra bien (solo o a mano con 'C'), el problema
//      NO es el ToF ni el PCA9685 ni su cableado -- es que la máquina de
//      estados de v7 nunca llegó de verdad a la fase de agarre. Revisar el
//      log de v7 por UART para ver en qué fase se quedó.
//    - Si el gripper NO cierra ni siquiera a mano ('C'), pero el log dice
//      "[Gripper] PCA9685 listo" -- es mecánico (el servo no tiene fuerza,
//      algo lo traba) o el driver manda la señal pero el servo no
//      responde. Revisar el cableado de señal/alimentación del servo.
//    - Si el log dice "[Gripper] PCA9685 no responde" -- es el bus I2C 1
//      (cableado SDA/SCL, o el PCA9685 sin alimentación de verdad).
//    - PRUEBA DE CONSUMO: corré esto primero con el robot quieto (motores
//      sin girar, aunque estén conectados a la misma batería). Si funciona
//      bien así, y en v7 fallaba con los motores en movimiento, el
//      problema es de consumo eléctrico (los motores bajan el voltaje del
//      riel compartido lo suficiente como para que el ToF/PCA9685 fallen o
//      se reinicien) -- no un cable suelto.
//
//  COMANDOS POR EL PUERTO UART (DEBUG_LINK), además del cierre automático:
//    'O'  -> abre el gripper a mano
//    'C'  -> cierra el gripper a mano (mismo ángulo que el agarre automático)
//    'R'  -> rearma el cierre automático sin tener que abrir primero
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// ===========================================================================
//  [1] PINES — mismo cableado que calibracion/tof/firmware/ y v7 (bus 0
//  solo para el ToF, bus 1 para el PCA9685 -- ver la historia completa del
//  choque de bus en calibracion/tof/README.md, no repetida acá).
// ===========================================================================

namespace Pins {
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;
    constexpr uint8_t TOF_XSHUT = 3;

    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;
}

namespace I2CAddr {
    constexpr uint8_t PCA9685 = 0x40;
    constexpr uint8_t VL53L1X = 0x29;
}

namespace ServoChannel {
    constexpr uint8_t CLAW = 0;
}

namespace Pwm {
    constexpr uint32_t SERVO_FREQ_HZ  = 50;
    constexpr uint16_t SERVO_TICK_MIN = 205;   // pulso 1.0 ms ->   0 grados
    constexpr uint16_t SERVO_TICK_MAX = 410;   // pulso 2.0 ms -> 180 grados
}

// Mismos ángulos que firmware-esp32/ y los standalones.
constexpr int kClawOpenDeg          = 0;
constexpr int kClawClosedBanderaDeg = 65;

#define DEBUG_LINK Serial0   // consola por el puerto UART del DevKit
#define CAM_LINK   Serial    // USB nativo -- la Raspberry Pi va acá (ver v7)

// -- Umbral de agarre -- MISMOS valores que v5/v6/v7, confirmados en banco --
constexpr uint16_t kRangoAgarreMinMm = 56;
constexpr uint16_t kRangoAgarreMaxMm = 60;
constexpr int kLecturasConsecutivasRequeridas = 3;

// -- Señal de cámara -- MISMOS valores que v7-mision-completa-camara/ --
constexpr uint32_t kFrescoBanderaCamaraMs = 300;
constexpr uint32_t kSostenBanderaCamaraMs = 150;

// ===========================================================================
//  [2] DRIVER PCA9685 (gripper, bus I2C nº1)
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
//  [3] DRIVER VL53L1X (Pololu) -- bus I2C nº0, solo
// ===========================================================================

namespace Tof {
    constexpr uint32_t BOOT_DELAY_MS = 2;
    constexpr uint16_t TIMING_BUDGET_US = 50000;
    constexpr uint32_t RANGING_PERIOD_MS = 50;
}

VL53L1X g_tof;
bool g_tofOk = false;

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

// ===========================================================================
//  [4] SEÑAL DE CÁMARA -- idéntico a CamaraBandera:: en
//  standalones/v7-mision-completa-camara/src/main.cpp, sin depender de
//  Mission:: (acá no hay fases, así que las constantes van sueltas arriba).
// ===========================================================================

namespace CamaraBandera {
    volatile uint32_t ultima_senal_ms = 0;
    volatile uint32_t sostenida_desde_ms = 0;

    void Actualizar() {
        bool vista_este_ciclo = false;
        while (CAM_LINK.available() > 0) {
            if ((char)CAM_LINK.read() == 'V') vista_este_ciclo = true;
        }
        const uint32_t ahora = millis();
        if (vista_este_ciclo) ultima_senal_ms = ahora;

        const bool fresca = (uint32_t)(ahora - ultima_senal_ms) <= kFrescoBanderaCamaraMs;
        if (fresca) {
            if (sostenida_desde_ms == 0) sostenida_desde_ms = ahora;
        } else {
            sostenida_desde_ms = 0;
        }
    }

    bool Confirmada() {
        return sostenida_desde_ms != 0 &&
               (uint32_t)(millis() - sostenida_desde_ms) >= kSostenBanderaCamaraMs;
    }
}

// ===========================================================================
// SETUP
// ===========================================================================

bool g_pcaOk = false;
bool g_gripperCerrado = false;
int g_lecturasEnRango = 0;
bool g_autoArmado = true;   // false tras cerrar -- 'R' o salir de rango lo rearma

void AbrirGripper(const char *motivo) {
    if (!g_pcaOk) { DEBUG_LINK.println("[Gripper] PCA9685 no disponible."); return; }
    Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
    g_gripperCerrado = false;
    DEBUG_LINK.printf("[Gripper] ABRIENDO (%s)\n", motivo);
}

void CerrarGripper(const char *motivo) {
    if (!g_pcaOk) { DEBUG_LINK.println("[Gripper] PCA9685 no disponible."); return; }
    Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawClosedBanderaDeg));
    g_gripperCerrado = true;
    DEBUG_LINK.printf("[Gripper] CERRANDO (%s)\n", motivo);
}

void setup() {
    DEBUG_LINK.begin(115200);
    CAM_LINK.begin(115200);
    delay(1000);
    DEBUG_LINK.println("\nPrueba 09 - ToF + camara + gripper, aislado");
    DEBUG_LINK.println("Comandos: 'O' abre, 'C' cierra a mano, 'R' rearma el cierre automatico.\n");

    pinMode(Pins::TOF_XSHUT, OUTPUT);
    digitalWrite(Pins::TOF_XSHUT, LOW);

    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);

    digitalWrite(Pins::TOF_XSHUT, HIGH);
    delay(Tof::BOOT_DELAY_MS);

    g_tofOk = TofBringUp();
    DEBUG_LINK.println(g_tofOk ? "[ToF] VL53L1X listo (0x29, bus 0)."
                                 : "[ToF] VL53L1X no responde. Reintentando en segundo plano.");

    g_pcaOk = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
    if (g_pcaOk) {
        AbrirGripper("arranque");
        DEBUG_LINK.println("[Gripper] PCA9685 listo.");
    } else {
        DEBUG_LINK.println("[Gripper] PCA9685 no responde. Reintentando en segundo plano.");
    }

    DEBUG_LINK.println("Listo.\n");
}

// ===========================================================================
// LOOP
// ===========================================================================

void loop() {
    static uint32_t last_retry_ms = 0;
    static uint32_t last_print_ms = 0;

    CamaraBandera::Actualizar();

    if ((!g_tofOk || !g_pcaOk) && (uint32_t)(millis() - last_retry_ms) > 1000) {
        last_retry_ms = millis();
        if (!g_tofOk) {
            g_tofOk = g_tof.init();
            if (g_tofOk) {
                g_tof.setDistanceMode(VL53L1X::Long);
                g_tof.setMeasurementTimingBudget(Tof::TIMING_BUDGET_US);
                g_tof.startContinuous(Tof::RANGING_PERIOD_MS);
            }
        }
        if (!g_pcaOk) {
            g_pcaOk = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
            if (g_pcaOk) Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
        }
    }

    if (DEBUG_LINK.available()) {
        const char cmd = DEBUG_LINK.read();
        if (cmd == 'O' || cmd == 'o') {
            AbrirGripper("a mano");
            g_lecturasEnRango = 0;
            g_autoArmado = true;
        } else if (cmd == 'C' || cmd == 'c') {
            CerrarGripper("a mano");
        } else if (cmd == 'R' || cmd == 'r') {
            g_autoArmado = true;
            g_lecturasEnRango = 0;
            DEBUG_LINK.println("[Auto] rearmado -- listo para cerrar solo de nuevo.");
        }
        while (DEBUG_LINK.available()) DEBUG_LINK.read();
    }

    // El ToF mide a ~20 Hz (RANGING_PERIOD_MS=50) pero este loop() da
    // muchas más vueltas por segundo que eso -- si distancia_mm/valida
    // fueran variables locales reiniciadas cada vuelta, la inmensa mayoría
    // de las vueltas verían "sin dato nuevo todavía" y se leerían como
    // inválidas, aunque el sensor esté midiendo perfectamente bien (esto
    // pasaba acá: bug ya corregido 2026-09-13). Se guardan como estáticas,
    // actualizadas SOLO cuando de verdad hay una lectura nueva, y el conteo
    // de "lecturas consecutivas en rango" también se actualiza ahí mismo
    // -- no en cada vuelta del loop() -- para que de verdad cuente
    // mediciones físicas distintas, no la misma lectura vista de refilón
    // varias veces.
    static uint16_t g_ultimaDistanciaMm = 0;
    static bool g_ultimaValida = false;
    static uint32_t g_ultimaLecturaMs = 0;
    constexpr uint32_t kLecturaObsoletaMs = 200;   // ~4 ciclos del sensor sin dato nuevo -> se muestra inválida

    if (g_tofOk && g_tof.dataReady()) {
        const uint16_t mm = g_tof.read(false);
        const bool valid = !g_tof.timeoutOccurred() &&
                            g_tof.ranging_data.range_status == VL53L1X::RangeValid;
        if (g_tof.timeoutOccurred()) g_tofOk = false;

        g_ultimaDistanciaMm = mm;
        g_ultimaValida = valid;
        g_ultimaLecturaMs = millis();

        const bool en_rango_ahora = valid && mm >= kRangoAgarreMinMm && mm <= kRangoAgarreMaxMm;
        if (en_rango_ahora) {
            if (g_lecturasEnRango < kLecturasConsecutivasRequeridas) ++g_lecturasEnRango;
        } else {
            g_lecturasEnRango = 0;
            if (g_gripperCerrado) g_autoArmado = true;   // salió de rango -- se puede probar de nuevo
        }
    }

    const bool distancia_valida = g_ultimaValida &&
        (uint32_t)(millis() - g_ultimaLecturaMs) <= kLecturaObsoletaMs;
    const uint16_t distancia_mm = g_ultimaDistanciaMm;

    const bool camara_confirma = CamaraBandera::Confirmada();

    // -- Cierre automático: cámara confirmando Y ToF en rango 3 veces
    // seguidas Y no cerrado ya (o rearmado) ------------------------------
    if (g_autoArmado && !g_gripperCerrado &&
        camara_confirma && g_lecturasEnRango >= kLecturasConsecutivasRequeridas) {
        CerrarGripper("AUTOMATICO: camara confirma + ToF en rango 3 veces seguidas");
        g_autoArmado = false;
    }

    // Diagnóstico a 10 Hz.
    if ((uint32_t)(millis() - last_print_ms) >= 100) {
        last_print_ms = millis();
        DEBUG_LINK.printf(
            "distancia=%s%4u mm  en_rango_x%d/%d  camara=%s  gripper=%s\n",
            distancia_valida ? " " : "(invalida) ", distancia_mm,
            g_lecturasEnRango, kLecturasConsecutivasRequeridas,
            camara_confirma ? "CONFIRMA" : "no ve",
            g_gripperCerrado ? "CERRADO" : "abierto");
    }
}
