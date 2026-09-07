// ===========================================================================
//  Banco de calibración del ToF VL53L1X — Athena Rover 2026
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  Objetivo: encontrar a qué distancia (mm) cerrar el gripper sobre la
//  bandera. Imprime la distancia del VL53L1X continuamente por consola, y
//  escucha comandos por serial para abrir/cerrar el gripper A MANO en
//  cualquier momento -- así se puede sostener la bandera a una distancia,
//  ver el número, cerrar, y confirmar a ojo si agarró bien.
//
//  No es el firmware del rover (ese vive en firmware-esp32/), pero SÍ corre
//  sobre el mismo ESP32-S3, con el VL53L1X y el PCA9685 (gripper)
//  conectados -- sin motores, sin QTR, sin sensores de color.
//
//  Driver del VL53L1X: mismo procedimiento de arranque que tenía
//  firmware-esp32/src/main.cpp antes de que el bus I2C nº0 se retirara
//  anoche (ver git log de ese archivo).
//
//  ⚠️ CONFIRMADO EN BANCO 2026-09-07: `VL53L1X::setAddress()` (la
//  reasignación de 0x29 a 0x30, pensada para poder compartir el bus con el
//  TCS34725 delantero, que también es 0x29 fijo) NO surte efecto en este
//  sensor -- un barrido I2C después de llamarla lo sigue mostrando en
//  0x29. No es un problema de cableado ni de alimentación (el sensor
//  responde bien en 0x29 antes Y después del intento, y mide perfecto una
//  vez que se deja `init()` trabajar sobre esa misma dirección de
//  fábrica). Por eso `setAddress()` está comentada más abajo: **este banco
//  asume que el VL53L1X es el ÚNICO dispositivo en el bus 0** (sin el
//  TCS34725 delantero conectado a la vez). Investigar `setAddress()` es
//  tarea aparte, necesaria recién cuando haga falta compartir el bus de
//  nuevo.
//
//  Protocolo por serial:
//    'O'  -> abre el gripper
//    'C'  -> cierra el gripper (ángulo de "agarrar bandera")
//  Todo lo demás se ignora. La distancia se imprime sola, sin pedirla.
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// ===========================================================================
//  PINES — idénticos a firmware-esp32/src/main.cpp
// ===========================================================================

namespace Pins {
    // Bus I2C nº0: el VL53L1X vive acá (junto con el TCS34725 delantero en
    // el robot real, aunque acá no haga falta conectarlo).
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;
    constexpr uint8_t TOF_XSHUT = 3;

    // Bus I2C nº1: el PCA9685 (gripper) vive acá.
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;
}

namespace I2CAddr {
    constexpr uint8_t PCA9685 = 0x40;

    // El VL53L1X arranca SIEMPRE en 0x29 -- se le reasigna esta dirección
    // nueva apenas sale de reset, antes de hacer cualquier otra cosa.
    constexpr uint8_t VL53L1X_BOOT_ADDR = 0x29;
    constexpr uint8_t VL53L1X = 0x30;
}

namespace ServoChannel {
    constexpr uint8_t CLAW = 0;
}

namespace Pwm {
    constexpr uint32_t SERVO_FREQ_HZ  = 50;
    constexpr uint16_t SERVO_TICK_MIN = 205;   // pulso 1.0 ms ->   0 grados
    constexpr uint16_t SERVO_TICK_MAX = 410;   // pulso 2.0 ms -> 180 grados
}

// Mismos ángulos que firmware-esp32/ y los standalones -- ver
// pruebas-platformio/06-calibracion-gripper/.
constexpr int kClawOpenDeg          = 0;
constexpr int kClawClosedBanderaDeg = 65;

// ===========================================================================
//  DRIVER PCA9685
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
//  DRIVER VL53L1X (Pololu)
// ===========================================================================

namespace Tof {
    constexpr uint32_t BOOT_DELAY_MS = 2;       // datasheet pide ~1.2 ms, con margen
    constexpr uint16_t TIMING_BUDGET_US = 50000; // 50 ms -- alcanza de sobra para esta distancia corta
    constexpr uint32_t RANGING_PERIOD_MS = 50;
}

VL53L1X g_tof;
bool g_tofOk = false;

bool TofBringUp() {
    g_tof.setBus(&Wire);
    g_tof.setTimeout(500);
    // DIAGNÓSTICO: la reasignación a 0x30 no está surtiendo efecto (ver el
    // aviso grande arriba) -- sin nada más en el bus ahora mismo, se deja
    // el sensor en su dirección de fábrica (0x29, la que ya trae la
    // librería por defecto) para aislar si el problema es SOLO la
    // reasignación o algo más profundo.
    // g_tof.setAddress(I2CAddr::VL53L1X);
    if (!g_tof.init()) return false;
    g_tof.setDistanceMode(VL53L1X::Long);
    g_tof.setMeasurementTimingBudget(Tof::TIMING_BUDGET_US);
    g_tof.startContinuous(Tof::RANGING_PERIOD_MS);
    return true;
}

// ===========================================================================
// SETUP
// ===========================================================================

bool g_pcaOk = false;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nBanco de calibracion del ToF VL53L1X -- Athena Rover 2026");
    Serial.println("Comandos: 'O' abre el gripper, 'C' lo cierra (angulo de bandera).\n");

    // XSHUT en LOW ANTES de abrir el bus -- así el VL53L1X no responde en
    // 0x29 hasta que se le reasigna dirección, ver el aviso al principio.
    pinMode(Pins::TOF_XSHUT, OUTPUT);
    digitalWrite(Pins::TOF_XSHUT, LOW);

    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);

    digitalWrite(Pins::TOF_XSHUT, HIGH);
    delay(Tof::BOOT_DELAY_MS);

    // DIAGNÓSTICO: barrido completo del bus 0 justo aquí -- XSHUT ya está
    // liberado (el sensor debería responder en su dirección ACTUAL,
    // cualquiera que sea) pero TODAVÍA no se le pidió cambiarla. Si el
    // VL53L1X nunca perdió alimentación de verdad entre una sesión y otra,
    // puede haber quedado en 0x30 (la dirección reasignada la última vez)
    // en vez de volver a su default de fábrica 0x29 -- este barrido lo
    // muestra tal cual está, sin asumir nada.
    Serial.println("[Diag] Barriendo bus I2C 0 (0x08-0x77)...");
    int encontrados = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[Diag]   responde en 0x%02X\n", addr);
            ++encontrados;
        }
    }
    if (encontrados == 0) {
        Serial.println("[Diag]   NADA respondio en todo el bus -- revisa cableado/alimentacion del ToF, no es un tema de direccion.");
    }
    Serial.println();

    g_tofOk = TofBringUp();
    if (!g_tofOk) Serial.println("[ToF] VL53L1X no responde. Reintentando en segundo plano.");

    // Segundo barrido, DESPUÉS del intento de reasignación: si ahora
    // aparece en 0x30, la reasignación funcionó y el problema está en
    // init() de ahí en más; si sigue en 0x29, la reasignación en sí no
    // surtió efecto; si no aparece nada, el sensor quedó colgado/mudo tras
    // el intento.
    Serial.println("[Diag] Segundo barrido, despues de intentar reasignar direccion...");
    int encontrados2 = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[Diag]   responde en 0x%02X\n", addr);
            ++encontrados2;
        }
    }
    if (encontrados2 == 0) {
        Serial.println("[Diag]   NADA respondio -- el sensor quedo mudo tras el intento de reasignar.");
    }
    Serial.println();

    g_pcaOk = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
    if (g_pcaOk) {
        Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
        Serial.println("[Gripper] PCA9685 listo, abierto.");
    } else {
        Serial.println("[Gripper] PCA9685 no responde. Reintentando en segundo plano.");
    }

    Serial.println("Listo.\n");
}

// ===========================================================================
// LOOP
// ===========================================================================

void loop() {
    static uint32_t last_retry_ms = 0;
    static uint32_t last_print_ms = 0;

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

    if (Serial.available()) {
        const char cmd = Serial.read();
        if (cmd == 'O' || cmd == 'o') {
            if (g_pcaOk) {
                Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
                Serial.println("[Gripper] ABRIENDO");
            } else {
                Serial.println("[Gripper] PCA9685 no disponible.");
            }
        } else if (cmd == 'C' || cmd == 'c') {
            if (g_pcaOk) {
                Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawClosedBanderaDeg));
                Serial.println("[Gripper] CERRANDO sobre la bandera");
            } else {
                Serial.println("[Gripper] PCA9685 no disponible.");
            }
        }
        while (Serial.available()) Serial.read();   // descarta \r\n sobrantes
    }

    // Imprime la distancia a 10 Hz, sin bloquear el resto del loop().
    if ((uint32_t)(millis() - last_print_ms) >= 100) {
        last_print_ms = millis();
        if (g_tofOk && g_tof.dataReady()) {
            const uint16_t mm = g_tof.read(false);
            const bool valid = !g_tof.timeoutOccurred() &&
                                g_tof.ranging_data.range_status == VL53L1X::RangeValid;
            if (g_tof.timeoutOccurred()) g_tofOk = false;
            Serial.printf("distancia=%4u mm  %s\n", mm, valid ? "OK" : "SIN MEDICION VALIDA");
        } else if (!g_tofOk) {
            Serial.println("distancia=  ??? mm  SENSOR NO RESPONDE");
        }
    }
}
