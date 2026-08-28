// ===========================================================================
//  Prueba 04 — Servos a cero
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  OBJETIVO: la única función de este sketch es mandar los servos del
//  gripper (pinza y elevación, canales 0 y 1 del PCA9685) a 0°, y dejarlos
//  ahí. Sin sensores, sin lógica, sin protocolo con la Raspberry Pi.
//
//  Para qué sirve:
//    - Punto de referencia mecánico fijo (0°) para montar los brazos/cuernos
//      del servo en la posición correcta antes de atornillarlos.
//    - Sacar el gripper de cualquier posición rara en la que haya quedado
//      tras una prueba anterior, sin tener que hablar con la Raspberry Pi.
//
//  ⚠️ ANTES DE ENCENDER: monta el gripper DESACOPLADO del mecanismo (mismo
//  criterio que en firmware-esp32/, sección [8.2]). Un servo que ya estaba
//  en otro ángulo puede dar un salto brusco hasta 0° en cuanto arranca; si
//  el cuerno ya está atornillado contra un tope mecánico, ese salto lo
//  fuerza. Calibra el 0° con el servo suelto y atorníllalo después.
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

// Canales del PCA9685 usados por el gripper (ver firmware-esp32/src/main.cpp).
namespace ServoChannel {
    constexpr uint8_t CLAW = 0;   // abre/cierra la pinza
    constexpr uint8_t LIFT = 1;   // sube/baja el gripper
}

#define DEBUG_LINK Serial0   // consola por el puerto UART del DevKit

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
//  [4] setup() / loop()
// ===========================================================================

void setup() {
    DEBUG_LINK.begin(115200);
    delay(200);
    DEBUG_LINK.println("\nPrueba 04 - Servos a cero (pinza y elevacion)");

    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);

    if (!Pca9685::Init(Pwm::SERVO_FREQ_HZ)) {
        DEBUG_LINK.println("[Setup] ERROR: el PCA9685 no responde. Revisa cableado I2C y V+.");
        return;
    }

    const uint16_t zero_ticks = ServoAngleToTicks(0);
    const bool claw_ok = Pca9685::SetChannel(ServoChannel::CLAW, zero_ticks);
    const bool lift_ok = Pca9685::SetChannel(ServoChannel::LIFT, zero_ticks);

    DEBUG_LINK.printf("[Setup] pinza a 0: %s\n", claw_ok ? "OK" : "FALLO");
    DEBUG_LINK.printf("[Setup] elevacion a 0: %s\n", lift_ok ? "OK" : "FALLO");
}

void loop() {
    // Todo el trabajo pasó en setup(): el PCA9685 mantiene el pulso por su
    // cuenta sin necesidad de refrescarlo en cada vuelta.
    delay(1000);
}
