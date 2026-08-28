// ===========================================================================
//  Prueba 03 — Motores adelante
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  OBJETIVO: la única función de este sketch es mover los 4 motores hacia
//  adelante, a velocidad máxima y constante. Sin sensores, sin lógica, sin
//  nada más — es la prueba más simple posible del proyecto.
//
//  Para qué sirve:
//    - Confirmar que los 2 L298N y las 4 ruedas están bien cableados (jumpers
//      ENA/ENB quitados, sentido de giro correcto) antes de sumarle sensores
//      o cualquier máquina de estados.
//    - Como prueba mínima de subida: si esto compila, sube y el robot avanza,
//      cualquier problema de conexión de PlatformIO/esptool que hayas tenido
//      (por ejemplo un "write timeout") era del puerto o del cable, no de
//      este proyecto.
//
//  ⚠️ ANTES DE ENCENDER: pon el robot sobre un soporte con las 4 ruedas al
//  aire, o en el piso con espacio libre por delante. Arranca en cuanto
//  termina setup(), sin espera ni confirmación por serial — no hay forma de
//  detenerlo por software una vez subido, solo desconectando la batería de
//  motores o reseteando el ESP32.
//
// ===========================================================================

#include <Arduino.h>

// ===========================================================================
//  [1] PINES — idénticos a hardware/conexiones-esp32-s3.md
// ===========================================================================

namespace Pins {
    // Driver IZQUIERDO -> motor delantero izq. (FL) y trasero izq. (RL)
    constexpr uint8_t L298N_L_IN1 = 4;
    constexpr uint8_t L298N_L_IN2 = 5;
    constexpr uint8_t L298N_L_ENA = 6;
    constexpr uint8_t L298N_L_IN3 = 7;
    constexpr uint8_t L298N_L_IN4 = 15;
    constexpr uint8_t L298N_L_ENB = 16;

    // Driver DERECHO -> motor delantero der. (FR) y trasero der. (RR)
    constexpr uint8_t L298N_R_IN1 = 10;
    constexpr uint8_t L298N_R_IN2 = 11;
    constexpr uint8_t L298N_R_ENA = 12;
    constexpr uint8_t L298N_R_IN3 = 13;
    constexpr uint8_t L298N_R_IN4 = 14;
    constexpr uint8_t L298N_R_ENB = 17;
}

#define DEBUG_LINK Serial0   // consola por el puerto UART del DevKit

// ===========================================================================
//  [2] MOTORES
// ===========================================================================

namespace Pwm {
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;   // el L298N calienta y pierde par a 20 kHz
    constexpr uint8_t  MOTOR_RESOLUTION = 8;      // duty 0..255
}

struct Motor {
    uint8_t in1, in2, en, ledc_channel;
};

// kMotorFL: con el cableado físico actual, la rueda conectada a OUT1/OUT2
// del L298N izquierdo (la trasera izquierda del chasis) gira al revés
// respecto a las otras tres. Se resuelve intercambiando el ORDEN de los dos
// GPIO aquí mismo: Pins::L298N_L_IN1 y L298N_L_IN2 siguen siendo,
// físicamente, los pines soldados a los terminales IN1 e IN2 del L298N (ver
// hardware/conexiones-esp32-s3.md) — lo único que cambia es cuál de los dos
// hace de "in1" y cuál de "in2" para ESTE motor, así que MotorForwardFull()
// queda idéntica para los 4 motores, sin funciones ni ramas especiales. Si
// se recablea este motor para que coincida con los otros tres, basta con
// volver a poner IN1, IN2 en orden aquí.
constexpr Motor kMotorFL = {Pins::L298N_L_IN2, Pins::L298N_L_IN1, Pins::L298N_L_ENA, 0};
constexpr Motor kMotorRL = {Pins::L298N_L_IN3, Pins::L298N_L_IN4, Pins::L298N_L_ENB, 1};
constexpr Motor kMotorFR = {Pins::L298N_R_IN1, Pins::L298N_R_IN2, Pins::L298N_R_ENA, 2};
constexpr Motor kMotorRR = {Pins::L298N_R_IN3, Pins::L298N_R_IN4, Pins::L298N_R_ENB, 3};

// La API de LEDC cambió entre el core 2.x y el 3.x de Arduino-ESP32.
void PwmAttach(uint8_t pin, uint8_t channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;
    ledcAttach(pin, Pwm::MOTOR_FREQ_HZ, Pwm::MOTOR_RESOLUTION);
#else
    ledcSetup(channel, Pwm::MOTOR_FREQ_HZ, Pwm::MOTOR_RESOLUTION);
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
    PwmAttach(m.en, m.ledc_channel);
    PwmWrite(m.en, m.ledc_channel, 0);
}

// Adelante a fondo: IN1=HIGH, IN2=LOW, duty al máximo (255/255).
void MotorForwardFull(const Motor &m) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
    PwmWrite(m.en, m.ledc_channel, 255);
}

// ===========================================================================
//  [3] setup() / loop()
// ===========================================================================

void setup() {
    DEBUG_LINK.begin(115200);
    delay(200);
    DEBUG_LINK.println("\nPrueba 03 - Motores adelante a velocidad maxima");

    MotorSetup(kMotorFL);
    MotorSetup(kMotorRL);
    MotorSetup(kMotorFR);
    MotorSetup(kMotorRR);

    MotorForwardFull(kMotorFL);
    MotorForwardFull(kMotorRL);
    MotorForwardFull(kMotorFR);
    MotorForwardFull(kMotorRR);

    DEBUG_LINK.println("[Setup] los 4 motores adelante a velocidad maxima. No se detienen por software.");
}

void loop() {
    // Todo el trabajo pasó en setup(): el PWM se queda en el duty que se le
    // escribió sin necesidad de refrescarlo en cada vuelta.
    delay(1000);
}
