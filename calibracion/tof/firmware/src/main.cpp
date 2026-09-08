// ===========================================================================
//  Banco de calibración del ToF VL53L1X + TCS34725 delantero — Athena Rover 2026
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  Objetivo: encontrar a qué distancia (mm) cerrar el gripper sobre la
//  bandera, y a la vez validar que el ToF y el sensor de color delantero
//  conviven bien en el robot. Imprime distancia + color en vivo por
//  consola, y escucha comandos por serial para abrir/cerrar el gripper A
//  MANO en cualquier momento -- así se puede sostener la bandera a una
//  distancia, ver el número, cerrar, y confirmar a ojo si agarró bien.
//
//  No es el firmware del rover (ese vive en firmware-esp32/), pero SÍ corre
//  sobre el mismo ESP32-S3, con el VL53L1X, el TCS34725 delantero y el
//  PCA9685 (gripper) conectados -- sin motores, sin QTR.
//
// ---------------------------------------------------------------------------
//  HISTORIA DEL PROBLEMA DE BUS COMPARTIDO (ToF + TCS34725 delantero)
// ---------------------------------------------------------------------------
//
//  Los dos arrancan en la MISMA dirección fija 0x29, y ninguno de los dos
//  chips tiene pines de selección de dirección (A0-A2) -- esos pines
//  existen en un multiplexor I2C (TCA9548A), NO en el VL53L1X ni el
//  TCS34725. La idea original era reasignarle una dirección nueva al ToF
//  (que sí tiene pin XSHUT para eso) y dejar 0x29 libre para el color.
//
//  - 2026-09-07: `setAddress()` no surtía efecto -- se llamaba ANTES de
//    `init()`, demasiado pronto (el chip todavía no había terminado su
//    arranque interno de firmware). Sin resolver ese día.
//
//  - 2026-09-08, primer y segundo intento: se reordenó a `init()` ->
//    `setAddress()` (sin delay extra, confirmado innecesario) y pareció
//    funcionar perfecto -- barrido mostraba el chip movido a 0x30, lecturas
//    estables cruzando toda la zona de agarre. Se documentó como
//    "RESUELTO". ERROR: esas pruebas se hicieron con el TCS34725 delantero
//    SIN CORRIENTE (un cable de 3.3V se había quedado desconectado sin
//    notarlo) -- o sea que nunca hubo un segundo dispositivo real
//    compitiendo en 0x29. El "resuelto" fue prematuro.
//
//  - 2026-09-08, tercer intento (el real): con el TCS34725 delantero ya
//    con corriente de verdad (LED del sensor encendido, confirmado a
//    simple vista), el `init()` del ToF FALLA POR COMPLETO -- no solo
//    `setAddress()`, la comunicación básica. Reproducido dos veces
//    seguidas. Causa real: el TCS34725 NO tiene pin de reset/apagado --
//    en cuanto tiene corriente, contesta en 0x29 todo el tiempo. El
//    VL53L1X, para su propio `init()`, también necesita hablarle a 0x29
//    (su dirección de fábrica) antes de poder reasignarse. Con los DOS
//    contestando 0x29 a la vez, cualquier lectura por ese bus recibe
//    respuesta simultánea de ambos chips -- como tienen mapas de
//    registros distintos, se corrompen entre sí. El orden de llamadas
//    (`init()` antes de `setAddress()`) nunca fue el problema real; el
//    problema real es que DOS dispositivos viven en la misma dirección al
//    mismo tiempo, sin forma de silenciar a uno de los dos por software
//    (el TCS no tiene pin de reset).
//
//  FIX ADOPTADO: separar los buses físicamente, no por dirección. El
//  TCS34725 delantero se recablea del bus I2C nº0 (GPIO8/9, donde vivía
//  con el ToF) al bus I2C nº1 (GPIO47/48, donde ya vive el PCA9685 del
//  gripper -- dirección 0x40, sin choque con el TCS34725 en 0x29). El
//  TCS34725 trasero se desconecta (bus 1 no puede tener dos dispositivos
//  en 0x29 tampoco). Con esto, el bus 0 queda con el ToF SOLO -- ya no
//  hace falta reasignarle dirección para nada, se queda en su 0x29 de
//  fábrica. Ver `Pins::` e `I2CAddr::` más abajo para el cableado nuevo.
//
//  Protocolo por serial:
//    'O'  -> abre el gripper
//    'C'  -> cierra el gripper (ángulo de "agarrar bandera")
//  Todo lo demás se ignora. Distancia y color se imprimen solos, sin pedirlos.
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// ===========================================================================
//  PINES
// ===========================================================================

namespace Pins {
    // Bus I2C nº0: SOLO el VL53L1X -- ya no comparte con nadie, ver la
    // historia arriba. No hace falta reasignarle dirección.
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;
    constexpr uint8_t TOF_XSHUT = 3;

    // Bus I2C nº1: PCA9685 (gripper, 0x40) + TCS34725 delantero (0x29,
    // recableado acá desde el bus 0) -- direcciones distintas, sin choque.
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    constexpr uint8_t TCS_LED_FRONT = 18;   // LED propio del TCS34725 delantero -- GPIO aparte, no I2C
}

namespace I2CAddr {
    constexpr uint8_t PCA9685 = 0x40;

    // Ya NO se reasigna -- el bus 0 es solo suyo, se queda en su
    // dirección de fábrica.
    constexpr uint8_t VL53L1X = 0x29;

    // Dirección fija del TCS34725, no se puede cambiar.
    constexpr uint8_t TCS34725 = 0x29;
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
//  DRIVER TCS34725 -- copia de ../../color/detector-tcs/src/main.cpp, pero
//  por Wire1 (bus 1) en vez de Wire (bus 0) -- ver la historia arriba.
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

// Umbrales del reajuste 2026-09-07 #2, ver ../../color/detector-tcs/src/main.cpp
// para el detalle completo -- se copian tal cual, mismo sensor delantero.
// Se calibraron con el TCS34725 en el bus 0 -- si el bus nuevo (1, junto al
// PCA9685) le cambia la luz ambiente o el ruido eléctrico percibido, podría
// hacer falta recalibrar; primera señal de alerta sería GRIS/AMARILLO
// confundiéndose de nuevo como pasó el 2026-09-07.
namespace UmbralColor {
    constexpr uint16_t CLEAR_NEGRO_MAX = 314;
    constexpr float ROJO_R_MIN     = 0.450f;
    constexpr float ROJO_G_MAX     = 0.312f;
    constexpr float ROJO_B_MAX     = 0.300f;
    constexpr float AZUL_B_MIN     = 0.206f;
    constexpr float AZUL_R_MAX     = 0.390f;
    constexpr float AMARILLO_R_MIN = 0.420f;
    constexpr float AMARILLO_G_MIN = 0.200f;
    constexpr float AMARILLO_B_MAX = 0.140f;
}

const char *ClasificarColor(const Tcs34725::Rgbc &s) {
    if (s.c < UmbralColor::CLEAR_NEGRO_MAX) return "NEGRO";

    const float total = (float)s.c;
    const float r = (float)s.r / total;
    const float g = (float)s.g / total;
    const float b = (float)s.b / total;

    if (r > UmbralColor::ROJO_R_MIN && g < UmbralColor::ROJO_G_MAX && b < UmbralColor::ROJO_B_MAX) return "ROJO";
    if (b > UmbralColor::AZUL_B_MIN && r < UmbralColor::AZUL_R_MAX) return "AZUL";
    if (r > UmbralColor::AMARILLO_R_MIN && g > UmbralColor::AMARILLO_G_MIN && b < UmbralColor::AMARILLO_B_MAX) return "AMARILLO";

    return "GRIS";
}

// ===========================================================================
//  DRIVER VL53L1X (Pololu) -- ahora solo en el bus 0, sin reasignación.
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
    g_tof.setAddress(I2CAddr::VL53L1X);   // no-op real (ya está en 0x29), lo deja explícito

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
bool g_tcsOk = false;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nBanco de calibracion del ToF VL53L1X + TCS34725 delantero -- Athena Rover 2026");
    Serial.println("Comandos: 'O' abre el gripper, 'C' lo cierra (angulo de bandera).\n");

    pinMode(Pins::TOF_XSHUT, OUTPUT);
    digitalWrite(Pins::TOF_XSHUT, LOW);

    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);

    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);     // bus 0: solo el ToF
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);    // bus 1: PCA9685 + TCS34725 delantero

    digitalWrite(Pins::TOF_XSHUT, HIGH);
    delay(Tof::BOOT_DELAY_MS);

    // DIAGNÓSTICO: barrido de los dos buses. Bus 0 debería mostrar SOLO
    // 0x29 (el ToF, solo). Bus 1 debería mostrar 0x29 (TCS34725 delantero)
    // y 0x40 (PCA9685) -- si el bus 0 muestra algo más que 0x29, o el bus 1
    // le falta alguno de los dos, el recableado no quedó como se esperaba.
    Serial.println("[Diag] Barriendo bus I2C 0 (0x08-0x77) -- deberia ser solo el ToF...");
    int encontrados0 = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[Diag]   responde en 0x%02X\n", addr);
            ++encontrados0;
        }
    }
    if (encontrados0 == 0) {
        Serial.println("[Diag]   NADA respondio en el bus 0 -- revisa cableado/alimentacion del ToF.");
    }
    Serial.println();

    Serial.println("[Diag] Barriendo bus I2C 1 (0x08-0x77) -- deberia ser TCS (0x29) + PCA9685 (0x40)...");
    int encontrados1 = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0) {
            Serial.printf("[Diag]   responde en 0x%02X\n", addr);
            ++encontrados1;
        }
    }
    if (encontrados1 == 0) {
        Serial.println("[Diag]   NADA respondio en el bus 1 -- revisa cableado/alimentacion del TCS34725 y del PCA9685.");
    }
    Serial.println();

    g_tofOk = TofBringUp();
    if (g_tofOk) {
        Serial.println("[ToF] VL53L1X listo (0x29, bus 0, solo).");
    } else {
        Serial.println("[ToF] VL53L1X no responde. Reintentando en segundo plano.");
    }

    g_tcsOk = Tcs34725::Init();
    if (g_tcsOk) {
        Serial.println("[Color] TCS34725 delantero listo (0x29, bus 1).");
    } else {
        Serial.println("[Color] TCS34725 delantero no responde.");
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

    if ((!g_tofOk || !g_pcaOk || !g_tcsOk) && (uint32_t)(millis() - last_retry_ms) > 1000) {
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
        if (!g_tcsOk) g_tcsOk = Tcs34725::Init();
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

    // Imprime distancia + color a 10 Hz, sin bloquear el resto del loop().
    if ((uint32_t)(millis() - last_print_ms) >= 100) {
        last_print_ms = millis();

        char distStr[40];
        if (g_tofOk && g_tof.dataReady()) {
            const uint16_t mm = g_tof.read(false);
            const bool valid = !g_tof.timeoutOccurred() &&
                                g_tof.ranging_data.range_status == VL53L1X::RangeValid;
            if (g_tof.timeoutOccurred()) g_tofOk = false;
            snprintf(distStr, sizeof(distStr), "distancia=%4u mm  %s", mm, valid ? "OK" : "SIN MEDICION VALIDA");
        } else if (!g_tofOk) {
            snprintf(distStr, sizeof(distStr), "distancia=  ??? mm  SENSOR NO RESPONDE");
        } else {
            snprintf(distStr, sizeof(distStr), "distancia= (sin dato nuevo)");
        }

        char colorStr[48];
        Tcs34725::Rgbc color;
        if (g_tcsOk && Tcs34725::Read(color)) {
            snprintf(colorStr, sizeof(colorStr), "color=%-8s (c=%5u r=%5u g=%5u b=%5u)",
                ClasificarColor(color), color.c, color.r, color.g, color.b);
        } else {
            if (g_tcsOk) g_tcsOk = false;   // Read() falló -- se cae, reintenta en segundo plano
            snprintf(colorStr, sizeof(colorStr), "color=SENSOR NO RESPONDE");
        }

        Serial.printf("%-45s  %s\n", distStr, colorStr);
    }
}
