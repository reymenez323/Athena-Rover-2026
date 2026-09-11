// ===========================================================================
//  Prueba 08 — Calibración interactiva del giro en su propio eje
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  OBJETIVO: encontrar, con el chasis y los motores ACTUALES (ya con el fix
//  de nombres FL/RL y el motor trasero derecho reemplazado), cuántos ms de
//  giro a fondo equivalen a un grado -- para poder calibrar el giro de
//  esquive tras soltar la caja (ver Mission::Phase::ESQUIVAR_CAJA en
//  standalones/v6-mision-completa/src/main.cpp).
//
//  POR QUÉ HACE FALTA UNA CALIBRACIÓN NUEVA: v5-agarrar-bandera había
//  medido en banco kMsPorGrado = 3000ms/180° (2026-09-07), pero esa medida
//  es de ANTES del fix de nombres FL/RL (pruebas-platformio/07-caracteriza-
//  cion-motores) y de ANTES de reemplazar el motor trasero derecho -- con
//  la fricción/torque del chasis ya cambiados, ese número no es confiable
//  sin volver a medirlo.
//
//  CÓMO SE USA — por el monitor serial (115200 baudios, línea + Enter):
//    girar <izq|der> <grados>    gira esa cantidad de grados con la
//                                 calibración actual (ms_por_grado * grados)
//    girarms <izq|der> <ms>      gira por un tiempo crudo, SIN convertir --
//                                 úsalo para medir: corre esto, mide el
//                                 ángulo real con un transportador o marcas
//                                 en el piso, y pásaselo a "calibrar"
//    calibrar <ms> <grados>      fija ms_por_grado = ms / grados (a partir
//                                 de una corrida de "girarms" ya medida)
//    msgrado <valor>             fija ms_por_grado directo, si ya lo sabes
//    duty <n>                    cambia el duty de AMBOS lados (0-255,
//                                 arranca en 255/255 -- a fondo, mismo
//                                 criterio que v5)
//    dutylados <izq> <der>       duty INDEPENDIENTE por lado físico (izq =
//                                 FL+RL, der = FR+RR) -- útil si un lado es
//                                 mecánicamente más débil que el otro y el
//                                 giro sale desparejo; bajarle al lado
//                                 fuerte a veces empareja mejor que subirle
//                                 al débil, que ya puede estar a fondo
//    alto                        detiene los 4 motores
//    ?                           reimprime el menú de ayuda
//    (línea vacía)               reimprime el estado actual
//
//  "izq"/"der" es el sentido en que gira el CHASIS (visto desde arriba),
//  no un motor en particular. Si al primer "girar der" ves que en realidad
//  giró a la izquierda, es solo cuestión de nomenclatura -- usa el comando
//  contrario, el giro en sí ya está bien.
//
//  ⚠️ ANTES DE ENCENDER: pon el robot en el piso con espacio libre
//  alrededor para girar sin chocar con nada -- a diferencia de los sketches
//  de motores rectos, este SÍ arranca girando en cuanto mandas "girar".
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

#define DEBUG_LINK Serial0   // consola + comandos por el puerto UART del DevKit

// ===========================================================================
//  [2] MOTORES — misma definición ya corregida de 07-caracterizacion-motores
//  (kMotorRL en OUT1/OUT2 con el orden IN2,IN1 invertido -- ese es el motor
//  físicamente trasero izq. que gira al revés de los otros tres -- y
//  kMotorFL en OUT3/OUT4 -- ver la nota larga en 07-caracterizacion-motores
//  y en hardware/conexiones-esp32-s3.md).
// ===========================================================================

namespace Pwm {
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;   // el L298N calienta y pierde par a 20 kHz
    constexpr uint8_t  MOTOR_RESOLUTION = 8;      // duty 0..255
}

struct Motor {
    uint8_t in1, in2, en, ledc_channel;
    const char *nombre;
};

constexpr Motor kMotorRL = {Pins::L298N_L_IN2, Pins::L298N_L_IN1, Pins::L298N_L_ENA, 0, "RL"};
constexpr Motor kMotorFL = {Pins::L298N_L_IN3, Pins::L298N_L_IN4, Pins::L298N_L_ENB, 1, "FL"};
constexpr Motor kMotorFR = {Pins::L298N_R_IN1, Pins::L298N_R_IN2, Pins::L298N_R_ENA, 2, "FR"};
constexpr Motor kMotorRR = {Pins::L298N_R_IN3, Pins::L298N_R_IN4, Pins::L298N_R_ENB, 3, "RR"};

constexpr Motor kMotores[4] = {kMotorFL, kMotorFR, kMotorRL, kMotorRR};

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
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
    PwmAttach(m.en, m.ledc_channel);
    PwmWrite(m.en, m.ledc_channel, 0);
}

enum class Dir : uint8_t { ALTO, ADELANTE, ATRAS };

void MotorAplicar(const Motor &m, Dir dir, uint8_t duty) {
    switch (dir) {
        case Dir::ADELANTE:
            digitalWrite(m.in1, HIGH);
            digitalWrite(m.in2, LOW);
            PwmWrite(m.en, m.ledc_channel, duty);
            break;
        case Dir::ATRAS:
            digitalWrite(m.in1, LOW);
            digitalWrite(m.in2, HIGH);
            PwmWrite(m.en, m.ledc_channel, duty);
            break;
        case Dir::ALTO:
        default:
            digitalWrite(m.in1, LOW);
            digitalWrite(m.in2, LOW);
            PwmWrite(m.en, m.ledc_channel, 0);
            break;
    }
}

void DetenerTodos() {
    for (uint8_t i = 0; i < 4; ++i) MotorAplicar(kMotores[i], Dir::ALTO, 0);
}

// Pivotea sobre su propio eje: un lado ADELANTE, el otro ATRAS -- duty
// INDEPENDIENTE por lado físico (izq = FL+RL, der = FR+RR), no por sentido
// de giro -- así el lado mecánicamente más débil (o el que hay que frenar
// un poco para que el giro no salga desparejo) se ajusta igual sin
// importar si esta vez le toca ir ADELANTE o ATRAS. Mismo primitivo base
// que Phase::GIRAR en v5-agarrar-bandera (SetDrive(motor,
// +kVelocidadGiroMax, -kVelocidadGiroMax)), pero con dos duty en vez de uno.
void AplicarGiro(char sentido, uint8_t duty_izq, uint8_t duty_der) {
    const bool derecha = (sentido == 'd');
    MotorAplicar(kMotorFL, derecha ? Dir::ADELANTE : Dir::ATRAS, duty_izq);
    MotorAplicar(kMotorRL, derecha ? Dir::ADELANTE : Dir::ATRAS, duty_izq);
    MotorAplicar(kMotorFR, derecha ? Dir::ATRAS : Dir::ADELANTE, duty_der);
    MotorAplicar(kMotorRR, derecha ? Dir::ATRAS : Dir::ADELANTE, duty_der);
}

// ===========================================================================
//  [3] CALIBRACIÓN + COMANDOS
// ===========================================================================

namespace Cal {
    // Punto de partida: valor bench-confirmado de v5-agarrar-bandera
    // (2026-09-07), de ANTES del fix FL/RL y del reemplazo del motor RR --
    // tratar como referencia, no como calibrado para el chasis actual.
    float   ms_por_grado = 3000.0f / 180.0f;
    // Duty por lado físico (izq = FL+RL, der = FR+RR), no por sentido de
    // giro -- ver AplicarGiro(). Arrancan iguales, a fondo; "dutylados" los
    // desacopla para compensar un lado más débil que el otro.
    uint8_t duty_izq = 255;
    uint8_t duty_der = 255;
}

void ImprimirEstado() {
    DEBUG_LINK.printf("[Estado] ms_por_grado=%.3f (90 grados = %lu ms)   duty_izq=%u/255  duty_der=%u/255\n",
        Cal::ms_por_grado, (unsigned long)(90.0f * Cal::ms_por_grado), Cal::duty_izq, Cal::duty_der);
}

void ImprimirAyuda() {
    DEBUG_LINK.println("Comandos: girar <izq|der> <grados> | girarms <izq|der> <ms> | calibrar <ms> <grados> | msgrado <valor> | duty <n> | dutylados <izq> <der> | alto | ?");
}

void ImprimirLogGiro(const char *modo, char sentido, uint32_t ms, uint8_t duty_izq, uint8_t duty_der) {
    DEBUG_LINK.printf("[LOG] t_ms=%lu %s sentido=%s duracion_ms=%lu duty_izq=%u/255 duty_der=%u/255\n",
        (unsigned long)millis(), modo, (sentido == 'd') ? "DERECHA" : "IZQUIERDA",
        (unsigned long)ms, duty_izq, duty_der);
}

void EjecutarGiro(const char *modo, char sentido, uint32_t ms, uint8_t duty_izq, uint8_t duty_der) {
    ImprimirLogGiro(modo, sentido, ms, duty_izq, duty_der);
    AplicarGiro(sentido, duty_izq, duty_der);
    delay(ms);
    DetenerTodos();
    DEBUG_LINK.println("[Giro] listo, motores detenidos.");
}

// Procesa una línea de comando ya sin '\r'/'\n' en los extremos.
void ManejarComando(String line) {
    line.trim();
    if (line.length() == 0) {
        ImprimirEstado();
        return;
    }

    if (line == "?") {
        ImprimirAyuda();
        return;
    }

    if (line == "alto") {
        DetenerTodos();
        DEBUG_LINK.println("[Giro] detenido.");
        return;
    }

    if (line.startsWith("duty ")) {
        const uint8_t d = (uint8_t)constrain(line.substring(5).toInt(), 0, 255);
        Cal::duty_izq = d;
        Cal::duty_der = d;
        ImprimirEstado();
        return;
    }

    if (line.startsWith("dutylados ")) {
        String args = line.substring(10);
        args.trim();
        int espacio = args.indexOf(' ');
        if (espacio < 0) {
            DEBUG_LINK.println("[Error] uso: dutylados <izq> <der>");
            return;
        }
        Cal::duty_izq = (uint8_t)constrain(args.substring(0, espacio).toInt(), 0, 255);
        Cal::duty_der = (uint8_t)constrain(args.substring(espacio + 1).toInt(), 0, 255);
        ImprimirEstado();
        return;
    }

    if (line.startsWith("msgrado ")) {
        float valor = line.substring(8).toFloat();
        if (valor <= 0) {
            DEBUG_LINK.println("[Error] ms_por_grado debe ser positivo.");
            return;
        }
        Cal::ms_por_grado = valor;
        ImprimirEstado();
        return;
    }

    if (line.startsWith("calibrar ")) {
        String args = line.substring(9);
        args.trim();
        int espacio = args.indexOf(' ');
        if (espacio < 0) {
            DEBUG_LINK.println("[Error] uso: calibrar <ms_usado> <grados_medidos>");
            return;
        }
        float ms = args.substring(0, espacio).toFloat();
        float grados = args.substring(espacio + 1).toFloat();
        if (ms <= 0 || grados <= 0) {
            DEBUG_LINK.println("[Error] ms y grados deben ser positivos (usa el valor absoluto del angulo medido).");
            return;
        }
        Cal::ms_por_grado = ms / grados;
        DEBUG_LINK.printf("[Calibracion] ms_por_grado actualizado a %.3f (a partir de %.0f ms / %.1f grados medidos)\n",
            Cal::ms_por_grado, ms, grados);
        ImprimirEstado();
        return;
    }

    if (line.startsWith("girarms ")) {
        String args = line.substring(8);
        args.trim();
        int espacio = args.indexOf(' ');
        if (espacio < 0) {
            DEBUG_LINK.println("[Error] uso: girarms <izq|der> <ms>");
            return;
        }
        String sentidoStr = args.substring(0, espacio);
        sentidoStr.trim();
        sentidoStr.toLowerCase();
        if (sentidoStr != "izq" && sentidoStr != "der") {
            DEBUG_LINK.println("[Error] sentido debe ser \"izq\" o \"der\".");
            return;
        }
        uint32_t ms = (uint32_t)max(0L, args.substring(espacio + 1).toInt());
        EjecutarGiro("girarms", sentidoStr[0], ms, Cal::duty_izq, Cal::duty_der);
        DEBUG_LINK.println("[Giro] mide el angulo real (transportador o marcas en el piso) y usa \"calibrar <ms> <grados_medidos>\" para fijar la calibracion.");
        return;
    }

    if (line.startsWith("girar ")) {
        String args = line.substring(6);
        args.trim();
        int espacio = args.indexOf(' ');
        if (espacio < 0) {
            DEBUG_LINK.println("[Error] uso: girar <izq|der> <grados>");
            return;
        }
        String sentidoStr = args.substring(0, espacio);
        sentidoStr.trim();
        sentidoStr.toLowerCase();
        if (sentidoStr != "izq" && sentidoStr != "der") {
            DEBUG_LINK.println("[Error] sentido debe ser \"izq\" o \"der\".");
            return;
        }
        float grados = args.substring(espacio + 1).toFloat();
        if (grados <= 0) {
            DEBUG_LINK.println("[Error] grados debe ser positivo.");
            return;
        }
        uint32_t ms = (uint32_t)(grados * Cal::ms_por_grado);
        EjecutarGiro("girar", sentidoStr[0], ms, Cal::duty_izq, Cal::duty_der);
        return;
    }

    DEBUG_LINK.printf("[Error] comando no reconocido: \"%s\"\n", line.c_str());
    ImprimirAyuda();
}

// ===========================================================================
//  [4] setup() / loop()
// ===========================================================================

void setup() {
    DEBUG_LINK.begin(115200);
    delay(200);
    DEBUG_LINK.println("\nPrueba 08 - Calibracion interactiva del giro en su propio eje");

    for (uint8_t i = 0; i < 4; ++i) {
        MotorSetup(kMotores[i]);
    }
    DetenerTodos();

    DEBUG_LINK.println("[Setup] Motores detenidos. No gira nada hasta que mandes un comando.");
    ImprimirAyuda();
    ImprimirEstado();
}

// Algunos monitores seriales no mandan '\n'/'\r' al enviar texto. Sin este
// respaldo, un comando sin salto de línea se queda esperando en el buffer
// para siempre -- mismo criterio que 07-caracterizacion-motores.
constexpr uint32_t LINE_IDLE_TIMEOUT_MS = 200;

void loop() {
    static String line;
    static uint32_t last_byte_ms = 0;

    while (DEBUG_LINK.available() > 0) {
        char c = (char)DEBUG_LINK.read();
        if (c == '\n' || c == '\r') {
            if (line.length() > 0) {
                ManejarComando(line);
                line = "";
            }
        } else {
            line += c;
            last_byte_ms = millis();
        }
    }

    if (line.length() > 0 && (uint32_t)(millis() - last_byte_ms) > LINE_IDLE_TIMEOUT_MS) {
        ManejarComando(line);
        line = "";
    }
}
