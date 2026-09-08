// ===========================================================================
//  Prueba 07 — Caracterización interactiva de los 4 motores
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  OBJETIVO: accionar CADA motor por separado (delantero izq./der., trasero
//  izq./der.), tanto adelante como atrás, y dejar un log de lo que se probó
//  para poder compartirlo y confirmar (o corregir) el sentido de giro y el
//  cableado de cada uno — el mismo tipo de problema que ya resolvió el
//  intercambio de IN1/IN2 documentado en hardware/conexiones-esp32-s3.md
//  para el motor trasero izquierdo.
//
//  A diferencia de 03-motores-adelante (que arranca solo y a fondo, sin
//  forma de pararlo por software), este sketch NO mueve nada hasta que se
//  le manda un comando por el monitor serial. Reutiliza los mismos pines y
//  la misma definición de motores que 03 (incluido el intercambio IN1/IN2
//  del motor trasero izquierdo).
//
//  CÓMO SE USA — por el monitor serial (115200 baudios, línea + Enter):
//    motor <fl|fr|rl|rr|todos>   selecciona el motor objetivo (arranca en "todos")
//    adelante [duty]             motor(es) seleccionado(s) hacia adelante
//    atras [duty]                motor(es) seleccionado(s) hacia atrás
//    alto                        detiene el motor(es) seleccionado(s)
//    duty <n>                    cambia el duty por defecto (0-255, arranca en 200)
//    secuencia [duty] [ms]       rutina automática: prueba los 4 motores UNO A
//                                UNO, adelante y luego atrás, con pausas entre
//                                fases — genera un log limpio y completo
//    ?                           reimprime el menú de ayuda
//    (línea vacía)               reimprime el estado actual
//
//  Cada cambio de dirección imprime una línea "[LOG] ..." con timestamp,
//  motor, dirección y duty aplicados — son las líneas que hay que mirar (o
//  compartir) para revisar el comportamiento. Con "log2file" activado en
//  platformio.ini, TODA la sesión del monitor (comandos incluidos) queda
//  guardada en un .log dentro de esta misma carpeta.
//
//  ⚠️ ANTES DE ENCENDER: pon el robot sobre un soporte con las 4 ruedas al
//  aire, o en el piso con espacio libre alrededor — con "todos" y duty alto
//  el robot sí puede desplazarse.
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
//  [2] MOTORES — misma definición que 03-motores-adelante
// ===========================================================================

namespace Pwm {
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;   // el L298N calienta y pierde par a 20 kHz
    constexpr uint8_t  MOTOR_RESOLUTION = 8;      // duty 0..255
}

struct Motor {
    uint8_t in1, in2, en, ledc_channel;
    const char *nombre;
};

// kMotorFL: con el cableado físico actual, la rueda conectada a OUT1/OUT2
// del L298N izquierdo (la trasera izquierda del chasis) gira al revés
// respecto a las otras tres. Se resuelve intercambiando el ORDEN de los dos
// GPIO aquí mismo (ver la nota completa en 03-motores-adelante/src/main.cpp
// y en hardware/conexiones-esp32-s3.md). Si se recablea este motor para que
// coincida con los otros tres, basta con volver a poner IN1, IN2 en orden.
constexpr Motor kMotorFL = {Pins::L298N_L_IN2, Pins::L298N_L_IN1, Pins::L298N_L_ENA, 0, "FL"};
constexpr Motor kMotorRL = {Pins::L298N_L_IN3, Pins::L298N_L_IN4, Pins::L298N_L_ENB, 1, "RL"};
constexpr Motor kMotorFR = {Pins::L298N_R_IN1, Pins::L298N_R_IN2, Pins::L298N_R_ENA, 2, "FR"};
constexpr Motor kMotorRR = {Pins::L298N_R_IN3, Pins::L298N_R_IN4, Pins::L298N_R_ENB, 3, "RR"};

// Orden fijo usado para "todos" y para "secuencia": FL, FR, RL, RR.
constexpr Motor kMotores[4] = {kMotorFL, kMotorFR, kMotorRL, kMotorRR};

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
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
    PwmAttach(m.en, m.ledc_channel);
    PwmWrite(m.en, m.ledc_channel, 0);
}

enum class Dir : uint8_t { ALTO, ADELANTE, ATRAS };

const char *DirNombre(Dir d) {
    switch (d) {
        case Dir::ADELANTE: return "ADELANTE";
        case Dir::ATRAS:    return "ATRAS";
        default:            return "ALTO";
    }
}

// Aplica dirección+duty a UN motor. IN1=HIGH,IN2=LOW -> adelante (misma
// convención que MotorForwardFull() de 03-motores-adelante).
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

// ===========================================================================
//  [3] ESTADO + LOG
// ===========================================================================

// Índice 0..3 = un motor puntual (mismo orden que kMotores); 4 = "todos".
namespace Cal {
    constexpr uint8_t SEL_TODOS = 4;
    uint8_t sel      = SEL_TODOS;
    uint8_t duty      = 200;   // por defecto, no a fondo (255) — ver "duty <n>"
    Dir     dir_actual[4] = {Dir::ALTO, Dir::ALTO, Dir::ALTO, Dir::ALTO};
    uint8_t duty_actual[4] = {0, 0, 0, 0};
}

// Línea estructurada pensada para ser fácil de leer/filtrar en el .log
// compartido: timestamp, motor(es), dirección y duty aplicados.
void ImprimirLog(const char *motor_nombre, Dir dir, uint8_t duty) {
    DEBUG_LINK.printf("[LOG] t_ms=%lu motor=%s dir=%s duty=%u/255\n",
        (unsigned long)millis(), motor_nombre, DirNombre(dir), duty);
}

void ImprimirEstado() {
    const char *sel_nombre = (Cal::sel == Cal::SEL_TODOS) ? "TODOS" : kMotores[Cal::sel].nombre;
    DEBUG_LINK.printf("[Estado] motor_seleccionado=%s   duty_por_defecto=%u/255\n", sel_nombre, Cal::duty);
    for (uint8_t i = 0; i < 4; ++i) {
        DEBUG_LINK.printf("           %s: dir=%-8s duty=%u/255\n",
            kMotores[i].nombre, DirNombre(Cal::dir_actual[i]), Cal::duty_actual[i]);
    }
}

void ImprimirAyuda() {
    DEBUG_LINK.println("Comandos: motor <fl|fr|rl|rr|todos> | adelante [duty] | atras [duty] | alto | duty <n> | secuencia [duty] [ms] | ?");
}

// Aplica dirección+duty al motor (o los 4 motores) actualmente
// seleccionado(s), actualiza el estado y deja el log correspondiente.
void AplicarSeleccion(Dir dir, uint8_t duty) {
    if (Cal::sel == Cal::SEL_TODOS) {
        for (uint8_t i = 0; i < 4; ++i) {
            MotorAplicar(kMotores[i], dir, duty);
            Cal::dir_actual[i] = dir;
            Cal::duty_actual[i] = (dir == Dir::ALTO) ? 0 : duty;
        }
        ImprimirLog("TODOS", dir, duty);
    } else {
        const Motor &m = kMotores[Cal::sel];
        MotorAplicar(m, dir, duty);
        Cal::dir_actual[Cal::sel] = dir;
        Cal::duty_actual[Cal::sel] = (dir == Dir::ALTO) ? 0 : duty;
        ImprimirLog(m.nombre, dir, duty);
    }
}

// Detiene los 4 motores sin tocar Cal::sel (para transiciones internas,
// p. ej. entre fases de "secuencia").
void DetenerTodos() {
    for (uint8_t i = 0; i < 4; ++i) {
        MotorAplicar(kMotores[i], Dir::ALTO, 0);
        Cal::dir_actual[i] = Dir::ALTO;
        Cal::duty_actual[i] = 0;
    }
}

// Rutina automática: prueba los 4 motores uno a la vez, adelante y luego
// atrás, con una pausa de alto entre cada fase. Bloqueante a propósito —
// es un barrido de una sola vez, pensado para generar de corrido el log
// completo de caracterización de los 4 motores.
constexpr uint32_t PAUSA_ENTRE_FASES_MS = 500;

void CorrerSecuencia(uint8_t duty, uint32_t duracion_ms) {
    DEBUG_LINK.printf("[Secuencia] Iniciando: duty=%u/255, %lu ms por fase, %u motores.\n",
        duty, (unsigned long)duracion_ms, 4);

    for (uint8_t i = 0; i < 4; ++i) {
        const Motor &m = kMotores[i];
        DEBUG_LINK.printf("[Secuencia] ---- %s ----\n", m.nombre);

        MotorAplicar(m, Dir::ADELANTE, duty);
        Cal::dir_actual[i] = Dir::ADELANTE;
        Cal::duty_actual[i] = duty;
        ImprimirLog(m.nombre, Dir::ADELANTE, duty);
        delay(duracion_ms);

        MotorAplicar(m, Dir::ALTO, 0);
        Cal::dir_actual[i] = Dir::ALTO;
        Cal::duty_actual[i] = 0;
        ImprimirLog(m.nombre, Dir::ALTO, 0);
        delay(PAUSA_ENTRE_FASES_MS);

        MotorAplicar(m, Dir::ATRAS, duty);
        Cal::dir_actual[i] = Dir::ATRAS;
        Cal::duty_actual[i] = duty;
        ImprimirLog(m.nombre, Dir::ATRAS, duty);
        delay(duracion_ms);

        MotorAplicar(m, Dir::ALTO, 0);
        Cal::dir_actual[i] = Dir::ALTO;
        Cal::duty_actual[i] = 0;
        ImprimirLog(m.nombre, Dir::ALTO, 0);
        delay(PAUSA_ENTRE_FASES_MS);
    }

    DEBUG_LINK.println("[Secuencia] Lista. Los 4 motores quedaron detenidos.");
    ImprimirEstado();
}

// ===========================================================================
//  [4] COMANDOS
// ===========================================================================

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

    if (line.startsWith("motor ")) {
        String objetivo = line.substring(6);
        objetivo.trim();
        objetivo.toLowerCase();
        if (objetivo == "fl") Cal::sel = 0;
        else if (objetivo == "fr") Cal::sel = 1;
        else if (objetivo == "rl") Cal::sel = 2;
        else if (objetivo == "rr") Cal::sel = 3;
        else if (objetivo == "todos") Cal::sel = Cal::SEL_TODOS;
        else {
            DEBUG_LINK.printf("[Error] motor no reconocido: \"%s\" (usa fl|fr|rl|rr|todos)\n", objetivo.c_str());
            return;
        }
        ImprimirEstado();
        return;
    }

    if (line == "adelante" || line.startsWith("adelante ")) {
        uint8_t duty = Cal::duty;
        if (line.length() > 9) duty = (uint8_t)constrain(line.substring(9).toInt(), 0, 255);
        AplicarSeleccion(Dir::ADELANTE, duty);
        return;
    }

    if (line == "atras" || line.startsWith("atras ")) {
        uint8_t duty = Cal::duty;
        if (line.length() > 6) duty = (uint8_t)constrain(line.substring(6).toInt(), 0, 255);
        AplicarSeleccion(Dir::ATRAS, duty);
        return;
    }

    if (line == "alto") {
        AplicarSeleccion(Dir::ALTO, 0);
        return;
    }

    if (line.startsWith("duty ")) {
        Cal::duty = (uint8_t)constrain(line.substring(5).toInt(), 0, 255);
        ImprimirEstado();
        return;
    }

    if (line == "secuencia" || line.startsWith("secuencia ")) {
        uint8_t duty = Cal::duty;
        uint32_t duracion_ms = 1500;
        if (line.length() > 10) {
            String args = line.substring(10);
            args.trim();
            int espacio = args.indexOf(' ');
            if (espacio < 0) {
                duty = (uint8_t)constrain(args.toInt(), 0, 255);
            } else {
                duty = (uint8_t)constrain(args.substring(0, espacio).toInt(), 0, 255);
                duracion_ms = (uint32_t)max(200L, args.substring(espacio + 1).toInt());
            }
        }
        CorrerSecuencia(duty, duracion_ms);
        return;
    }

    DEBUG_LINK.printf("[Error] comando no reconocido: \"%s\"\n", line.c_str());
    ImprimirAyuda();
}

// ===========================================================================
//  [5] setup() / loop()
// ===========================================================================

void setup() {
    DEBUG_LINK.begin(115200);
    delay(200);
    DEBUG_LINK.println("\nPrueba 07 - Caracterizacion interactiva de los 4 motores");

    for (uint8_t i = 0; i < 4; ++i) {
        MotorSetup(kMotores[i]);
    }
    DetenerTodos();

    DEBUG_LINK.println("[Setup] Los 4 motores quedan detenidos. No se mueve nada hasta que mandes un comando.");
    ImprimirAyuda();
    ImprimirEstado();
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
