// ===========================================================================
//  Calibración IR — banco de pruebas (ESP32-S3)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  No es el firmware del rover (ese vive en firmware-esp32/), pero SÍ corre
//  sobre el mismo ESP32-S3 — el equipo usa un solo microcontrolador en todo
//  el proyecto. Este sketch se sube aparte, con nada más conectado que los
//  sensores bajo prueba (sin motores, sin PCA9685, sin TCS34725, sin LED
//  RGB), para caracterizarlos antes de fijar umbrales — ver ../README.md
//  para el procedimiento completo.
//
//  Pines: cableado físico actual confirmado por el equipo — sensores en
//  GPIO1/GPIO2, control compartido en GPIO42. Son los MISMOS GPIO que
//  QTR_LEFT_OUT/QTR_RIGHT_OUT/QTR_EMITTER_CTRL en
//  hardware/conexiones-esp32-s3.md (el diseño de vuelo), así que además de
//  válidos en ADC1 (GPIO1 y GPIO2 lo son) quedan documentados en dos
//  sitios a la vez.
//
//  AMBOS sensores son QTRX-HD-01A analógicos (no hay ningún sensor en modo
//  RC conectado ahora mismo). Los nombres "B"/"segundo sensor" en vez de
//  "RC" son a propósito, para no dar a entender un modo que ya no se usa.
//
//  Protocolo: este sketch no hace nada por su cuenta. Se queda esperando un
//  comando por serial y responde una lectura cada vez que lo recibe. La
//  orquestación (cuántos puntos, cuántas muestras, cuánto pausar entre
//  puntos, y el guardado a CSV) vive en el script de Python de al lado
//  (../calibrar_ir.py) — si cambias el formato de aquí, cámbialo allá
//  también.
//
//    Comando recibido : 'R'
//    Respuesta enviada : DATA,<analog_a>,<analog_b>,<generic>
//
// ===========================================================================

#include <Arduino.h>
#include <QTRSensors.h>

// =====================================================
// PIN ASSIGNMENT - ESP32-S3 (mismo chip que firmware-esp32/)
// =====================================================
//
// Cableado real del banco de pruebas. QTR_A_CTRL y QTR_B_CTRL son EL MISMO
// pin a propósito: las dos luces IR (una por sensor) comparten un solo
// cable de control físico, así que se encienden y apagan juntas.

// QTRX-HD-01A — sensor A
const uint8_t QTR_A_SENSOR = 1;   // = QTR_LEFT_OUT en hardware/conexiones-esp32-s3.md
const uint8_t QTR_A_CTRL   = 42;  // = QTR_EMITTER_CTRL en hardware/conexiones-esp32-s3.md

// QTRX-HD-01A — sensor B (mismo modelo que A, pin de control compartido)
const uint8_t QTR_B_SENSOR = 2;   // = QTR_RIGHT_OUT en hardware/conexiones-esp32-s3.md
const uint8_t QTR_B_CTRL   = QTR_A_CTRL;  // mismo pin físico que QTR_A_CTRL, ver nota arriba

// Generic IR
const uint8_t GENERIC_IR = 14;

// =====================================================
// QTR OBJECTS
// =====================================================

QTRSensors qtrA;
QTRSensors qtrB;

uint16_t valuesA[1];
uint16_t valuesB[1];

// =====================================================
// FUNCTIONS
// =====================================================

void readSensors()
{
    qtrA.read(valuesA);
    qtrB.read(valuesB);

    int genericValue = digitalRead(GENERIC_IR);

    /*
       IMPORTANT:
       Output format must remain:
       DATA,analog_a,analog_b,generic
    */
    Serial.print("DATA,");
    Serial.print(valuesA[0]);
    Serial.print(",");
    Serial.print(valuesB[0]);
    Serial.print(",");
    Serial.println(genericValue);
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // FIX: fijamos explícitamente la resolución del ADC del ESP32
    // (0-4095). Si no se fija, depende del valor por defecto del
    // core Arduino-ESP32 instalado, lo que puede cambiar tus datos
    // de calibración entre versiones del core o entre placas.
    analogReadResolution(12);

    // FIX: mismo problema que con la resolución, pero con la atenuación.
    // Sin fijarla explícitamente, dos sesiones de calibración del mismo
    // sensor sobre la misma superficie dieron lecturas de sensor B
    // desplazadas ~15-20 cuentas entre sí (ver el análisis que motivó este
    // cambio: comparando IR_NEGRO/IR_GRIS de 07:xx vs. 15:xx del
    // 2026-08-28, ningún umbral fijo servía bien para las dos sesiones a
    // la vez). 11 dB es la misma atenuación que usa firmware-esp32/ para
    // estos sensores.
    analogSetPinAttenuation(QTR_A_SENSOR, ADC_11db);
    analogSetPinAttenuation(QTR_B_SENSOR, ADC_11db);

    // -------------------------
    // QTRX-HD-01A — sensor A
    // -------------------------
    qtrA.setTypeAnalog();
    const uint8_t pinsA[] = { QTR_A_SENSOR };
    qtrA.setSensorPins(pinsA, 1);
    // Average multiple ADC readings
    qtrA.setSamplesPerSensor(8);
    qtrA.setEmitterPin(QTR_A_CTRL);

    // -------------------------
    // QTRX-HD-01A — sensor B
    // -------------------------
    qtrB.setTypeAnalog();
    const uint8_t pinsB[] = { QTR_B_SENSOR };
    qtrB.setSensorPins(pinsB, 1);
    // Average multiple ADC readings
    qtrB.setSamplesPerSensor(8);
    qtrB.setEmitterPin(QTR_B_CTRL);

    // -------------------------
    // Generic IR
    // -------------------------
    // NOTA: si el módulo IR genérico tiene salida en colector abierto
    // y "flota" cuando no detecta nada (lecturas inestables), cambia
    // esto a INPUT_PULLUP. Si el módulo ya trae su propia resistencia
    // pull-up/pull-down en la placa, deja INPUT como está.
    pinMode(GENERIC_IR, INPUT);

    Serial.println("READY");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    /*
       The ESP32 waits for commands from the computer.
       Command:
       R
       Response:
       DATA,analog_a,analog_b,generic
    */
    if (Serial.available())
    {
        char command = Serial.read();

        if (command == 'R')
        {
            readSensors();
        }

        // FIX: descarta cualquier byte extra (\r, \n, etc.) que haya
        // llegado junto al comando, para que no quede colgado en el
        // buffer y se procese por error en el siguiente ciclo.
        while (Serial.available())
        {
            Serial.read();
        }
    }
}
