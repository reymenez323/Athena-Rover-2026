// ===========================================================================
//  Calibración IR — banco de pruebas (ESP32-S3)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  No es el firmware del rover (ese vive en firmware-esp32/), pero SÍ corre
//  sobre el mismo ESP32-S3 — el equipo usa un solo microcontrolador en todo
//  el proyecto. Este sketch se sube aparte, con nada más conectado que el
//  sensor bajo prueba (sin motores, sin PCA9685, sin TCS34725, sin LED RGB),
//  para caracterizar un QTRX-HD-01A/RC de banco antes de fijar umbrales —
//  ver ../README.md para el procedimiento completo.
//
//  Pines: idénticos al cableado físico ya armado por el equipo — no se
//  tocan aquí. GPIO35 cae en el rango que hardware/conexiones-esp32-s3.md
//  marca como prohibido en el S3 (PSRAM octal); el equipo confirmó que en
//  su módulo específico funciona sin problema en la práctica, así que se
//  deja tal cual en vez de recablear.
//
//  Protocolo: este sketch no hace nada por su cuenta. Se queda esperando un
//  comando por serial y responde una lectura cada vez que lo recibe. La
//  orquestación (cuántos puntos, cuántas muestras, cuánto pausar entre
//  puntos, y el guardado a CSV) vive en el script de Python de al lado
//  (../calibrar_ir.py) — si cambias el formato de aquí, cámbialo allá
//  también.
//
//    Comando recibido : 'R'
//    Respuesta enviada : DATA,<analog>,<rc>,<generic>
//
// ===========================================================================

#include <Arduino.h>
#include <QTRSensors.h>

// =====================================================
// PIN ASSIGNMENT - ESP32-S3 (mismo chip que firmware-esp32/)
// =====================================================
//
// Cableado real del banco de pruebas. QTR_A_CTRL y QTR_RC_CTRL son EL MISMO
// pin a propósito: las dos luces IR (la del sensor analógico y la del RC)
// comparten un solo cable de control físico, así que se encienden y apagan
// juntas — no son dos señales independientes.

// QTRX-HD-01A - ANALOG
const uint8_t QTR_A_SENSOR = 35;
const uint8_t QTR_A_CTRL   = 25;

// QTRX-HD-01RC - RC
const uint8_t QTR_RC_SENSOR = 4;
const uint8_t QTR_RC_CTRL   = QTR_A_CTRL;  // mismo pin físico que QTR_A_CTRL, ver nota arriba

// Generic IR
const uint8_t GENERIC_IR = 14;

// =====================================================
// QTR OBJECTS
// =====================================================

QTRSensors qtrAnalog;
QTRSensors qtrRC;

uint16_t analogValues[1];
uint16_t rcValues[1];

// =====================================================
// FUNCTIONS
// =====================================================

void readSensors()
{
    qtrAnalog.read(analogValues);
    qtrRC.read(rcValues);

    int genericValue = digitalRead(GENERIC_IR);

    /*
       IMPORTANT:
       Output format must remain:
       DATA,analog,rc,generic
    */
    Serial.print("DATA,");
    Serial.print(analogValues[0]);
    Serial.print(",");
    Serial.print(rcValues[0]);
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

    // -------------------------
    // Analog QTRX-HD-01A
    // -------------------------
    qtrAnalog.setTypeAnalog();
    const uint8_t analogPins[] = { QTR_A_SENSOR };
    qtrAnalog.setSensorPins(analogPins, 1);
    // Average multiple ADC readings
    qtrAnalog.setSamplesPerSensor(8);
    qtrAnalog.setEmitterPin(QTR_A_CTRL);

    // -------------------------
    // RC QTRX-HD-01RC
    // -------------------------
    qtrRC.setTypeRC();
    const uint8_t rcPins[] = { QTR_RC_SENSOR };
    qtrRC.setSensorPins(rcPins, 1);
    // Maximum discharge measurement
    qtrRC.setTimeout(5000);
    qtrRC.setEmitterPin(QTR_RC_CTRL);

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
       DATA,analog,rc,generic
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
