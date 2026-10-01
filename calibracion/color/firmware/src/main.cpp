// ===========================================================================
//  Calibración de color — banco de pruebas (ESP32-S3)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  No es el firmware del rover (ese vive en firmware-esp32/), pero SÍ corre
//  sobre el mismo ESP32-S3 — el equipo usa un solo microcontrolador en todo
//  el proyecto. Este sketch se sube aparte, con nada más conectado que los
//  DOS TCS34725 bajo prueba (sin motores, sin PCA9685, sin QTR), para
//  caracterizarlos antes de fijar los umbrales de ClassifyColor() en
//  firmware-esp32/ — ver ../README.md para el procedimiento completo.
//
//  Driver TCS34725 copiado TAL CUAL de firmware-esp32/src/main.cpp (mismo
//  ATIME/GAIN, mismo protocolo de registros) a propósito: si este banco
//  midiera con una configuración distinta a la que usa el robot de verdad,
//  los datos capturados no servirían para calibrar nada. Es la misma razón
//  por la que calibracion/reflectancia/firmware/ tuvo que empezar a fijar la
//  atenuación del ADC explícitamente — no repetir ese error aquí.
//
//  Pines: cableado físico según hardware/conexiones-esp32-s3.md y v9 — los
//  DOS TCS34725 cuelgan de un multiplexor TCA9548A (0x71) en el bus I2C nº1
//  (GPIO47/48): delantero en el canal 0, trasero en el canal 1. Los dos
//  comparten dirección fija 0x29 — el multiplexor es lo que los separa.
//  Antes de CADA lectura se selecciona el canal del sensor pedido, igual que
//  hace standalones/v9-tira-sensor-trasero/: si este banco midiera por otra
//  ruta que el robot, los datos no servirían para calibrar. El bus I2C nº0
//  (ToF) ya no se usa aquí.
//
//  SE CALIBRA UN SENSOR A LA VEZ, ELEGIDO POR COMANDO — no hace falta tocar
//  este archivo ni reflashear para cambiar de delantero a trasero: lo decide
//  quien llama, con el comando 'F' o 'T' (ver el protocolo abajo). Los dos
//  LED se encienden siempre al arrancar, sin importar cuál se vaya a usar.
//
//  Protocolo: este sketch no hace nada por su cuenta. Se queda esperando un
//  comando por serial y responde una lectura del sensor pedido cada vez que
//  lo recibe. La orquestación (qué superficie, cuántos puntos, cuántas
//  muestras, el guardado a CSV) vive en ../calibrar_color.py — si cambias
//  el formato de aquí, cámbialo allá también.
//
//    Comando recibido : 'F' (delantero, canal 0 del mux)  ó  'T' (trasero, canal 1)
//    Respuesta enviada : DATA,<ok>,<clear>,<r>,<g>,<b>
//
//    <ok> es 0 si el TCS34725 pedido no respondió (no conectado, cable
//    flojo, etc.) — en ese caso los otros 4 campos son 0 y hay que
//    ignorarlos, no tratarlos como una lectura real de "sin luz". El estado
//    de inicialización de cada sensor se recuerda por separado, así que un
//    sensor caído no afecta al otro ni se reintenta de más.
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>

// ===========================================================================
//  PINES — idénticos a hardware/conexiones-esp32-s3.md y a firmware-esp32
// ===========================================================================

namespace Pins {
    // Bus I2C nº1 — multiplexor TCA9548A, con los dos TCS34725 detrás.
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    // LED blanco de iluminación de cada sensor — activo en alto. Encendido
    // fijo: la clasificación de color no puede depender de la luz del salón.
    constexpr uint8_t TCS_LED_FRONT = 18;
    constexpr uint8_t TCS_LED_REAR  = 41;   // igual que en v9 (GPIO liberado por la tira WS2812)
}

namespace I2CAddr {
    constexpr uint8_t TCS34725 = 0x29;   // fija — por eso el multiplexor
    // TCA9548A con A0 a 3.3V: de fábrica (0x70) choca con el all-call del PCA9685.
    constexpr uint8_t MULTIPLEXOR = 0x71;
}

namespace MuxChannel {
    constexpr uint8_t FRONT = 0;
    constexpr uint8_t REAR  = 1;
}

// Un solo registro: escribir un byte con el bit del canal en 1 lo activa y
// apaga los demás. Copia de Multiplexor:: de v9.
bool SelectMuxChannel(uint8_t channel) {
    Wire.beginTransmission(I2CAddr::MULTIPLEXOR);
    Wire.write((uint8_t)(1u << channel));
    return Wire.endTransmission() == 0;
}

// ===========================================================================
//  DRIVER TCS34725 (sobre Wire, bus nº1) — copia exacta del namespace Tcs34725 de firmware-esp32/
// ===========================================================================

namespace Tcs34725 {
    constexpr uint8_t CMD_BIT      = 0x80;   // todo acceso a registro lleva este bit
    constexpr uint8_t CMD_AUTO_INC = 0x20;

    constexpr uint8_t REG_ENABLE  = 0x00;
    constexpr uint8_t REG_ATIME   = 0x01;
    constexpr uint8_t REG_CONTROL = 0x0F;
    constexpr uint8_t REG_ID      = 0x12;
    constexpr uint8_t REG_CDATAL  = 0x14;   // luego R, G, B consecutivos

    constexpr uint8_t ENABLE_PON = 0x01;    // enciende el oscilador interno
    constexpr uint8_t ENABLE_AEN = 0x02;    // habilita el conversor RGBC

    // Mismos valores que firmware-esp32/: 24 ms de integración, ganancia 4x.
    constexpr uint8_t ATIME_24MS = 0xEB;
    constexpr uint8_t GAIN_4X    = 0x01;

    struct Rgbc { uint16_t c = 0, r = 0, g = 0, b = 0; };

    bool WriteReg(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(I2CAddr::TCS34725);
        Wire.write(CMD_BIT | reg);
        Wire.write(value);
        return Wire.endTransmission() == 0;
    }

    bool ReadReg(uint8_t reg, uint8_t &out) {
        Wire.beginTransmission(I2CAddr::TCS34725);
        Wire.write(CMD_BIT | reg);
        if (Wire.endTransmission() != 0) return false;
        if (Wire.requestFrom((int)I2CAddr::TCS34725, 1) != 1) return false;
        out = (uint8_t)Wire.read();
        return true;
    }

    bool Init() {
        uint8_t id = 0;
        if (!ReadReg(REG_ID, id)) return false;
        // 0x44 = TCS34725, 0x4D = TCS34727. Cualquier otra cosa no es el sensor.
        if (id != 0x44 && id != 0x4D) return false;

        if (!WriteReg(REG_ATIME, ATIME_24MS)) return false;
        if (!WriteReg(REG_CONTROL, GAIN_4X)) return false;
        if (!WriteReg(REG_ENABLE, ENABLE_PON)) return false;
        delay(3);                                   // arranque del oscilador
        return WriteReg(REG_ENABLE, ENABLE_PON | ENABLE_AEN);
    }

    bool Read(Rgbc &out) {
        Wire.beginTransmission(I2CAddr::TCS34725);
        Wire.write(CMD_BIT | CMD_AUTO_INC | REG_CDATAL);
        if (Wire.endTransmission() != 0) return false;
        if (Wire.requestFrom((int)I2CAddr::TCS34725, 8) != 8) return false;

        // Los cuatro canales llegan como uint16 little-endian, en orden C R G B.
        out.c = (uint16_t)(Wire.read() | (Wire.read() << 8));
        out.r = (uint16_t)(Wire.read() | (Wire.read() << 8));
        out.g = (uint16_t)(Wire.read() | (Wire.read() << 8));
        out.b = (uint16_t)(Wire.read() | (Wire.read() << 8));
        return true;
    }
}

// ===========================================================================
//  ESTADO Y LECTURA
// ===========================================================================

// Un estado de inicialización POR SENSOR, no uno solo: con la selección por
// comando, cualquiera de los dos puede pedirse en cualquier momento, y que
// el trasero no responda no debería hacer que el delantero se reintente de
// más (ni viceversa).
bool g_frontOk = false;
bool g_backOk  = false;

// Si el sensor no estaba OK, reintenta inicializarlo antes de leer. Sin
// esto, un sensor que no respondió al arrancar (cable conectado después,
// por ejemplo) se quedaría muerto para siempre en esta sesión.
bool ReadOrReinit(uint8_t muxChannel, Tcs34725::Rgbc &out, bool &okState) {
    // Sin canal seleccionado no hay sensor al que hablarle.
    if (!SelectMuxChannel(muxChannel)) {
        okState = false;
        out = Tcs34725::Rgbc{};
        return false;
    }
    if (!okState) {
        okState = Tcs34725::Init();
    }
    if (okState && Tcs34725::Read(out)) {
        return true;
    }
    okState = false;
    out = Tcs34725::Rgbc{};
    return false;
}

void readSensor(uint8_t muxChannel, bool &okState) {
    Tcs34725::Rgbc s;
    const bool ok = ReadOrReinit(muxChannel, s, okState);

    // IMPORTANTE: el formato de salida debe mantenerse en sincronía con
    // ../calibrar_color.py: DATA,ok,clear,r,g,b
    Serial.print("DATA,");
    Serial.print(ok ? 1 : 0); Serial.print(",");
    Serial.print(s.c); Serial.print(",");
    Serial.print(s.r); Serial.print(",");
    Serial.print(s.g); Serial.print(",");
    Serial.println(s.b);
}

// ===========================================================================
// SETUP
// ===========================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);

    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);
    pinMode(Pins::TCS_LED_REAR, OUTPUT);
    digitalWrite(Pins::TCS_LED_REAR, HIGH);

    // Se imprime si el multiplexor responde: sin él ninguno de los dos
    // sensores puede leerse, y es el primer sospechoso si todo da ok=0.
    if (!SelectMuxChannel(MuxChannel::FRONT)) {
        Serial.println("[Setup] El multiplexor TCA9548A (0x71) no responde en el bus I2C 1.");
    }
    g_frontOk = SelectMuxChannel(MuxChannel::FRONT) && Tcs34725::Init();
    g_backOk  = SelectMuxChannel(MuxChannel::REAR)  && Tcs34725::Init();

    Serial.println("READY");
}

// ===========================================================================
// LOOP
// ===========================================================================

void loop() {
    /*
       El ESP32 espera comandos de la computadora.
       Comando:
       F  -> lee el sensor DELANTERO (canal 0 del multiplexor)
       T  -> lee el sensor TRASERO   (canal 1 del multiplexor)
       Respuesta:
       DATA,ok,clear,r,g,b
    */
    if (Serial.available()) {
        const char command = Serial.read();

        if (command == 'F') {
            readSensor(MuxChannel::FRONT, g_frontOk);
        } else if (command == 'T') {
            readSensor(MuxChannel::REAR, g_backOk);
        }

        // Descarta cualquier byte extra (\r, \n, etc.) que haya llegado
        // junto al comando, para que no quede colgado en el buffer y se
        // procese por error en el siguiente ciclo.
        while (Serial.available()) {
            Serial.read();
        }
    }
}
