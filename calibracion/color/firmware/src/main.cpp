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
//  Pines: cableado físico según hardware/conexiones-esp32-s3.md — bus I2C
//  nº0 (delantero, compartido con el PCA9685 en el diseño de vuelo, pero
//  aquí solo tiene el TCS34725 conectado) y bus I2C nº1 (trasero, dedicado).
//  Los dos sensores comparten dirección fija 0x29 — por eso van en buses
//  separados, no hay forma de diferenciarlos en el mismo bus.
//
//  POR AHORA SOLO SE CALIBRA EL SENSOR DELANTERO — ver SENSOR_ES_DELANTERO
//  más abajo, es el ÚNICO lugar del archivo que hay que tocar para pasar al
//  trasero más adelante (una constante, no una reescritura). Los dos buses
//  I2C y los dos LED se siguen inicializando siempre sin importar cuál esté
//  activo, a propósito: así cambiar la constante es de verdad lo único que
//  hace falta, no hay que acordarse de mover nada más.
//
//  Protocolo: este sketch no hace nada por su cuenta. Se queda esperando un
//  comando por serial y responde una lectura del sensor ACTIVO cada vez que
//  lo recibe. La orquestación (qué superficie, cuántos puntos, cuántas
//  muestras, el guardado a CSV) vive en ../calibrar_color.py — si cambias
//  el formato de aquí, cámbialo allá también.
//
//    Comando recibido : 'R'
//    Respuesta enviada : DATA,<ok>,<clear>,<r>,<g>,<b>
//
//    <ok> es 0 si el TCS34725 activo no respondió (no conectado, cable
//    flojo, etc.) — en ese caso los otros 4 campos son 0 y hay que
//    ignorarlos, no tratarlos como una lectura real de "sin luz".
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>

// ===========================================================================
//  PINES — idénticos a hardware/conexiones-esp32-s3.md y a firmware-esp32
// ===========================================================================

namespace Pins {
    // Bus I2C nº0 — TCS34725 DELANTERO (en el diseño de vuelo comparte este
    // bus con el PCA9685, pero aquí no hay PCA9685 conectado).
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;

    // Bus I2C nº1 — TCS34725 TRASERO, bus dedicado (ver nota de dirección
    // compartida en el encabezado).
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    // LED blanco de iluminación de cada sensor — activo en alto, con pull-up
    // propio hacia VIN si se deja sin conectar (ver hardware/conexiones-
    // esp32-s3.md). Encendidos fijos: la clasificación de color no puede
    // depender de la luz del salón de competencia.
    constexpr uint8_t TCS_LED_FRONT = 18;
    constexpr uint8_t TCS_LED_BACK  = 21;
}

namespace I2CAddr {
    constexpr uint8_t TCS34725 = 0x29;   // fija, no se puede cambiar — por eso 2 buses
}

// ===========================================================================
//  SENSOR BAJO PRUEBA — cambiar SOLO esto para pasar de delantero a trasero
// ===========================================================================
constexpr bool SENSOR_ES_DELANTERO = true;

TwoWire &BusActivo = SENSOR_ES_DELANTERO ? Wire : Wire1;

// ===========================================================================
//  DRIVER TCS34725 — copia exacta del namespace Tcs34725 de firmware-esp32/
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

    bool WriteReg(TwoWire &bus, uint8_t reg, uint8_t value) {
        bus.beginTransmission(I2CAddr::TCS34725);
        bus.write(CMD_BIT | reg);
        bus.write(value);
        return bus.endTransmission() == 0;
    }

    bool ReadReg(TwoWire &bus, uint8_t reg, uint8_t &out) {
        bus.beginTransmission(I2CAddr::TCS34725);
        bus.write(CMD_BIT | reg);
        if (bus.endTransmission() != 0) return false;
        if (bus.requestFrom((int)I2CAddr::TCS34725, 1) != 1) return false;
        out = (uint8_t)bus.read();
        return true;
    }

    bool Init(TwoWire &bus) {
        uint8_t id = 0;
        if (!ReadReg(bus, REG_ID, id)) return false;
        // 0x44 = TCS34725, 0x4D = TCS34727. Cualquier otra cosa no es el sensor.
        if (id != 0x44 && id != 0x4D) return false;

        if (!WriteReg(bus, REG_ATIME, ATIME_24MS)) return false;
        if (!WriteReg(bus, REG_CONTROL, GAIN_4X)) return false;
        if (!WriteReg(bus, REG_ENABLE, ENABLE_PON)) return false;
        delay(3);                                   // arranque del oscilador
        return WriteReg(bus, REG_ENABLE, ENABLE_PON | ENABLE_AEN);
    }

    bool Read(TwoWire &bus, Rgbc &out) {
        bus.beginTransmission(I2CAddr::TCS34725);
        bus.write(CMD_BIT | CMD_AUTO_INC | REG_CDATAL);
        if (bus.endTransmission() != 0) return false;
        if (bus.requestFrom((int)I2CAddr::TCS34725, 8) != 8) return false;

        // Los cuatro canales llegan como uint16 little-endian, en orden C R G B.
        out.c = (uint16_t)(bus.read() | (bus.read() << 8));
        out.r = (uint16_t)(bus.read() | (bus.read() << 8));
        out.g = (uint16_t)(bus.read() | (bus.read() << 8));
        out.b = (uint16_t)(bus.read() | (bus.read() << 8));
        return true;
    }
}

// ===========================================================================
//  ESTADO Y LECTURA
// ===========================================================================

bool g_sensorOk = false;

// Si el sensor no estaba OK, reintenta inicializarlo antes de leer. Sin
// esto, un sensor que no respondió al arrancar (cable conectado después,
// por ejemplo) se quedaría muerto para siempre en esta sesión.
bool ReadOrReinit(TwoWire &bus, Tcs34725::Rgbc &out, bool &okState) {
    if (!okState) {
        okState = Tcs34725::Init(bus);
    }
    if (okState && Tcs34725::Read(bus, out)) {
        return true;
    }
    okState = false;
    out = Tcs34725::Rgbc{};
    return false;
}

void readSensors() {
    Tcs34725::Rgbc s;
    const bool ok = ReadOrReinit(BusActivo, s, g_sensorOk);

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

    // Los dos buses y los dos LED se inicializan SIEMPRE, use o no use el
    // sensor de ese lado — así SENSOR_ES_DELANTERO es de verdad lo único
    // que hay que tocar para cambiar de sensor, ver el encabezado.
    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);

    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    pinMode(Pins::TCS_LED_BACK, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);
    digitalWrite(Pins::TCS_LED_BACK, HIGH);

    g_sensorOk = Tcs34725::Init(BusActivo);

    Serial.println("READY");
}

// ===========================================================================
// LOOP
// ===========================================================================

void loop() {
    /*
       El ESP32 espera comandos de la computadora.
       Comando:
       R
       Respuesta:
       DATA,ok,clear,r,g,b
    */
    if (Serial.available()) {
        const char command = Serial.read();

        if (command == 'R') {
            readSensors();
        }

        // Descarta cualquier byte extra (\r, \n, etc.) que haya llegado
        // junto al comando, para que no quede colgado en el buffer y se
        // procese por error en el siguiente ciclo.
        while (Serial.available()) {
            Serial.read();
        }
    }
}
