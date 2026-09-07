// ===========================================================================
//  Detector TCS34725 — banco de calibración (ESP32-S3)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  Lee los DOS TCS34725 (delantero y trasero) y clasifica cada uno, por
//  separado, entre AZUL / ROJO / AMARILLO / NEGRO / GRIS. Imprime el
//  resultado de ambos por consola cada 200 ms. El LED RGB de equipo (uno
//  solo, compartido) muestra la clasificación del sensor DELANTERO — no
//  hay forma de mostrar los dos color a la vez con un solo LED.
//
//  UMBRALES RECALIBRADOS 2026-08-28 contra 970 muestras reales del sensor
//  DELANTERO (ver ../analizar_umbrales_tcs.py y el comentario junto a
//  Umbral:: más abajo para el detalle completo, con matriz de confusión).
//  Error total 14.3% (era 58.1% con los umbrales originales sin calibrar).
//  ⚠️ NEGRO es la clase que peor le va (58.5% de acierto) — es un límite
//  estructural de esta clasificación por umbrales encadenados, no algo que
//  se arregle recalibrando de nuevo; ver la explicación junto a Umbral::.
//
//  ⚠️ TODAVÍA sin calibrar el sensor TRASERO: estos umbrales solo se
//  probaron contra datos del DELANTERO (../data_logs/ solo tiene corridas
//  DELANTERO por ahora, ver ../calibrar_color.py). Un solo juego de
//  umbrales compartido entre los dos sensores sigue siendo un supuesto sin
//  verificar (ver ../README.md, sección "¿Calibrar los dos sensores por
//  separado...?") — cuando existan capturas del trasero, correr
//  ../analizar_umbrales_tcs.py de nuevo con ambos datasets y comparar. Si
//  no se separan igual de limpio, este archivo necesita DOS structs
//  Umbral (uno por sensor), no uno.
//
//  Driver TCS34725 copiado TAL CUAL de firmware-esp32/ y de ../firmware/
//  (mismo ATIME/GAIN) — mismo motivo que en ../firmware/src/main.cpp: si
//  este banco leyera con una configuración distinta a la del robot real,
//  ni los datos ni la clasificación en vivo servirían para nada.
//
//  Pines: igual que ../firmware/src/main.cpp — ver hardware/conexiones-
//  esp32-s3.md — más el LED RGB en los mismos GPIO que usa el diseño final
//  (firmware-esp32/, pruebas-platformio/02-cuadro-color-rgb/).
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>

// ===========================================================================
//  PINES — idénticos a ../firmware/src/main.cpp y a firmware-esp32
// ===========================================================================

namespace Pins {
    constexpr uint8_t I2C0_SDA = 8;    // TCS34725 delantero
    constexpr uint8_t I2C0_SCL = 9;
    constexpr uint8_t I2C1_SDA = 47;   // TCS34725 trasero (bus dedicado, ver nota de direccion fija)
    constexpr uint8_t I2C1_SCL = 48;

    // Solo el delantero pasa por GPIO: el trasero se cableó directo a 3.3V
    // (ver hardware/conexiones-esp32-s3.md) -- GPIO21 quedó libre para el
    // switch de equipo.
    constexpr uint8_t TCS_LED_FRONT = 18;

    // XSHUT del VL53L1X — este sketch no lo usa para nada, pero comparte el
    // bus I2C nº0 con el TCS34725 DELANTERO en el robot real, y arranca
    // SIEMPRE respondiendo en 0x29 -- la MISMA dirección fija del TCS34725
    // (ver I2CAddr::TCS34725 abajo). Si sigue conectado y este pin se deja
    // flotando, el delantero da lecturas corrompidas mientras el trasero
    // (solo en su bus, sin nadie con quien chocar) sigue viéndose bien —
    // mismo bug ya encontrado y arreglado en ../firmware/src/main.cpp, ver
    // la nota larga ahí. Se mantiene en LOW (reset) todo el tiempo.
    constexpr uint8_t TOF_XSHUT = 3;

    constexpr uint8_t RGB_R = 39;
    constexpr uint8_t RGB_G = 38;
    // GPIO 41, NO el 3: el diseño final le cedió el 3 al XSHUT del VL53L1X
    // (ver hardware/conexiones-esp32-s3.md, sección del LED RGB) -- este
    // sketch se había quedado con la asignación vieja, de antes de ese
    // cambio, y con el 3 ocupado por dos cosas a la vez (RGB azul Y XSHUT)
    // ninguna de las dos habría funcionado bien.
    constexpr uint8_t RGB_B = 41;
}

namespace I2CAddr {
    constexpr uint8_t TCS34725 = 0x29;   // fija, no se puede cambiar — por eso 2 buses
}

// ===========================================================================
//  DRIVER TCS34725 — copia exacta del namespace Tcs34725 de firmware-esp32/
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
        if (id != 0x44 && id != 0x4D) return false;

        if (!WriteReg(bus, REG_ATIME, ATIME_24MS)) return false;
        if (!WriteReg(bus, REG_CONTROL, GAIN_4X)) return false;
        if (!WriteReg(bus, REG_ENABLE, ENABLE_PON)) return false;
        delay(3);
        return WriteReg(bus, REG_ENABLE, ENABLE_PON | ENABLE_AEN);
    }

    bool Read(TwoWire &bus, Rgbc &out) {
        bus.beginTransmission(I2CAddr::TCS34725);
        bus.write(CMD_BIT | CMD_AUTO_INC | REG_CDATAL);
        if (bus.endTransmission() != 0) return false;
        if (bus.requestFrom((int)I2CAddr::TCS34725, 8) != 8) return false;

        out.c = (uint16_t)(bus.read() | (bus.read() << 8));
        out.r = (uint16_t)(bus.read() | (bus.read() << 8));
        out.g = (uint16_t)(bus.read() | (bus.read() << 8));
        out.b = (uint16_t)(bus.read() | (bus.read() << 8));
        return true;
    }
}

// ===========================================================================
//  CLASIFICACIÓN — ver la advertencia de umbrales sin calibrar arriba
// ===========================================================================

enum class ColorLabel : uint8_t { NEGRO, AMARILLO, ROJO, AZUL, GRIS };

const char *LabelName(ColorLabel l) {
    switch (l) {
        case ColorLabel::NEGRO:    return "NEGRO";
        case ColorLabel::AMARILLO: return "AMARILLO";
        case ColorLabel::ROJO:     return "ROJO";
        case ColorLabel::AZUL:     return "AZUL";
        case ColorLabel::GRIS:     return "GRIS";
    }
    return "?";
}

// RECALIBRADO 2026-08-28, AMARILLO reajustado 2026-09-07 -- contra los CSV
// de ../data_logs/ (sensor DELANTERO) con ../analizar_umbrales_tcs.py --
// descenso de coordenadas sobre esta MISMA estructura de reglas, buscando
// los 9 umbrales que menos errores dieran.
//
//   Umbrales originales (sin calibrar):        564/970  errores (58.1%)
//   Recalibrados 2026-08-28 (los 5 colores):    139/970  errores (14.3%)
//   Reajuste 2026-09-07 (solo AMARILLO_G_MIN,
//     contra AMARILLO recapturado con luz de
//     oficina + sol -- ver el aviso abajo):    144/1233 errores (11.7%)
//
//   Por clase (tras el reajuste): AMARILLO 458/463, GRIS 196/198,
//   ROJO 140/150, AZUL 178/222, NEGRO 117/200 -- NEGRO sigue siendo, por
//   lejos, el que peor le va (58.5% de acierto). No es un umbral mal
//   elegido: el rango de `clear` de NEGRO se solapa fuertemente con el de
//   AZUL y GRIS -- un solo corte en `clear` no puede separarlos limpio, y
//   las reglas de r/g/b que siguen no fueron pensadas para distinguir
//   "negro oscuro" de "azul oscuro" o "gris oscuro". Esto es un LÍMITE
//   ESTRUCTURAL de esta clasificación secuencial por umbrales, no algo que
//   otra vuelta de ajuste vaya a arreglar — el clasificador K-NN de
//   pruebas-platformio/05-evitador-linea/ (que usa las 4 dimensiones
//   juntas, con distancias reales, en vez de reglas encadenadas con AND)
//   le acierta mucho mejor a NEGRO con el MISMO dataset — ver ese sketch
//   si hace falta una clasificación más confiable que ésta.
//
// POR QUÉ SOLO CAMBIÓ AMARILLO_G_MIN (0.350 -> 0.200) EL 2026-09-07: al
// probar el delantero con luz de oficina + sol directo (mucho más fuerte
// que la del 28 de agosto), su g/c para AMARILLO cayó consistentemente en
// 0.328-0.357 -- justo a caballo del viejo umbral de 0.350, así que
// clasificaba GRIS la mayoría de las veces aunque el sensor SÍ estaba
// viendo amarillo. Confirmado con ../detector-tcs (este mismo sketch) y
// 480 muestras nuevas (8 puntos): no era un sensor dañado, era un umbral
// sin margen para esta luz. Los otros 8 umbrales NO se retocaron -- sus
// clases no se recapturaron hoy, siguen siendo los del 28 de agosto.
//
// ⚠️ ESTOS NÚMEROS SON SOLO DEL DELANTERO. NO los copies a ClassifyColor()
// en firmware-esp32/src/main.cpp: ese firmware le quitó el sensor
// delantero (ver el aviso ahí) y ClassifyColor() ahora clasifica al
// TRASERO, que nunca se caracterizó con esta herramienta y bien puede
// tener una distribución de color distinta (LED de iluminación propio,
// posición distinta en el chasis). Aplicarle un umbral ajustado para el
// delantero sería resolver a ciegas un problema que no se midió.
namespace Umbral {
    constexpr uint16_t CLEAR_NEGRO_MAX = 392;   // clear < esto -> NEGRO, sin mirar el resto
    constexpr float ROJO_R_MIN     = 0.450f;
    constexpr float ROJO_G_MAX     = 0.312f;
    constexpr float ROJO_B_MAX     = 0.300f;
    constexpr float AZUL_B_MIN     = 0.216f;
    constexpr float AZUL_R_MAX     = 0.390f;
    constexpr float AMARILLO_R_MIN = 0.416f;
    constexpr float AMARILLO_G_MIN = 0.200f;
    constexpr float AMARILLO_B_MAX = 0.250f;
}

// Normaliza cada canal contra "clear" (luz total) antes de comparar, para
// que la decisión no dependa del brillo absoluto — mismo razonamiento que
// firmware-esp32/. Si s.c es 0 (sensor sin luz o lectura inválida), la
// división da NaN/Inf; todas las comparaciones con NaN son falsas, así que
// esto cae de forma segura en GRIS sin crashear — mismo comportamiento sin
// resolver que ya tiene firmware-esp32/, no es un bug nuevo de este archivo.
ColorLabel Clasificar(const Tcs34725::Rgbc &s) {
    if (s.c < Umbral::CLEAR_NEGRO_MAX) return ColorLabel::NEGRO;

    const float total = (float)s.c;
    const float r = (float)s.r / total;
    const float g = (float)s.g / total;
    const float b = (float)s.b / total;

    if (r > Umbral::ROJO_R_MIN && g < Umbral::ROJO_G_MAX && b < Umbral::ROJO_B_MAX) return ColorLabel::ROJO;
    if (b > Umbral::AZUL_B_MIN && r < Umbral::AZUL_R_MAX) return ColorLabel::AZUL;
    if (r > Umbral::AMARILLO_R_MIN && g > Umbral::AMARILLO_G_MIN && b < Umbral::AMARILLO_B_MAX) return ColorLabel::AMARILLO;

    return ColorLabel::GRIS;   // ni negro, ni rojo, ni azul, ni amarillo -> piso/gris
}

// ===========================================================================
//  LED RGB — muestra la clasificación del sensor DELANTERO únicamente
// ===========================================================================
//
// POLARIDAD CONFIRMADA con el LED físico: es CÁTODO COMÚN, o sea duty alto
// = canal más brillante. Mismo valor que firmware-esp32/. La constante se
// conserva por si algún día se cambia el LED por uno de ánodo común.

constexpr bool kCommonAnode = false;

namespace Pwm {
    constexpr uint32_t RGB_FREQ_HZ    = 5000;
    constexpr uint8_t  RGB_RESOLUTION = 8;
}

void PwmAttach(uint8_t pin, uint8_t channel, uint32_t freqHz, uint8_t resolution) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;
    ledcAttach(pin, freqHz, resolution);
#else
    ledcSetup(channel, freqHz, resolution);
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

namespace RgbLed {
    constexpr uint8_t CH_R = 0;
    constexpr uint8_t CH_G = 1;
    constexpr uint8_t CH_B = 2;

    void Setup() {
        PwmAttach(Pins::RGB_R, CH_R, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(Pins::RGB_G, CH_G, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(Pins::RGB_B, CH_B, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
    }

    void SetRaw(uint8_t r, uint8_t g, uint8_t b) {
        if (kCommonAnode) { r = 255 - r; g = 255 - g; b = 255 - b; }
        PwmWrite(Pins::RGB_R, CH_R, r);
        PwmWrite(Pins::RGB_G, CH_G, g);
        PwmWrite(Pins::RGB_B, CH_B, b);
    }

    // Colores elegidos para que se reconozcan a simple vista, no son la
    // paleta de equipo (esa es cosa de LedTask en firmware-esp32/).
    void ApplyLabel(ColorLabel l) {
        switch (l) {
            case ColorLabel::ROJO:     SetRaw(255, 0, 0);   break;
            case ColorLabel::AZUL:     SetRaw(0, 0, 255);   break;
            case ColorLabel::AMARILLO: SetRaw(255, 190, 0); break;
            case ColorLabel::NEGRO:    SetRaw(0, 0, 0);     break;
            case ColorLabel::GRIS:
            default:                   SetRaw(40, 40, 40);  break;   // blanco tenue = "piso/gris"
        }
    }
}

// ===========================================================================
// SETUP
// ===========================================================================

bool g_frontOk = false;
bool g_backOk  = false;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nDetector TCS34725 - clasificador de color (banco)");
    Serial.println("Umbrales recalibrados 2026-08-28 (delantero) -- NEGRO es el mas debil, ver encabezado.\n");

    // VL53L1X en reset ANTES de abrir el bus I2C0 -- ver la nota larga junto
    // a Pins::TOF_XSHUT sobre por qué, si sigue conectado, corrompe las
    // lecturas del TCS34725 delantero (misma dirección fija 0x29).
    pinMode(Pins::TOF_XSHUT, OUTPUT);
    digitalWrite(Pins::TOF_XSHUT, LOW);

    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);

    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);

    RgbLed::Setup();
    RgbLed::SetRaw(0, 0, 0);

    g_frontOk = Tcs34725::Init(Wire);
    g_backOk  = Tcs34725::Init(Wire1);
    if (!g_frontOk) Serial.println("[Setup] TCS34725 DELANTERO no responde (bus I2C 0).");
    if (!g_backOk)  Serial.println("[Setup] TCS34725 TRASERO no responde (bus I2C 1).");

    Serial.println("Listo.\n");
}

// ===========================================================================
// LOOP
// ===========================================================================

void loop() {
    // Reintento perezoso de los sensores caídos, sin bloquear el resto —
    // mismo patrón que ColorSensorTask en firmware-esp32/.
    static uint32_t lastRetryMs = 0;
    const uint32_t now = millis();
    if ((!g_frontOk || !g_backOk) && (uint32_t)(now - lastRetryMs) > 1000) {
        lastRetryMs = now;
        if (!g_frontOk) g_frontOk = Tcs34725::Init(Wire);
        if (!g_backOk)  g_backOk  = Tcs34725::Init(Wire1);
    }

    Tcs34725::Rgbc front, back;
    bool frontValid = false, backValid = false;
    ColorLabel frontLabel = ColorLabel::GRIS;
    ColorLabel backLabel  = ColorLabel::GRIS;

    if (g_frontOk && Tcs34725::Read(Wire, front)) {
        frontLabel = Clasificar(front);
        frontValid = true;
    } else {
        g_frontOk = false;
    }

    if (g_backOk && Tcs34725::Read(Wire1, back)) {
        backLabel = Clasificar(back);
        backValid = true;
    } else {
        g_backOk = false;
    }

    // El LED solo puede mostrar un color a la vez: se queda con el
    // delantero. Si no hay lectura válida, se apaga (no "NEGRO", que sería
    // engañoso -- son cosas distintas).
    RgbLed::SetRaw(0, 0, 0);
    if (frontValid) RgbLed::ApplyLabel(frontLabel);

    Serial.printf(
        "delantero=%-8s (c=%5u r=%5u g=%5u b=%5u)%s   trasero=%-8s (c=%5u r=%5u g=%5u b=%5u)%s\n",
        frontValid ? LabelName(frontLabel) : "?", front.c, front.r, front.g, front.b, frontValid ? "" : " SIN LEER",
        backValid  ? LabelName(backLabel)  : "?", back.c,  back.r,  back.g,  back.b,  backValid  ? "" : " SIN LEER");

    delay(200);
}
