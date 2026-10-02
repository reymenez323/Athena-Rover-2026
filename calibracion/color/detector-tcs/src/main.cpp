// ===========================================================================
//  Detector TCS34725 — banco de calibración (ESP32-S3)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  Lee los dos TCS34725 (delantero y trasero) y clasifica cada uno entre
//  AZUL / ROJO / AMARILLO / NEGRO / GRIS. Imprime el resultado por consola
//  cada 200 ms y lo muestra en la tira WS2812B de 8 LED, en los MISMOS
//  índices y MISMOS colores que usará en la misión de verdad
//  (Tira::DibujarMapa() / ColorARgb() en standalones/v9-tira-sensor-trasero/):
//  LED 3 = delantero, LED 4 = trasero. ROJO rojo, AZUL azul, AMARILLO
//  amarillo, GRIS (piso) gris tenue, NEGRO (o sensor sin leer) apagado. Los
//  demás LED (0,1,2,5,6,7 -- equipo/QTR/cámara-ToF/gripper/fase) no
//  aplican a este banco y quedan apagados -- ACTUALIZADO 2026-10-01 a
//  pedido de Montse: antes usaba 4 LED por sensor y morado para NEGRO, un
//  mapa propio de este banco que no coincidía con lo que se ve en pista.
//
//  ACTUALIZADO 2026-10-01: los sensores ahora están detrás del multiplexor
//  TCA9548A (0x71, canal 0 delantero, canal 1 trasero), igual que en standalones/v9-tira-sensor-trasero/
//  y ../firmware/; y el LED RGB de 3 canales se reemplazó por la tira (el
//  GPIO 39 es su DATA; el 38 y el 41 ya no son del RGB).
//
//  ACTUALIZADO 2026-09-09 al cableado real vigente (ver
//  ../../tof/README.md, "RESUELTO 2026-09-08 (de verdad)"): el TCS34725
//  delantero se recableó al bus I2C nº1 (GPIO47/48, junto al PCA9685) y el
//  TCS34725 trasero quedó desconectado por completo -- ya no compite por
//  el bus ni añade su propia luz de iluminación como interferencia. Este
//  archivo antes leía dos sensores (delantero en bus0, trasero en bus1);
//  ahora solo existe uno.
//
//  ⚠️ El TRASERO tiene sus propios umbrales (kUmbralTrasero), calibrados
//  2026-10-01 solo para GRIS y AZUL; ROJO/AMARILLO/NEGRO usan aún los del
//  delantero, sin medir.
//
//  UMBRALES RECALIBRADOS 2026-08-28 contra 970 muestras reales del sensor
//  DELANTERO (ver ../analizar_umbrales_tcs.py y el comentario junto a
//  kUmbralDelantero más abajo para el detalle completo, con matriz de confusión).
//  Error total 14.3% (era 58.1% con los umbrales originales sin calibrar).
//  ⚠️ NEGRO es la clase que peor le va (58.5% de acierto) — es un límite
//  estructural de esta clasificación por umbrales encadenados, no algo que
//  se arregle recalibrando de nuevo; ver la explicación junto a kUmbralDelantero.
//
//  Esos umbrales se calibraron con el delantero en su bus VIEJO (bus0). Si
//  el bus nuevo (bus1, junto al PCA9685) le cambia el ruido eléctrico
//  percibido, la primera señal de alerta sería confusiones entre GRIS y
//  AMARILLO -- compara lo que veas aquí contra la calibración original
//  antes de asumir que sigue sirviendo tal cual.
//
//  Driver TCS34725 copiado TAL CUAL de firmware-esp32/ y de ../firmware/
//  (mismo ATIME/GAIN) — mismo motivo que en ../firmware/src/main.cpp: si
//  este banco leyera con una configuración distinta a la del robot real,
//  ni los datos ni la clasificación en vivo servirían para nada.
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// ===========================================================================
//  PINES — bus I2C nº1 (GPIO47/48), igual que ../../tof/firmware/src/main.cpp
// ===========================================================================

namespace Pins {
    // Bus I2C nº1 (GPIO47/48): multiplexor TCA9548A con el TCS34725 detrás.
    // El bus I2C nº0 (GPIO8/9) queda para el VL53L1X solo -- este sketch no
    // lo toca porque no necesita distancia, solo color.
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;

    constexpr uint8_t TCS_LED_FRONT = 18;
    constexpr uint8_t TCS_LED_REAR  = 41;   // igual que en v9

    constexpr uint8_t TIRA_DATA = 39;   // DATA de la tira WS2812B
}

namespace I2CAddr {
    constexpr uint8_t TCS34725 = 0x29;   // fija, no se puede cambiar
    // TCA9548A con A0 a 3.3V: de fábrica (0x70) choca con el all-call del PCA9685.
    constexpr uint8_t MULTIPLEXOR = 0x71;
}

constexpr uint8_t kMuxCanalDelantero = 0;
constexpr uint8_t kMuxCanalTrasero   = 1;

// Un solo registro: escribir un byte con el bit del canal en 1 lo activa y
// apaga los demás. Copia de Multiplexor:: de v9.
bool SeleccionarCanalMux(TwoWire &bus, uint8_t canal) {
    bus.beginTransmission(I2CAddr::MULTIPLEXOR);
    bus.write((uint8_t)(1u << canal));
    return bus.endTransmission() == 0;
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

// RECALIBRADO 2026-08-28, AMARILLO y GRIS reajustados 2026-09-07 -- contra
// los CSV de ../data_logs/ (sensor DELANTERO) con
// ../analizar_umbrales_tcs.py -- descenso de coordenadas sobre esta MISMA
// estructura de reglas, buscando los 9 umbrales que menos errores dieran.
//
//   Umbrales originales (sin calibrar):           564/970  errores (58.1%)
//   Recalibrados 2026-08-28 (los 5 colores):       139/970  errores (14.3%)
//   Reajuste 2026-09-07 #1 (solo AMARILLO_G_MIN):  144/1233 errores (11.7%)
//   Reajuste 2026-09-07 #2 (los 9 de nuevo, con
//     AMARILLO+GRIS frescos de hoy):               171/1516 errores (11.3%)
//
//   Por clase (tras el reajuste #2): AMARILLO 461/463 (99.6%),
//   GRIS 458/481 (95.2%), AZUL 211/222 (95.0%), ROJO 141/150 (94.0%),
//   NEGRO 74/200 (37.0%) -- NEGRO empeoró respecto al reajuste #1 (era
//   58.5%): con GRIS más que duplicado en muestras (198 -> 481), el
//   descenso de coordenadas le da más peso a acertarle a GRIS/AMARILLO/
//   AZUL/ROJO a costa de NEGRO. Sigue siendo un LÍMITE ESTRUCTURAL de esta
//   clasificación secuencial (el rango de `clear` de NEGRO se solapa
//   fuerte con AZUL y GRIS, ver el detalle completo corriendo el script de
//   nuevo) -- el clasificador K-NN de pruebas-platformio/05-evitador-
//   linea/ le acierta mucho mejor con el MISMO dataset, ver ese sketch si
//   hace falta más confiabilidad en NEGRO.
//
// REAJUSTE #2, por qué: con el reajuste #1 (solo AMARILLO_G_MIN bajado a
// 0.200) empezaron a aparecer falsos positivos de AMARILLO sobre piso
// GRIS -- el r/c del delantero sobre gris ronda 0.40-0.42, casi pegado a
// AMARILLO_R_MIN (0.416), y bajar G_MIN quitó un filtro que por casualidad
// (nunca a propósito) venía bloqueando esos falsos positivos. Se
// recapturó GRIS (5 puntos, 500 muestras) bajo la misma luz de hoy y se
// corrieron los 9 umbrales de nuevo contra AMARILLO+GRIS frescos +
// ROJO/AZUL/NEGRO del 28 de agosto.
//
// ⚠️ ESTOS NÚMEROS SON SOLO DEL DELANTERO. NO los copies a ClassifyColor()
// en firmware-esp32/src/main.cpp: ese firmware le quitó el sensor
// delantero (ver el aviso ahí) y ClassifyColor() ahora clasifica al
// TRASERO, que nunca se caracterizó con esta herramienta y bien puede
// tener una distribución de color distinta (LED de iluminación propio,
// posición distinta en el chasis). Aplicarle un umbral ajustado para el
// delantero sería resolver a ciegas un problema que no se midió.
struct Umbrales {
    uint16_t clearNegroMax;   // clear < esto -> NEGRO, sin mirar el resto
    float rojoRMin, rojoGMax, rojoBMax;
    float azulBMin, azulRMax;
    float amarilloRMin, amarilloGMin, amarilloBMax;
};

// DELANTERO: los de arriba (2026-09-07).
constexpr Umbrales kUmbralDelantero = {
    314, 0.450f, 0.312f, 0.300f, 0.206f, 0.390f, 0.420f, 0.200f, 0.140f
};

// TRASERO, 2026-10-01: SOLO se calibraron GRIS (240 muestras, 4 puntos) y
// AZUL (100 muestras, 1 punto) -- ROJO, AMARILLO y NEGRO siguen con los
// valores del delantero, sin medir. Con los del delantero el trasero fallaba
// 137/340 (40%): 136 de 240 muestras de GRIS salían AZUL, porque el b/c del
// GRIS trasero (mediana 0.211, máx 0.260) queda pegado a AZUL_B_MIN=0.206.
// El AZUL trasero tiene b/c entre 0.299 y 0.366, así que se puso el umbral en
// el punto medio de la brecha: AZUL_B_MIN = 0.28 (0 errores GRIS/AZUL sobre
// esas muestras; ver el reporte en el commit/README).
// ⚠️ La muestra de AZUL es de un solo punto: conviene capturar más puntos
// (distinta distancia/luz) antes de darlo por bueno.
//
// NEGRO trasero medido 2026-10-01 (480 muestras, 4 puntos, con
// calibrar_color.py --sensor TRASERO --superficie NEGRO): clear
// min=60 mediana=288 p95=536 max=804. Hay rachas de muestras altas (hasta
// 400-600) hacia el final de cada punto de 30 s -- es la mano aflojando el
// contacto con el negro, no ruido del sensor; en la próxima captura, menos
// muestras por punto (40-50) reduce esa ventana.
//
// CLEAR_NEGRO_MAX recalibrado contra NEGRO (480 muestras) + GRIS (240
// muestras) del trasero, barriendo el umbral y minimizando errores totales:
//   314 (el del delantero, SIN CALIBRAR para el trasero): NEGRO 41.2% mal, GRIS 0.4% mal -- 199 errores
//   600 (elegido):                                         NEGRO  2.9% mal, GRIS 4.2% mal --  24 errores
//   606 (óptimo exacto):                                   NEGRO  2.5% mal, GRIS 4.6% mal --  23 errores
// Se eligió 600 (redondo, prácticamente igual al óptimo). Si en pista se ve
// que el robot no nota cuando se sale (falso NEGRO perdido es peor que una
// falsa alarma), subir hacia 650 baja el error de NEGRO a 1.9% a costa de
// subir el de GRIS a 7.5%.
constexpr Umbrales kUmbralTrasero = {
    600, 0.450f, 0.312f, 0.300f, 0.280f, 0.390f, 0.420f, 0.200f, 0.140f
};

// Normaliza cada canal contra "clear" (luz total) antes de comparar, para
// que la decisión no dependa del brillo absoluto — mismo razonamiento que
// firmware-esp32/. Si s.c es 0 (sensor sin luz o lectura inválida), la
// división da NaN/Inf; todas las comparaciones con NaN son falsas, así que
// esto cae de forma segura en GRIS sin crashear — mismo comportamiento sin
// resolver que ya tiene firmware-esp32/, no es un bug nuevo de este archivo.
ColorLabel Clasificar(const Tcs34725::Rgbc &s, const Umbrales &u) {
    if (s.c < u.clearNegroMax) return ColorLabel::NEGRO;

    const float total = (float)s.c;
    const float r = (float)s.r / total;
    const float g = (float)s.g / total;
    const float b = (float)s.b / total;

    if (r > u.rojoRMin && g < u.rojoGMax && b < u.rojoBMax) return ColorLabel::ROJO;
    if (b > u.azulBMin && r < u.azulRMax) return ColorLabel::AZUL;
    if (r > u.amarilloRMin && g > u.amarilloGMin && b < u.amarilloBMax) return ColorLabel::AMARILLO;

    return ColorLabel::GRIS;   // ni negro, ni rojo, ni azul, ni amarillo -> piso/gris
}

// ===========================================================================
//  TIRA WS2812B — muestra la clasificación del sensor delantero
// ===========================================================================

constexpr uint8_t kBrillo = 40;   // 0-255. Mismo valor que pruebas-platformio/10-tira-ws2812 (~100 mA toda la tira)

Adafruit_NeoPixel tira(8, Pins::TIRA_DATA, NEO_GRB + NEO_KHZ800);

namespace Tira {
    void Setup() {
        tira.begin();
        tira.setBrightness(kBrillo);
        tira.clear();
        tira.show();
    }

    void Apagar() {
        tira.clear();
        tira.show();
    }

    // MISMOS colores que Tira::ColorARgb() en v9 (ColorLabel::FLOOR/YELLOW/
    // RED/BLUE/BLACK-UNKNOWN) -- no inventar una paleta propia de este banco.
    uint32_t ColorDe(ColorLabel l) {
        switch (l) {
            case ColorLabel::GRIS:     return tira.Color(60, 60, 60);    // piso
            case ColorLabel::AMARILLO: return tira.Color(255, 170, 0);
            case ColorLabel::ROJO:     return tira.Color(255, 0, 0);
            case ColorLabel::AZUL:     return tira.Color(0, 0, 255);
            case ColorLabel::NEGRO:
            default:                   return 0;                         // apagado, igual que v9
        }
    }

    // LED 3 = delantero, LED 4 = trasero -- MISMOS índices que usa la misión
    // real (ver Tira::DibujarMapa() en v9). Un sensor que no leyó (valid=false)
    // se muestra apagado, igual que v9 cuando el color es UNKNOWN. Los demás
    // LED (equipo/QTR/cámara-ToF/gripper/fase) no aplican acá y quedan apagados.
    void Mostrar(bool frontValid, ColorLabel front, bool rearValid, ColorLabel rear) {
        tira.clear();
        tira.setPixelColor(3, frontValid ? ColorDe(front) : 0);
        tira.setPixelColor(4, rearValid  ? ColorDe(rear)  : 0);
        tira.show();
    }
}

// ===========================================================================
// SETUP
// ===========================================================================

bool g_frontOk = false;
bool g_rearOk  = false;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nDetector TCS34725 - clasificador de color (banco)");
    Serial.println("Sensores detras del multiplexor TCA9548A (0x71, bus I2C 1, GPIO47/48): delantero canal 0, trasero canal 1.");
    Serial.println("Tira WS2812B (GPIO 39): LED 3 = delantero, LED 4 = trasero (mismos indices y colores que v9 en mision). ROJO/AZUL/AMARILLO en su color, GRIS tenue, NEGRO/sin leer apagado.");
    Serial.println("Umbrales por sensor; el trasero solo tiene GRIS y AZUL calibrados. NEGRO es el mas debil.\n");

    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL);

    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);
    pinMode(Pins::TCS_LED_REAR, OUTPUT);
    digitalWrite(Pins::TCS_LED_REAR, HIGH);

    Tira::Setup();

    g_frontOk = SeleccionarCanalMux(Wire1, kMuxCanalDelantero) && Tcs34725::Init(Wire1);
    g_rearOk  = SeleccionarCanalMux(Wire1, kMuxCanalTrasero)   && Tcs34725::Init(Wire1);
    if (!g_frontOk) Serial.println("[Setup] TCS34725 delantero no responde (multiplexor 0x71 canal 0 / bus I2C 1).");
    if (!g_rearOk)  Serial.println("[Setup] TCS34725 trasero no responde (multiplexor 0x71 canal 1 / bus I2C 1).");

    Serial.println("Listo.\n");
}

// Lee un sensor por su canal del multiplexor, reintentando su init si estaba
// caído. Devuelve true si hubo lectura válida (y la clasifica en `label`).
bool LeerSensor(uint8_t canal, bool &ok, Tcs34725::Rgbc &rgbc, const Umbrales &umbrales, ColorLabel &label) {
    if (!ok) ok = SeleccionarCanalMux(Wire1, canal) && Tcs34725::Init(Wire1);
    if (ok && SeleccionarCanalMux(Wire1, canal) && Tcs34725::Read(Wire1, rgbc)) {
        label = Clasificar(rgbc, umbrales);
        return true;
    }
    ok = false;
    rgbc = Tcs34725::Rgbc{};
    return false;
}

// ===========================================================================
// LOOP
// ===========================================================================

void loop() {
    // Reintento perezoso de un sensor caído a 1 Hz, sin bloquear el resto —
    // mismo patrón que ColorSensorTask en firmware-esp32/.
    static uint32_t lastRetryMs = 0;
    const uint32_t now = millis();
    const bool retryNow = (uint32_t)(now - lastRetryMs) > 1000;
    if (retryNow) lastRetryMs = now;

    Tcs34725::Rgbc front, rear;
    ColorLabel frontLabel = ColorLabel::GRIS, rearLabel = ColorLabel::GRIS;
    bool frontValid = false, rearValid = false;

    if (g_frontOk || retryNow) frontValid = LeerSensor(kMuxCanalDelantero, g_frontOk, front, kUmbralDelantero, frontLabel);
    if (g_rearOk  || retryNow) rearValid  = LeerSensor(kMuxCanalTrasero,   g_rearOk,  rear,  kUmbralTrasero, rearLabel);

    Tira::Mostrar(frontValid, frontLabel, rearValid, rearLabel);

    Serial.printf(
        "delantero=%-8s (c=%5u r=%5u g=%5u b=%5u)%s | trasero=%-8s (c=%5u r=%5u g=%5u b=%5u)%s\n",
        frontValid ? LabelName(frontLabel) : "?", front.c, front.r, front.g, front.b, frontValid ? "" : " SIN LEER",
        rearValid  ? LabelName(rearLabel)  : "?", rear.c,  rear.r,  rear.g,  rear.b,  rearValid  ? "" : " SIN LEER");

    delay(200);
}
