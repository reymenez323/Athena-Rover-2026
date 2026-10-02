// ===========================================================================
//  Detector NEGRO/GRIS — banco de calibración (ESP32-S3)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  Lee los dos QTRX-HD-01A (izquierdo GPIO1, derecho GPIO2) y clasifica cada
//  uno por separado entre NEGRO (borde) y GRIS (pista). Imprime el resultado
//  cada 200 ms y lo muestra en la tira WS2812B de 8 LED (GPIO 39), en los
//  MISMOS índices y MISMOS colores que usará en la misión de verdad
//  (Tira::DibujarMapa() en standalones/v9-tira-sensor-trasero/): LED 1 = QTR
//  IZQUIERDO, LED 2 = QTR DERECHO -- ACTUALIZADO 2026-10-01 a pedido de
//  Montse (antes usaba 4 LED por sensor y su propia paleta, que no
//  coincidía con lo que se ve en pista).
//
//      NEGRO (borde)   -> morado   (160, 0, 200)
//      GRIS (pista)    -> verde tenue (0, 40, 0)
//      sin señal       -> gris tenue (25, 25, 25) -- sensor pegado al tope del ADC
//
//  MEDICIÓN: la misma del robot (v8/v9) -- emisor IR apagado (>= 1 ms) ->
//  "off"; encendido -> "on"; dif = on - off, y |dif| < umbral = NEGRO. Con el
//  sensor DERECHO se midió negro |dif| ~ 1-8 y gris ~ 76-87, umbral 40.
//
//  MEJORAS SOLO POR SOFTWARE (2026-10-01), para distinguir mejor sin tocar el
//  hardware. Las tres son parámetros ajustables abajo:
//    1. PROMEDIO: cada "off" y cada "on" es el promedio de kMuestrasPromedio
//       lecturas del ADC (el ADC del ESP32 es ruidoso). Debe coincidir con el
//       de ../firmware/ (la captura a CSV).
//    2. HISTÉRESIS: para pasar de GRIS a NEGRO hace falta |dif| < umbral; para
//       VOLVER a GRIS hace falta |dif| > umbral + histéresis. Una lectura que
//       ronda justo el umbral ya no hace parpadear el resultado.
//    3. CONFIRMACIÓN: un cambio de estado solo se acepta tras kLecturasConfirmar
//       lecturas seguidas que lo apoyen (a 50 Hz, 3 lecturas = 60 ms; el robot
//       usa kBordeDebounceMs = 50 ms).
//  El robot (v8/v9/firmware-esp32) YA tiene el PROMEDIO y la HISTÉRESIS
//  (portados 2026-10-01) -- ver kQtrMuestrasPromedio/kBordeHisteresis en
//  Mission::. Lo único que NO se portó es la CONFIRMACIÓN de 3 lecturas: el
//  robot sigue con su propio antirrebote por tiempo (kBordeDebounceMs = 50
//  ms), un enfoque distinto, no necesariamente peor -- si hace falta la
//  confirmación por lecturas también, hay que agregarla aparte.
//
//  ⚠️ Los umbrales se midieron SOLO con el derecho. kUmbralIzquierdo es una
//  (YA NO: 2026-10-02 ambos umbrales se midieron con el robot quieto sobre la pista real.)
//
//  Pines = cableado real (hardware/conexiones-esp32-s3.md): CTRL de los dos
//  emisores IR compartido en GPIO42. ¡Los QTRX van a 3.3 V, nunca a 5 V!
// ===========================================================================

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// =====================================================
// PINES
// =====================================================

constexpr uint8_t kPinQtrIzquierdo = 1;    // ADC1_CH0
constexpr uint8_t kPinQtrDerecho   = 2;    // ADC1_CH1
constexpr uint8_t kPinEmisor       = 42;   // CTRL de los emisores IR, compartido
constexpr uint8_t kPinTira         = 39;   // DATA de la tira WS2812B

// =====================================================
// PARÁMETROS
// =====================================================

constexpr uint8_t  kMuestrasPromedio = 16;   // lecturas del ADC por cada off / on. IGUAL que ../firmware/
constexpr int16_t  kUmbralDerecho    = 90;   // |dif| menor que esto = negro. 2026-10-02, robot quieto sobre la pista real: negro 31-49, gris 141-204 (antes 40: el negro llegó a 49)
constexpr int16_t  kUmbralIzquierdo  = 900;  // 2026-10-02, robot quieto sobre la pista real: negro 42-694, gris 1140-1377. Es el sensor menos consistente (el negro varía mucho según el punto)
constexpr int16_t  kHisteresis       = 10;   // para volver a GRIS hace falta |dif| > umbral + esto
constexpr uint8_t  kLecturasConfirmar = 3;   // lecturas seguidas que apoyan un cambio de estado antes de aceptarlo
constexpr uint16_t kTopeAdc          = 4085; // off Y on >= esto = pegado al tope (igual que kQtrTopeAdc de v9)
constexpr uint32_t kPeriodoLecturaMs = 20;   // 50 Hz, igual que el robot
constexpr uint32_t kPeriodoImpresionMs = 200;
constexpr uint8_t  kBrillo           = 40;   // 0-255, igual que pruebas-platformio/10-tira-ws2812

Adafruit_NeoPixel tira(8, kPinTira, NEO_GRB + NEO_KHZ800);

// =====================================================
// LECTURA
// =====================================================

enum class Estado : uint8_t { GRIS, NEGRO, SIN_SENAL };

struct Lectura {
    uint16_t offIzq, onIzq, offDer, onDer;
};

uint16_t LeerPromedio(uint8_t pin) {
    uint32_t suma = 0;
    for (uint8_t i = 0; i < kMuestrasPromedio; ++i) suma += (uint32_t)analogRead(pin);
    return (uint16_t)(suma / kMuestrasPromedio);
}

// Misma secuencia que ReflectanceTask en v9: apagar >= 1 ms, leer "off";
// encender, asentar 200 us, leer "on". Los dos sensores comparten el CTRL,
// así que se leen en el mismo ciclo.
Lectura Leer() {
    Lectura l;
    digitalWrite(kPinEmisor, LOW);
    delay(2);   // >= 1 ms: apagado real
    l.offIzq = LeerPromedio(kPinQtrIzquierdo);
    l.offDer = LeerPromedio(kPinQtrDerecho);

    digitalWrite(kPinEmisor, HIGH);
    delayMicroseconds(200);   // asentar el fototransistor con luz IR estable
    l.onIzq = LeerPromedio(kPinQtrIzquierdo);
    l.onDer = LeerPromedio(kPinQtrDerecho);
    return l;
}

// Estado de un sensor con histéresis y confirmación.
struct Filtro {
    Estado estable = Estado::GRIS;
    uint8_t contador = 0;   // lecturas seguidas que apoyan el estado contrario

    Estado Actualizar(uint16_t off, uint16_t on, int16_t umbral) {
        if (off >= kTopeAdc && on >= kTopeAdc) {
            estable = Estado::SIN_SENAL;
            contador = 0;
            return estable;
        }
        const int16_t absDif = (int16_t)abs((int32_t)on - (int32_t)off);

        // ¿Esta lectura pide cambiar de estado? El umbral depende del estado
        // actual (histéresis): entrar a NEGRO con dif < umbral, volver a GRIS
        // con dif > umbral + kHisteresis.
        bool pideCambio;
        Estado destino;
        if (estable == Estado::NEGRO) {
            pideCambio = absDif > umbral + kHisteresis;
            destino = Estado::GRIS;
        } else {   // GRIS, o saliendo de SIN_SENAL
            pideCambio = absDif < umbral;
            destino = Estado::NEGRO;
        }

        if (estable == Estado::SIN_SENAL) {
            // Con señal otra vez: se decide directo, sin esperar confirmación.
            estable = absDif < umbral ? Estado::NEGRO : Estado::GRIS;
            contador = 0;
        } else if (pideCambio) {
            if (++contador >= kLecturasConfirmar) {
                estable = destino;
                contador = 0;
            }
        } else {
            contador = 0;
        }
        return estable;
    }
};

Filtro g_izq, g_der;

const char *Nombre(Estado e) {
    switch (e) {
        case Estado::NEGRO:     return "NEGRO";
        case Estado::SIN_SENAL: return "SIN SENAL";
        default:                return "GRIS";
    }
}

// =====================================================
// TIRA
// =====================================================

// MISMOS colores que Tira::DibujarMapa() en v9 para los LED 1/2 (QTR).
uint32_t ColorDe(Estado e) {
    switch (e) {
        case Estado::NEGRO:     return tira.Color(160, 0, 200);   // borde detectado
        case Estado::SIN_SENAL: return tira.Color(25, 25, 25);    // pegado al tope -- sin señal útil
        default:                return tira.Color(0, 40, 0);      // pista libre
    }
}

// LED 1 = QTR izquierdo, LED 2 = QTR derecho -- MISMOS índices que usa la
// misión real. Los demás LED (equipo/color/cámara-ToF/gripper/fase) no
// aplican a este banco y quedan apagados.
void Mostrar(Estado izq, Estado der) {
    tira.clear();
    tira.setPixelColor(1, ColorDe(izq));
    tira.setPixelColor(2, ColorDe(der));
    tira.show();
}

// =====================================================
// SETUP / LOOP
// =====================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nDetector NEGRO/GRIS - banco de calibracion");
    Serial.println("Medicion con rechazo de luz ambiente (dif = on - off), igual que el robot.");
    Serial.printf("Mejoras por software: promedio de %u lecturas, histeresis %d, confirmacion de %u lecturas.\n",
                  (unsigned)kMuestrasPromedio, (int)kHisteresis, (unsigned)kLecturasConfirmar);
    Serial.println("Tira WS2812B (GPIO 39): LED 1 = QTR izquierdo, LED 2 = QTR derecho (mismos indices y colores que v9 en mision).");
    Serial.println("NEGRO morado, GRIS pista libre verde tenue, SIN SENAL gris tenue.");
    Serial.printf("Umbrales (2026-10-02): derecho %d, izquierdo %d.\n\n", (int)kUmbralDerecho, (int)kUmbralIzquierdo);

    analogReadResolution(12);

    // Misma atenuación que firmware-esp32/ y v9. Sin fijarla, el ADC queda en
    // el valor por defecto del core y las lecturas se desplazan de sesión en
    // sesión.
    analogSetPinAttenuation(kPinQtrIzquierdo, ADC_11db);
    analogSetPinAttenuation(kPinQtrDerecho, ADC_11db);

    pinMode(kPinEmisor, OUTPUT);
    digitalWrite(kPinEmisor, HIGH);

    tira.begin();
    tira.setBrightness(kBrillo);
    tira.clear();
    tira.show();

    Serial.println("Listo.\n");
}

void loop() {
    static uint32_t ultimaLecturaMs = 0, ultimaImpresionMs = 0;
    const uint32_t ahora = millis();
    if ((uint32_t)(ahora - ultimaLecturaMs) < kPeriodoLecturaMs) return;
    ultimaLecturaMs = ahora;

    const Lectura l = Leer();
    const Estado izq = g_izq.Actualizar(l.offIzq, l.onIzq, kUmbralIzquierdo);
    const Estado der = g_der.Actualizar(l.offDer, l.onDer, kUmbralDerecho);
    Mostrar(izq, der);

    if ((uint32_t)(ahora - ultimaImpresionMs) >= kPeriodoImpresionMs) {
        ultimaImpresionMs = ahora;
        Serial.printf(
            "izq=%-9s (off=%4u on=%4u dif=%5d) | der=%-9s (off=%4u on=%4u dif=%5d)\n",
            Nombre(izq), l.offIzq, l.onIzq, (int)l.onIzq - (int)l.offIzq,
            Nombre(der), l.offDer, l.onDer, (int)l.onDer - (int)l.offDer);
    }
}
