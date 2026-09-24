// ===========================================================================
//  Diagnóstico de los dos QTRX-HD-01A (izquierdo y derecho) -- Athena Rover
//
//  OBJETIVO: saber si el QTR IZQUIERDO responde o está averiado. En el
//  firmware está forzado a "no borde" por "roto, pegado en 4095"; un
//  compañero dice que solo tiene poco contraste entre gris y negro.
//
//  Lee IGUAL que el firmware de vuelo y v4: emisor IR apagado (>= 1 ms) ->
//  lectura "off"; emisor encendido -> lectura "on"; dif = on - off.
//  Con el sensor DERECHO: negro |dif| ~ 1-8, gris ~ 76-87, umbral 40.
//
//  QUÉ MIRAR:
//    - Si el izquierdo marca off=on=~4095 sobre TODO (gris, negro, tapado):
//      está pegado al tope -> avería eléctrica, no se arregla escalando.
//    - Si off/on/dif CAMBIAN al pasar de gris a negro pero con poca
//      diferencia: sí responde, hay que darle su propio umbral.
//
//  CÓMO SE USA (monitor serie 115200 por el puerto UART; sin motores):
//    Sostén los sensores a la altura real de montaje sobre el gris de la
//    pista, y luego sobre la cinta negra. Para cada superficie:
//      r  -> reinicia las estadísticas     s  -> imprime el resumen
//    Comandos: r, s, 0 (par off/on, normal), 1 (emisor siempre ON),
//              2 (emisor siempre OFF), ? (ayuda)
//
//  Pines = cableado real (hardware/conexiones-esp32-s3.md): QTR izq GPIO1,
//  QTR der GPIO2, CTRL de los emisores GPIO42 (compartido).
//  ¡Los QTRX van alimentados a 3.3 V, nunca a 5 V!
// ===========================================================================

#include <Arduino.h>

#define DEBUG_LINK Serial0

namespace Pins {
    constexpr uint8_t QTR_LEFT_OUT      = 1;    // ADC1_CH0
    constexpr uint8_t QTR_RIGHT_OUT     = 2;    // ADC1_CH1
    constexpr uint8_t QTR_EMITTER_CTRL  = 42;   // CTRL de los emisores IR, compartido
}

// ---- Parámetros ajustables -------------------------------------------------
constexpr uint32_t kPeriodoImpresionMs = 250;  // cada cuánto imprime una línea
constexpr int      kMuestrasPromedio   = 8;    // lecturas promediadas por medición (el firmware de vuelo usa 1; acá se promedian para ver mejor el ruido)
constexpr int16_t  kBordeRestadoUmbral = 40;   // mismo umbral del firmware: |dif| menor que esto = negro
constexpr uint16_t kTopeADC            = 4085; // off y on >= esto = el sensor está pegado al tope (12 bits, máx 4095)
constexpr uint32_t kApagadoEmisorMs    = 2;    // >= 1 ms para apagar de verdad el emisor (menos es solo atenuar)
constexpr uint32_t kAsentarEncendidoUs = 200;  // espera tras encender el emisor antes de leer

enum class Emisor : uint8_t { PAR = 0, SIEMPRE_ON = 1, SIEMPRE_OFF = 2 };
Emisor g_modo = Emisor::PAR;

struct Stats {
    uint32_t n = 0;
    int32_t  dif_min = 32767, dif_max = -32768;
    int64_t  dif_sum = 0;
    uint16_t off_min = 65535, off_max = 0;
    uint16_t on_min = 65535, on_max = 0;
    void reset() { *this = Stats(); }
    void add(uint16_t off, uint16_t on, int16_t dif) {
        ++n; dif_sum += dif;
        if (dif < dif_min) dif_min = dif;
        if (dif > dif_max) dif_max = dif;
        if (off < off_min) off_min = off;
        if (off > off_max) off_max = off;
        if (on < on_min) on_min = on;
        if (on > on_max) on_max = on;
    }
};
Stats g_left, g_right;

uint16_t LeerPromedio(uint8_t pin) {
    uint32_t suma = 0;
    for (int i = 0; i < kMuestrasPromedio; ++i) suma += (uint32_t)analogRead(pin);
    return (uint16_t)(suma / kMuestrasPromedio);
}

const char *Clasificar(uint16_t off, uint16_t on, int16_t dif) {
    if (off >= kTopeADC && on >= kTopeADC) return "PEGADO-AL-TOPE";
    return (abs(dif) < kBordeRestadoUmbral) ? "NEGRO" : "CLARO";
}

void PrintStats(const char *nombre, const Stats &s) {
    if (s.n == 0) { DEBUG_LINK.printf("  %s: sin muestras\n", nombre); return; }
    DEBUG_LINK.printf("  %s: n=%u  off[%u..%u]  on[%u..%u]  dif[min %d, media %d, max %d]\n",
                      nombre, (unsigned)s.n, s.off_min, s.off_max, s.on_min, s.on_max,
                      (int)s.dif_min, (int)(s.dif_sum / (int64_t)s.n), (int)s.dif_max);
}

void PrintHelp() {
    DEBUG_LINK.println("Comandos: r = reiniciar estadisticas | s = resumen | 0 = par off/on | 1 = emisor siempre ON | 2 = emisor siempre OFF | ? = ayuda");
}

void HandleChar(char c) {
    switch (c) {
        case 'r': g_left.reset(); g_right.reset(); DEBUG_LINK.println("[stats] reiniciadas"); break;
        case 's':
            DEBUG_LINK.println("[resumen desde el ultimo 'r']");
            PrintStats("IZQ", g_left);
            PrintStats("DER", g_right);
            break;
        case '0': g_modo = Emisor::PAR;         DEBUG_LINK.println("[modo] par off/on (normal)"); break;
        case '1': g_modo = Emisor::SIEMPRE_ON;  DEBUG_LINK.println("[modo] emisor SIEMPRE ON (dif no aplica)"); break;
        case '2': g_modo = Emisor::SIEMPRE_OFF; DEBUG_LINK.println("[modo] emisor SIEMPRE OFF (dif no aplica)"); break;
        case '?': PrintHelp(); break;
        default: break;
    }
}

void setup() {
    DEBUG_LINK.begin(115200);
    delay(300);
    DEBUG_LINK.println("\nDiagnostico QTR izquierdo/derecho (emisor off/on, dif = on - off)");

    analogReadResolution(12);
    analogSetPinAttenuation(Pins::QTR_LEFT_OUT, ADC_11db);
    analogSetPinAttenuation(Pins::QTR_RIGHT_OUT, ADC_11db);
    pinMode(Pins::QTR_EMITTER_CTRL, OUTPUT);
    digitalWrite(Pins::QTR_EMITTER_CTRL, HIGH);

    PrintHelp();
}

void loop() {
    while (DEBUG_LINK.available() > 0) HandleChar((char)DEBUG_LINK.read());

    uint16_t l_off, l_on, r_off, r_on;
    switch (g_modo) {
        case Emisor::PAR:
            digitalWrite(Pins::QTR_EMITTER_CTRL, LOW);
            delay(kApagadoEmisorMs);
            l_off = LeerPromedio(Pins::QTR_LEFT_OUT);
            r_off = LeerPromedio(Pins::QTR_RIGHT_OUT);
            digitalWrite(Pins::QTR_EMITTER_CTRL, HIGH);
            delayMicroseconds(kAsentarEncendidoUs);
            l_on = LeerPromedio(Pins::QTR_LEFT_OUT);
            r_on = LeerPromedio(Pins::QTR_RIGHT_OUT);
            break;
        case Emisor::SIEMPRE_ON:
            digitalWrite(Pins::QTR_EMITTER_CTRL, HIGH);
            delay(kApagadoEmisorMs);
            l_off = l_on = LeerPromedio(Pins::QTR_LEFT_OUT);
            r_off = r_on = LeerPromedio(Pins::QTR_RIGHT_OUT);
            break;
        default:  // SIEMPRE_OFF
            digitalWrite(Pins::QTR_EMITTER_CTRL, LOW);
            delay(kApagadoEmisorMs);
            l_off = l_on = LeerPromedio(Pins::QTR_LEFT_OUT);
            r_off = r_on = LeerPromedio(Pins::QTR_RIGHT_OUT);
            break;
    }

    const int16_t l_dif = (int16_t)((int32_t)l_on - (int32_t)l_off);
    const int16_t r_dif = (int16_t)((int32_t)r_on - (int32_t)r_off);
    g_left.add(l_off, l_on, l_dif);
    g_right.add(r_off, r_on, r_dif);

    static uint32_t ultimo_print_ms = 0;
    if ((uint32_t)(millis() - ultimo_print_ms) >= kPeriodoImpresionMs) {
        ultimo_print_ms = millis();
        if (g_modo == Emisor::PAR) {
            DEBUG_LINK.printf("IZQ off=%4u on=%4u dif=%4d %-14s | DER off=%4u on=%4u dif=%4d %s\n",
                              l_off, l_on, l_dif, Clasificar(l_off, l_on, l_dif),
                              r_off, r_on, r_dif, Clasificar(r_off, r_on, r_dif));
        } else {
            DEBUG_LINK.printf("IZQ lectura=%4u | DER lectura=%4u   (emisor forzado)\n", l_on, r_on);
        }
    }
    delay(20);
}
