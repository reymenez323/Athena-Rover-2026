// Banco de prueba: tira WS2812B de 8 LED soldada por Montse (2026-09-27).
// GND y VCC al buck a 5V, DATA (IN) al GPIO 39. Consola por Serial0 (COM11), 115200.
// Basado en el strandtest oficial de Adafruit (colorWipe/theaterChase/rainbow):
// https://github.com/adafruit/Adafruit_NeoPixel/blob/master/examples/strandtest/strandtest.ino
//
// Comandos (una letra + Enter en el monitor serie):
//   r = rojo solido (equipo ROJO)      b = azul solido (equipo AZUL)
//   w = colorWipe                       t = theaterChase
//   c = rainbowCycle                    h = theaterChaseRainbow
//   1..8 = enciende solo ese LED (verifica el orden fisico)          x = apagar
//
// Los efectos largos (c, h) revisan el puerto serie en cada paso: cualquier
// tecla nueva los corta al instante, en vez de esperar a que terminen solos
// (2026-09-27: "h" tardaba ~92 s corridos y no reaccionaba a nada mientras tanto).

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ---- PARAMETROS (editar aqui) ----------------------------------------------
constexpr uint8_t  kPinTira = 39;   // DATA de la tira (antes era el canal R del LED RGB)
constexpr uint16_t kNumLeds = 8;
constexpr uint8_t  kBrillo  = 40;   // 0-255. A 40/255 (~15%) toda la tira consume ~100 mA. NO subir sin medir la fuente
// -----------------------------------------------------------------------------

Adafruit_NeoPixel tira(kNumLeds, kPinTira, NEO_GRB + NEO_KHZ800);

// true si llego una tecla nueva por el puerto serie -- se usa para cortar un
// efecto largo a medio camino, sin consumirla (la procesa loop() despues).
bool HayTeclaNueva() {
    return Serial0.available() > 0;
}

void colorWipe(uint32_t c, int esperaMs) {
    for (int i = 0; i < tira.numPixels(); i++) {
        if (HayTeclaNueva()) return;
        tira.setPixelColor(i, c);
        tira.show();
        delay(esperaMs);
    }
}

void theaterChase(uint32_t c, int esperaMs) {
    for (int a = 0; a < 10; a++) {
        for (int b = 0; b < 3; b++) {
            if (HayTeclaNueva()) return;
            tira.clear();
            for (int i = b; i < tira.numPixels(); i += 3) tira.setPixelColor(i, c);
            tira.show();
            delay(esperaMs);
        }
    }
}

void rainbowCycle(int esperaMs) {
    for (long j = 0; j < 256 * 3; j++) {
        if (HayTeclaNueva()) return;
        for (int i = 0; i < tira.numPixels(); i++) {
            int idx = (i * 256 / tira.numPixels() + j) & 255;
            tira.setPixelColor(i, tira.gamma32(tira.ColorHSV(idx * 256)));
        }
        tira.show();
        delay(esperaMs);
    }
}

void theaterChaseRainbow(int esperaMs) {
    for (int j = 0; j < 256; j++) {
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                if (HayTeclaNueva()) return;
                tira.clear();
                for (int i = b; i < tira.numPixels(); i += 3) {
                    int idx = (i * 256 / tira.numPixels() + j) & 255;
                    tira.setPixelColor(i, tira.gamma32(tira.ColorHSV(idx * 256)));
                }
                tira.show();
                delay(esperaMs);
            }
        }
    }
}

void unoPorVez(int n) {
    tira.clear();
    tira.setPixelColor(n - 1, tira.Color(0, 60, 0));
    tira.show();
    Serial0.printf("LED %d encendido (verde)\n", n);
}

void setup() {
    Serial0.begin(115200);
    delay(300);
    tira.begin();
    tira.setBrightness(kBrillo);
    tira.clear();
    tira.show();
    Serial0.println("\nBanco de la tira listo (GPIO 39, 8 LED).");
    Serial0.println("Comandos: r=rojo  b=azul  w=colorWipe  t=theaterChase  c=rainbowCycle  h=chaseRainbow  1-8=un LED  x=apagar");
    Serial0.println("(c y h se pueden cortar con cualquier tecla)");
}

void loop() {
    if (!Serial0.available()) return;
    char cmd = Serial0.read();

    // Muchos monitores serie mandan un salto de linea (\r y/o \n) justo
    // detras de cada letra. Sin descartarlo aqui, ese caracter sobrante
    // queda esperando en el buffer y los efectos largos (que revisan el
    // puerto para poder cancelarse) se cortaban antes de dibujar el primer
    // cuadro -- parecia que la tira no encendia (2026-09-27).
    delay(5);
    while (Serial0.available() && (Serial0.peek() == '\r' || Serial0.peek() == '\n')) {
        Serial0.read();
    }

    switch (cmd) {
        case 'r': tira.fill(tira.Color(255, 0, 0)); tira.show(); Serial0.println("ROJO"); break;
        case 'b': tira.fill(tira.Color(0, 0, 255)); tira.show(); Serial0.println("AZUL"); break;
        case 'w': Serial0.println("colorWipe"); colorWipe(tira.Color(0, 150, 0), 120); break;
        case 't': Serial0.println("theaterChase"); theaterChase(tira.Color(127, 127, 127), 80); break;
        case 'c': Serial0.println("rainbowCycle"); rainbowCycle(10); break;
        case 'h': Serial0.println("theaterChaseRainbow"); theaterChaseRainbow(40); break;
        case 'x': tira.clear(); tira.show(); Serial0.println("apagado"); break;
        default:
            if (cmd >= '1' && cmd <= '8') unoPorVez(cmd - '0');
            break;
    }
}
