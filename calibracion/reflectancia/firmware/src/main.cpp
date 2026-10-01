// ===========================================================================
//  Calibración IR — banco de pruebas (ESP32-S3)
//  Athena Rover 2026 · Retos del Rover H07 · INTEC · Reymildo & Montse
// ===========================================================================
//
//  No es el firmware del rover (ese vive en firmware-esp32/), pero SÍ corre
//  sobre el mismo ESP32-S3 — el equipo usa un solo microcontrolador en todo
//  el proyecto. Este sketch se sube aparte, con nada más conectado que los
//  sensores bajo prueba (sin motores, sin PCA9685, sin TCS34725), para
//  caracterizarlos antes de fijar umbrales — ver ../README.md.
//
//  ACTUALIZADO 2026-10-01: antes leía con QTRSensors en modo normal (emisor
//  siempre encendido, valor crudo). Eso ya no es lo que hace el robot: desde
//  el 2026-09-06 v8/v9 miden con rechazo de luz ambiente -- emisor IR
//  apagado (>= 1 ms) -> lectura "off"; emisor encendido -> lectura "on";
//  dif = on - off, y es la DIFERENCIA lo que se compara contra el umbral.
//  Este sketch mide con EXACTAMENTE esa secuencia (la de ReflectanceTask en
//  v9), y manda off y on de cada sensor para que el script calcule dif y se
//  vea cómo se separan NEGRO y GRIS con el método real. Si midiera distinto
//  que el robot, los datos no servirían para fijar umbrales.
//
//  Pines = cableado real (hardware/conexiones-esp32-s3.md): QTR izquierdo
//  GPIO1, QTR derecho GPIO2, CTRL de los emisores GPIO42 (compartido, se
//  encienden y apagan juntos). ¡Los QTRX van a 3.3 V, nunca a 5 V!
//
//  Protocolo: este sketch no hace nada por su cuenta. Se queda esperando un
//  comando por serial y responde una lectura cada vez que lo recibe. La
//  orquestación (puntos, muestras, pausas, CSV) vive en ../calibrar_ir.py —
//  si cambias el formato de aquí, cámbialo allá también.
//
//    Comando recibido : 'R'
//    Respuesta enviada : DATA,<off_izq>,<on_izq>,<off_der>,<on_der>
//
// ===========================================================================

#include <Arduino.h>

constexpr uint8_t kPinQtrIzquierdo = 1;    // ADC1_CH0
constexpr uint8_t kPinQtrDerecho   = 2;    // ADC1_CH1
constexpr uint8_t kPinEmisor       = 42;   // CTRL de los emisores IR, compartido

// Lecturas del ADC que se promedian en CADA medida (off y on, cada una). El
// ADC del ESP32 es ruidoso; promediar baja ese ruido sin tocar el hardware.
// Debe ser el MISMO valor que kMuestrasPromedio de ../detector-negro-gris/:
// si se captura con un promedio y se detecta con otro, el umbral que salga
// de aquí no vale allá. Cada lectura del ADC cuesta decenas de microsegundos.
constexpr uint8_t kMuestrasPromedio = 16;

uint16_t LeerPromedio(uint8_t pin) {
    uint32_t suma = 0;
    for (uint8_t i = 0; i < kMuestrasPromedio; ++i) suma += (uint32_t)analogRead(pin);
    return (uint16_t)(suma / kMuestrasPromedio);
}

void readSensors() {
    // Misma secuencia que ReflectanceTask en v9.
    digitalWrite(kPinEmisor, LOW);
    delay(2);   // >= 1 ms: apagado real
    const uint16_t offIzq = LeerPromedio(kPinQtrIzquierdo);
    const uint16_t offDer = LeerPromedio(kPinQtrDerecho);

    digitalWrite(kPinEmisor, HIGH);
    delayMicroseconds(200);   // asentar el fototransistor con luz IR estable
    const uint16_t onIzq = LeerPromedio(kPinQtrIzquierdo);
    const uint16_t onDer = LeerPromedio(kPinQtrDerecho);

    // IMPORTANTE: el formato de salida debe mantenerse en sincronía con
    // ../calibrar_ir.py: DATA,off_izq,on_izq,off_der,on_der
    Serial.print("DATA,");
    Serial.print(offIzq); Serial.print(",");
    Serial.print(onIzq);  Serial.print(",");
    Serial.print(offDer); Serial.print(",");
    Serial.println(onDer);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Resolución y atenuación fijadas a propósito: sin esto dependen del
    // valor por defecto del core y las lecturas se desplazan de sesión en
    // sesión (ver el historial de este archivo). Mismas que firmware-esp32/
    // y v9.
    analogReadResolution(12);
    analogSetPinAttenuation(kPinQtrIzquierdo, ADC_11db);
    analogSetPinAttenuation(kPinQtrDerecho, ADC_11db);

    pinMode(kPinEmisor, OUTPUT);
    digitalWrite(kPinEmisor, HIGH);

    Serial.println("READY");
}

void loop() {
    if (Serial.available()) {
        const char command = Serial.read();

        if (command == 'R') {
            readSensors();
        }

        // Descarta cualquier byte extra (\r, \n, etc.) que haya llegado junto
        // al comando, para que no se procese por error en el siguiente ciclo.
        while (Serial.available()) {
            Serial.read();
        }
    }
}
