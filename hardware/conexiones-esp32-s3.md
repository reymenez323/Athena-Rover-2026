# Conexiones al ESP32-S3 — Athena Rover 2026

Generado a partir de los pines reales declarados en `firmware-esp32/src/main.cpp`
(namespace `Pins`). **Si cambias un pin en el código, actualiza esta tabla.**

Placa asumida: **ESP32-S3-DevKitC-1**.

## Índice

1. [Antes de conectar nada — cinco cosas que queman hardware](#️-antes-de-conectar-nada--cinco-cosas-que-queman-hardware)
2. [Pines PROHIBIDOS del ESP32-S3](#pines-prohibidos-del-esp32-s3)
3. [Código de colores de cableado](#código-de-colores-de-cableado)
4. [Motores — 2× L298N](#motores--2-l298n)
5. [Servos — PCA9685](#servos--pca9685-i2c-dirección-0x40)
6. [Sensores de color — 2× TCS34725](#sensores-de-color--2-tcs34725)
7. [Reflectancia — 2× QTRX-HD-01A](#reflectancia--2-qtrx-hd-01a)
8. [LED de equipo](#led-de-equipo-lo-exige-el-reglamento)
9. [Enlace con la Raspberry Pi 4B](#enlace-con-la-raspberry-pi-4b)
10. [Resumen: mapa completo de pines usados](#resumen-mapa-completo-de-pines-usados)
11. [Alimentación — esquema recomendado](#alimentación--esquema-recomendado)
12. [Orden sugerido para el montaje y las pruebas](#orden-sugerido-para-el-montaje-y-las-pruebas)

---

## ⚠️ Antes de conectar nada — cinco cosas que queman hardware

| # | Riesgo | Qué hacer |
|---|--------|-----------|
| 1 | **QTRX-HD-01A a 5 V** | Alimentarlos a **3.3 V**. Su salida analógica es proporcional a su VIN: a 5 V entregan hasta 5 V a un pin de ADC que solo tolera 3.3 V. Quema la entrada. |
| 2 | **Jumpers ENA/ENB del L298N puestos** | **Quitarlos.** Con el jumper, el enable queda fijo a 5 V y el PWM no hace nada: los motores solo giran a fondo o nada. |
| 3 | **Salida de 5 V del L298N al ESP32** | **No conectarla.** El regulador de 5 V del L298N no debe alimentar ni tocar los 3.3 V del ESP32. |
| 4 | **Servos alimentados desde el ESP32** | El PCA9685 necesita su **propia fuente de 5–6 V** en el borne V+. Dos servos con carga piden picos de más de 1 A: el regulador del DevKit no los aguanta y el ESP32 se reinicia. |
| 5 | **Masas separadas** | **Todas las masas van unidas**: ESP32, ambos L298N, PCA9685, sensores y las baterías. Sin masa común, las señales lógicas no tienen referencia y el comportamiento es aleatorio. |

---

## Pines PROHIBIDOS del ESP32-S3

No usar ninguno de estos, aunque el DevKit los saque al conector:

| GPIO | Por qué |
|------|---------|
| 19, 20 | USB nativo (D‑/D+). **Es el enlace con la Raspberry Pi.** |
| 43, 44 | U0TXD / U0RXD — puerto de programación y consola de depuración. |
| 26–32 | Flash SPI interna. Tocarlos cuelga el chip. |
| 33–37 | PSRAM Octal (módulos N8R8 / N16R8). |
| 0, 45, 46 | Pines de *strapping*: su nivel al arrancar decide el modo de boot. |
| 3 | Strapping de JTAG. Se puede usar, pero mejor dejarlo libre. |

---

## Código de colores de cableado

El robot tiene **cuatro dominios de voltaje/masa distintos** (GND, 3.3 V
lógica, 5–6 V servos, 7.4–12 V motores) conviviendo en el mismo chasis —
justo la clase de mezcla que aparece en la tabla de riesgos de arriba. Usar
el mismo color de cable para dos cosas distintas es la forma más fácil de
meter una pata en la oscuridad debajo del chasis.

Ajustada al carrete real del equipo (**negro, rojo, amarillo, verde, azul,
blanco** — 6 colores, no 9), la prioridad es, en este orden:

1. **Cada línea de alimentación tiene su propio color, sin compartir con
   ninguna otra** — son las que revientan hardware.
2. **SDA y SCL van en colores distintos entre sí**, porque son dos cables
   que corren pegados hasta el mismo conector y cambiarlos es el descuido
   más fácil de cometer con la mano.
3. Con los 6 colores ya repartidos entre lo anterior, el resto de señales
   (control de motores, salidas de los QTR, LED de equipo) reutiliza el
   color de Azul o Blanco según le corresponda por tipo — **no pasa nada
   porque nunca conviven en el mismo conector que el I2C**: los cables junto
   al L298N o al QTR no se van a confundir por el ojo con los que salen del
   PCA9685/TCS34725, que están en otra zona del chasis. Si se cruzaran, el
   peor caso es que el robot se porte mal, no que se queme algo.

| Color | Uso |
|-------|-----|
| **Negro** | GND — todas las masas, sin excepción |
| **Rojo** | Potencia de motores — batería 7.4–12 V hacia los L298N. Nada más lleva rojo, ni siquiera el LED rojo de equipo. |
| **Amarillo** | Lógica 3.3 V — VIN de los QTRX y los TCS34725. Nunca a 5 V (revisar la tabla de riesgos: quema el sensor). |
| **Verde** | Potencia de servos — BEC/batería 5–6 V hacia el **V+** del PCA9685 |
| **Azul** | **I2C — SDA, en ambos buses.** También: PWM/salidas analógicas de motores y QTR (ENA/ENB, OUT) — no se mezclan con el I2C porque están en zonas distintas del chasis. |
| **Blanco** | **I2C — SCL, en ambos buses.** También: señales digitales de control (IN1–IN4 de los L298N, CTRL de los QTR) y el control de los 2 LED de equipo — mismo razonamiento. |

En el tramo del PCA9685/TCS34725, donde SDA y SCL corren juntos, no hace
falta marquilla: el color ya dice cuál es cuál. Si en algún otro punto
tuvieras dos cables Azul o dos Blanco muy cerca uno del otro (por ejemplo,
un ENA junto a un OUT de QTR), ahí sí marca la punta.

Dos reglas simples que evitan la mayoría de los sustos:

- **El rojo es solo para la batería de motores, sin excepción.** Es la línea
  que más corriente mueve, así que un cruce ahí es el que más daño hace.
  Aunque el LED de equipo sea rojo, su cable de control va en **blanco**
  (es una señal digital de bajo amperaje, no una línea de potencia) — así se
  evita la tentación de "total, ya tengo rojo a mano".
- **Corta el negro y el color de señal de cada conector al mismo largo.** Así
  se identifican por tacto (o a simple vista) cuál masa va con cuál señal sin
  tener que seguir el cable completo.

---

## Motores — 2× L298N

Cada driver mueve dos motores. El firmware controla cada lado en conjunto
(los dos motores izquierdos reciben siempre la misma consigna).

### L298N nº 1 — lado IZQUIERDO

| Pin L298N | GPIO ESP32-S3 | Función |
|-----------|:-------------:|---------|
| IN1 | **4** | Motor delantero izq. — sentido A |
| IN2 | **5** | Motor delantero izq. — sentido B |
| ENA | **6** | Motor delantero izq. — velocidad (PWM) |
| IN3 | **7** | Motor trasero izq. — sentido A |
| IN4 | **15** | Motor trasero izq. — sentido B |
| ENB | **16** | Motor trasero izq. — velocidad (PWM) |
| GND | GND | Masa común (obligatorio) |
| 12 V | Batería de motores | Alimentación de potencia |

### L298N nº 2 — lado DERECHO

| Pin L298N | GPIO ESP32-S3 | Función |
|-----------|:-------------:|---------|
| IN1 | **10** | Motor delantero der. — sentido A |
| IN2 | **11** | Motor delantero der. — sentido B |
| ENA | **12** | Motor delantero der. — velocidad (PWM) |
| IN3 | **13** | Motor trasero der. — sentido A |
| IN4 | **14** | Motor trasero der. — sentido B |
| ENB | **17** | Motor trasero der. — velocidad (PWM) |
| GND | GND | Masa común (obligatorio) |
| 12 V | Batería de motores | Alimentación de potencia |

> PWM a **1 kHz**. El L298N es un driver bipolar antiguo: a 20 kHz calienta y
> pierde par. A 1 kHz se oye un zumbido agudo — es normal, no está fallando.

---

## Servos — PCA9685 (I2C, dirección 0x40)

| Pin PCA9685 | Conexión | Nota |
|-------------|----------|------|
| VCC | 3.3 V del ESP32 | Solo la lógica del chip |
| **V+** | **Fuente aparte de 5–6 V** | Alimentación de los servos. **No** desde el ESP32. |
| GND | GND común | |
| SDA | **GPIO 8** | Bus I2C nº 0 |
| SCL | **GPIO 9** | Bus I2C nº 0 |

| Canal PCA9685 | Servo |
|:-------------:|-------|
| **0** | Pinza (abrir / cerrar) |
| **1** | Elevación del gripper (subir / bajar) |

---

## Sensores de color — 2× TCS34725

**Los dos sensores tienen la misma dirección fija (0x29) y no se puede cambiar.**
Por eso van en **buses I2C separados**: el ESP32-S3 tiene dos controladores I2C,
así te ahorras el multiplexor TCA9548A.

### Sensor DELANTERO — bus I2C nº 0 (compartido con el PCA9685)

| Pin | GPIO ESP32-S3 |
|-----|:-------------:|
| SDA | **8** |
| SCL | **9** |
| VIN | 3.3 V |
| GND | GND |
| LED | **18** |

### Sensor TRASERO — bus I2C nº 1 (dedicado)

| Pin | GPIO ESP32-S3 |
|-----|:-------------:|
| SDA | **47** |
| SCL | **48** |
| VIN | 3.3 V |
| GND | GND |
| LED | **21** |

> Cada bus necesita resistencias de pull-up de 4.7 kΩ a 3.3 V en SDA y SCL.
> La mayoría de los módulos TCS34725 y PCA9685 ya las traen: si pones tres
> módulos con pull-ups en el mismo bus, la resistencia equivalente baja
> demasiado. Si el I2C falla, es lo primero que hay que revisar.

### LED de iluminación del propio TCS34725

Cada módulo trae 2 LED blancos para iluminar la superficie que está leyendo
(necesarios para no depender de la luz ambiente del salón, que cambia y
arruina la clasificación de color). Se controlan con el pin marcado **LED**
en la placa.

> Confirmado: el equipo usa un clon del diseño de referencia de Adafruit, así
> que el pin **LED** es activo en alto y trae su propio pull-up hacia VIN —
> si lo dejas sin conectar, los LED quedan encendidos siempre. Conectarlo a
> un GPIO da control por software (apagarlos cuando no hacen falta, o evitar
> que el sensor delantero le meta luz al trasero).

| Señal | GPIO ESP32-S3 |
|-------|:-------------:|
| LED sensor delantero | **18** |
| LED sensor trasero | **21** |

Van en cable **blanco** (señal digital de control, como el resto — ver el
[código de colores](#código-de-colores-de-cableado)), no en el color de
ninguna línea de alimentación.

---

## Reflectancia — 2× QTRX-HD-01A

| Señal | GPIO ESP32-S3 | Nota |
|-------|:-------------:|------|
| OUT izquierdo | **1** | ADC1_CH0 |
| OUT derecho | **2** | ADC1_CH1 |
| CTRL (ambos) | **42** | Control de los emisores IR, compartido |
| VIN | **3.3 V** | ⚠️ **Nunca 5 V** — ver la tabla de riesgos |
| GND | GND | |

> Tienen que ir en **ADC1** (GPIO 1–10). El ADC2 del ESP32 queda inutilizable
> en cuanto se enciende el WiFi.
>
> **Cómo leen, que es contraintuitivo:** el QTR entrega voltaje *inversamente*
> proporcional a la reflectancia. Superficie clara → mucha luz IR rebotada →
> salida cerca de 0 V. Superficie oscura → salida cerca de VIN.
> O sea: **valor ADC alto = cinta negra**.

---

## LED de equipo (lo exige el reglamento)

| LED | GPIO ESP32-S3 | Nota |
|-----|:-------------:|------|
| Rojo | **40** | Con resistencia en serie de 220–330 Ω |
| Azul | **41** | Con resistencia en serie de 220–330 Ω |

> Aunque el LED sea rojo o azul, su cable de control va en **blanco**, como
> el resto de las señales digitales — ver el [código de colores](#código-de-colores-de-cableado).
> No uses rojo para el LED rojo: ese color está reservado sin excepción para
> la batería de motores.

---

## Enlace con la Raspberry Pi 4B

Cable **USB-A (Pi) → USB-C del puerto "USB" del DevKit** — el puerto del USB
nativo, **no** el puerto "UART".

| | |
|---|---|
| En la Raspberry Pi aparece como | `/dev/ttyACM0` |
| Velocidad | 115200 baudios |
| GPIO involucrados | 19 y 20 (internos, no se cablean) |

Ventaja de usar el USB nativo en vez del puerto UART: el puerto "UART" del
DevKit queda libre para depurar con un segundo cable mientras el robot está
conversando con la Pi.

Si `/dev/ttyACM0` no aparece, revisa con `ls /dev/ttyACM*` y ajusta
`serial_port` en la configuración de la Raspberry Pi.

---

## Resumen: mapa completo de pines usados

| GPIO | Destino | GPIO | Destino |
|:----:|---------|:----:|---------|
| 1 | QTR izquierdo (ADC) | 14 | L298N‑D IN4 |
| 2 | QTR derecho (ADC) | 15 | L298N‑I IN4 |
| 4 | L298N‑I IN1 | 16 | L298N‑I ENB |
| 5 | L298N‑I IN2 | 17 | L298N‑D ENB |
| 6 | L298N‑I ENA | 18 | LED TCS34725 delantero |
| 7 | L298N‑I IN3 | 21 | LED TCS34725 trasero |
| 8 | I2C0 SDA | 40 | LED rojo |
| 9 | I2C0 SCL | 41 | LED azul |
| 10 | L298N‑D IN1 | 42 | QTR emisores (CTRL) |
| 11 | L298N‑D IN2 | 47 | I2C1 SDA |
| 12 | L298N‑D ENA | 48 | I2C1 SCL |
| 13 | L298N‑D IN3 | | |

**23 pines usados.** Quedan libres para ampliaciones: GPIO **38, 39**
(y 3, con cuidado por el strapping de JTAG).

---

## Alimentación — esquema recomendado

```
Batería de motores (7.4–12 V)
   ├──> L298N nº1  (12V)
   └──> L298N nº2  (12V)

Batería / BEC 5–6 V
   ├──> PCA9685 V+  (servos)
   └──> Raspberry Pi 4B (5 V, 3 A)

ESP32-S3
   └──> alimentado por el cable USB de la Raspberry Pi

TODAS LAS MASAS UNIDAS EN UN SOLO PUNTO
```

Se separan las alimentaciones a propósito: los picos de corriente de los
motores hunden la tensión, y si la lógica cuelga de la misma fuente, el ESP32
se reinicia justo cuando el robot arranca. Sucede siempre y cuesta horas de
depuración.

---

## Orden sugerido para el montaje y las pruebas

1. **Solo el ESP32** por USB. Cargar el firmware, ver el mensaje de arranque en el
   puerto de depuración a 115200.
2. **Añadir los LED** (GPIO 40/41). Comprobar que la Raspberry Pi los enciende.
3. **Añadir el I2C**, un chip a la vez. El firmware avisa por la consola si el
   PCA9685 o algún TCS34725 no responde.
4. **Añadir los servos** con el gripper DESACOPLADO del mecanismo, para no
   forzarlo contra un tope mientras se calibran los ángulos.
5. **Añadir los QTR.** Verificar en la telemetría que el valor sube al poner
   cinta negra debajo.
6. **Los motores al final**, con el robot en un soporte y las ruedas al aire.
