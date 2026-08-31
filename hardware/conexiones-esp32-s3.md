# Conexiones al ESP32-S3 — Athena Rover 2026

Generado a partir de los pines reales declarados en `firmware-esp32/src/main.cpp`
(namespace `Pins`). **Si cambias un pin en el código, actualiza esta tabla.**

> El robot ya **no tiene los 2 LED discretos rojo/azul** que documentaban
> versiones anteriores de este archivo (GPIO 40/41) — el equipo los
> reemplazó por un único **LED RGB** (sección 8), que ahora cumple la
> identificación de equipo que exige el reglamento y queda disponible para
> cualquier otra señal visual que haga falta más adelante. Ya está integrado
> tanto en `firmware-esp32/` como en `pruebas-platformio/02-cuadro-color-rgb/`,
> con los mismos GPIO en los dos.

Placa asumida: **ESP32-S3-DevKitC-1**.

## Índice

1. [Antes de conectar nada — cinco cosas que queman hardware](#️-antes-de-conectar-nada--cinco-cosas-que-queman-hardware)
2. [Pines PROHIBIDOS del ESP32-S3](#pines-prohibidos-del-esp32-s3)
3. [Código de colores de cableado](#código-de-colores-de-cableado)
4. [Motores — 2× L298N](#motores--2-l298n)
5. [Servos — PCA9685](#servos--pca9685-i2c-dirección-0x40)
6. [Sensores de color — 2× TCS34725](#sensores-de-color--2-tcs34725)
7. [ToF — VL53L1X (distancia frente al gripper)](#tof--vl53l1x-distancia-frente-al-gripper)
8. [Reflectancia — 2× QTRX-HD-01A](#reflectancia--2-qtrx-hd-01a)
9. [LED RGB indicador de equipo](#led-rgb-indicador-de-equipo)
10. [Enlace con la Raspberry Pi 4B](#enlace-con-la-raspberry-pi-4b)
11. [Resumen: mapa completo de pines usados](#resumen-mapa-completo-de-pines-usados)
12. [Alimentación — esquema recomendado](#alimentación--esquema-recomendado)
13. [Orden sugerido para el montaje y las pruebas](#orden-sugerido-para-el-montaje-y-las-pruebas)

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
| **Rojo** | Potencia de motores — batería 7.4–12 V hacia los L298N. Nada más lleva rojo. |
| **Amarillo** | Lógica 3.3 V — VIN de los QTRX y los TCS34725. Nunca a 5 V (revisar la tabla de riesgos: quema el sensor). |
| **Verde** | Potencia de servos — BEC/batería 5–6 V hacia el **V+** del PCA9685. ⚠️ **Única excepción del código**: también lleva la señal LED de cada TCS34725 — ver la nota abajo. |
| **Azul** | **I2C — SDA, en ambos buses.** También: PWM/salidas analógicas de motores y QTR (ENA/ENB, OUT), y alguno de los 3 canales del LED RGB de equipo — no se mezclan con el I2C porque están en zonas distintas del chasis. |
| **Blanco** | **I2C — SCL, en ambos buses.** También: señales digitales de control (IN1–IN4 de los L298N, CTRL de los QTR) y el resto de canales del LED RGB de equipo — mismo razonamiento. |

En el tramo del PCA9685/TCS34725, donde SDA y SCL corren juntos, no hace
falta marquilla: el color ya dice cuál es cuál. Si en algún otro punto
tuvieras dos cables Azul o dos Blanco muy cerca uno del otro (por ejemplo,
un ENA junto a un OUT de QTR), ahí sí marca la punta.

> ⚠️ **Excepción documentada — Verde ya no es exclusivo de servos.** El
> equipo cableó la señal LED del TCS34725 (ver más abajo) en verde, el mismo
> color que el V+ de 5–6 V del PCA9685. A diferencia del resto de las
> reutilizaciones de esta tabla (que son señales de bajo riesgo en zonas
> separadas), **este caso sí conviven cerca**: el TCS34725 delantero está en
> el mismo bus I2C que el PCA9685, así que es fácil tener un verde de cada
> uno en el mismo tramo de cableado. **Marca la punta de todo cable verde**
> (cinta o marquilla: "V+" para el de servos, "LED" para el del sensor) —
> aquí sí importa, porque confundirlos significa meterle 5–6 V a un pin
> lógico de 3.3 V del TCS34725, que es del tipo de error que sí quema algo.

Dos reglas simples que evitan la mayoría de los sustos:

- **El rojo es solo para la batería de motores, sin excepción.** Es la línea
  que más corriente mueve, así que un cruce ahí es el que más daño hace.
  Aunque el LED RGB de equipo pueda encender en rojo, sus 3 cables de señal
  van en **azul/blanco** (son PWM de bajo amperaje, no una línea de
  potencia) — así se evita la tentación de "total, ya tengo rojo a mano".
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

> ⚠️ **El motor en OUT1/OUT2 de este driver gira al revés de los otros
> tres.** Con el cableado físico actual, el motor conectado ahí (la rueda
> trasera izquierda del chasis, no la delantera pese a como está descrita
> la tabla de arriba) queda invertido respecto a las otras tres ruedas. La
> corrección **no está aquí, en el cableado** — está en software, en cada
> proyecto que controla motores (`firmware-esp32/src/main.cpp` y
> `pruebas-platformio/{01-mantente-en-cuadro,02-cuadro-color-rgb,03-motores-adelante}/src/main.cpp`):
> la definición de ese motor recibe **GPIO 5 (IN2) como "in1" y GPIO 4 (IN1)
> como "in2"**, intercambiados a propósito, en vez de un flag de inversión o
> una función aparte. Si en algún momento se recablean físicamente los 2
> cables de ese motor para que coincida con los otros tres, hay que
> **deshacer el intercambio en los 4 archivos de código** (volver a pasar
> IN1, IN2 en orden) — esta tabla seguiría describiendo la conexión física
> real (GPIO 4 → IN1, GPIO 5 → IN2), eso no cambia.

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

Van en cable **verde** — la única excepción del [código de colores](#código-de-colores-de-cableado):
verde también es el color del V+ de 5–6 V de los servos, así que **marca la
punta de cada cable verde** ("V+" o "LED") al conectarlo. Aquí sí importa
más que en las demás reutilizaciones de la tabla: el TCS34725 delantero
comparte bus I2C (y zona de cableado) con el PCA9685, y meterle 5–6 V al pin
LED por error sí puede dañar el sensor.

---

## ToF — VL53L1X (distancia frente al gripper)

Mide la distancia a lo que tenga delante del gripper (la bandera) para que la
Raspberry Pi sepa cuándo cerrar la pinza — la lógica de "cuándo" vive en
`raspberry-pi/src/athena/decision.py`, el ESP32 solo mide y reporta.

**Dirección I2C fija de fábrica: 0x29 — igual que AMBOS TCS34725.** No hay
forma de elegir otra dirección desde el pin ni por strapping. Por eso NO va en
un bus propio: comparte el bus I2C nº1 con el TCS34725 trasero, y su pin
**XSHUT** es imprescindible (no opcional) para poder arrancar sin que las dos
direcciones choquen.

| Pin VL53L1X | GPIO ESP32-S3 / Conexión | Nota |
|-------------|:------------------------:|------|
| SDA | **47** | Bus I2C nº 1 — compartido con el TCS34725 trasero |
| SCL | **48** | Bus I2C nº 1 — compartido con el TCS34725 trasero |
| XSHUT | **40** | Reset por software. Ver la secuencia de arranque abajo |
| VIN | 3.3 V | |
| GND | GND común | |

### ⚠️ Por qué hace falta XSHUT, y el riesgo que queda sin resolver

El VL53L1X arranca siempre en 0x29. El TCS34725 trasero, en el mismo bus, está
fijo en esa misma dirección **y no tiene ningún pin de apagado**: en cuanto
tiene alimentación, responde en 0x29 sin que el firmware pueda callarlo. La
secuencia de arranque (implementada en `TofSensorTask`, `firmware-esp32/src/main.cpp`)
es:

1. **GPIO 40 en LOW desde `setup()`**, antes de crear ninguna tarea — el
   VL53L1X queda en reset y no contesta en el bus. Esto es importante hacerlo
   ANTES de arrancar cualquier tarea: algunas placas del sensor traen un
   pull-up en XSHUT que lo deja activo apenas se energiza, así que si el
   firmware tardara en ponerlo en LOW, habría una ventana de arranque con los
   dos chips respondiendo en 0x29 a la vez.
2. GPIO 40 a HIGH: el sensor sale de reset y arranca (~1.2 ms).
3. Se le reasigna la dirección **0x30** con una única escritura corta a su
   registro de dirección. Este es el único instante en que el VL53L1X sigue
   en 0x29 mientras el TCS34725 trasero también está vivo ahí — no se puede
   evitar con el hardware actual sin agregarle un pin de apagado al TCS34725
   (por ejemplo, cortando su alimentación con un transistor). El equipo
   decidió aceptar este riesgo acotado (una sola transacción de 3 bytes, no
   la inicialización completa del sensor) en vez de sumar ese hardware extra.
4. Recién ahora corre la inicialización completa del VL53L1X, ya en 0x30 y
   sin nadie más escuchando ahí.

**Si el TCS34725 trasero empieza a dar lecturas raras justo después de un
reset del ESP32 (y no antes), este es el primer sospechoso.** Revisar con un
analizador lógico si se repite: se vería como una transacción I2C corta a
0x29 justo después del arranque, seguida de dos direcciones I2C distintas
conviviendo en el mismo bus.

> El VL53L1X se inicializa con la librería de PlatformIO `pololu/VL53L1X`
> (ver `firmware-esp32/platformio.ini`) — es la única dependencia externa de
> todo el firmware. A diferencia del PCA9685 o el TCS34725 (unos pocos
> registros, documentados a mano en el propio `main.cpp`), el VL53L1X necesita
> un algoritmo de medición completo con decenas de valores de calibración:
> reescribirlo no aportaba nada frente a una librería madura y bien probada.

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

## LED RGB indicador de equipo

Un solo LED RGB en el chasis cumple la identificación de equipo que exige el
reglamento (antes eran 2 LED discretos, rojo y azul — **el equipo ya no los
tiene montados**, quedaron completamente reemplazados por este). Se controla
por PWM (LEDC), un canal por color, y queda con margen para cualquier otra
señal visual que haga falta más adelante (además de indicar equipo, la
prueba `pruebas-platformio/02-cuadro-color-rgb/` ya lo usa para mostrar en
vivo el color que detecta el TCS34725 delantero).

Usa los únicos 3 GPIO que quedaban libres para ampliaciones en este
documento — ver el [resumen de pines](#resumen-mapa-completo-de-pines-usados) —
así que con esto **ya no queda ningún GPIO libre** en el mapa. Los GPIO 40 y
41, que documentaban los 2 LED discretos anteriores, están libres de nuevo.

| Canal | GPIO ESP32-S3 | Nota |
|-------|:-------------:|------|
| R | **39** | |
| G | **38** | |
| B | **3** | Strapping de JTAG: solo importa su nivel en el instante de encender/resetear. Como salida PWM normal después de bootear no da problema. |

> ⚠️ **Polaridad sin confirmar con el LED físico.** Tanto `firmware-esp32/`
> como `pruebas-platformio/02-cuadro-color-rgb/` asumen **cátodo común**
> (duty PWM alto = canal más brillante). Si al probarlo los colores salen
> invertidos, es ánodo común — hay una constante (`kCommonAnode` /
> `kRgbCommonAnode`) en cada uno para cambiarlo sin tocar el resto del
> código. Actualizar esta nota una vez confirmado con el LED físico.

Cable de control: igual que el resto de señales digitales/PWM de bajo
amperaje de este robot (ENA/ENB, OUT de los QTR), usa **Azul** o **Blanco**
según el [código de colores](#código-de-colores-de-cableado) — no comparte
zona de cableado con el I2C, así que no hay riesgo de confundirlo con
SDA/SCL.

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
| 1 | QTR izquierdo (ADC) | 15 | L298N‑I IN4 |
| 2 | QTR derecho (ADC) | 16 | L298N‑I ENB |
| 3 | LED RGB — canal B | 17 | L298N‑D ENB |
| 4 | L298N‑I IN1 | 18 | LED TCS34725 delantero |
| 5 | L298N‑I IN2 | 21 | LED TCS34725 trasero |
| 6 | L298N‑I ENA | 38 | LED RGB — canal G |
| 7 | L298N‑I IN3 | 39 | LED RGB — canal R |
| 8 | I2C0 SDA | 40 | XSHUT del VL53L1X (ToF) |
| 9 | I2C0 SCL | 41 | *(libre)* |
| 10 | L298N‑D IN1 | 42 | QTR emisores (CTRL) |
| 11 | L298N‑D IN2 | 47 | I2C1 SDA (TCS34725 trasero + VL53L1X) |
| 12 | L298N‑D ENA | 48 | I2C1 SCL (TCS34725 trasero + VL53L1X) |
| 13 | L298N‑D IN3 | | |
| 14 | L298N‑D IN4 | | |

**25 pines usados.** Queda libre para ampliaciones: GPIO **41** — el otro
GPIO que quedaba libre (40) ahora lo usa el XSHUT del VL53L1X (ver la sección
de [ToF](#tof--vl53l1x-distancia-frente-al-gripper)).

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
2. **Añadir el LED RGB** (GPIO 39/38/3). Comprobar que la Raspberry Pi lo
   enciende en rojo y en azul, y de paso confirmar la polaridad (cátodo vs.
   ánodo común — ver la nota en la sección del LED RGB).
3. **Añadir el I2C**, un chip a la vez. El firmware avisa por la consola si el
   PCA9685 o algún TCS34725 no responde. **Dejar el VL53L1X para el final de
   este paso**, después de confirmar que el TCS34725 trasero ya funciona
   bien por su cuenta: así, si algo se rompe al conectar el ToF, es fácil
   saber que fue por eso.
4. **Añadir los servos** con el gripper DESACOPLADO del mecanismo, para no
   forzarlo contra un tope mientras se calibran los ángulos.
5. **Añadir los QTR.** Verificar en la telemetría que el valor sube al poner
   cinta negra debajo.
6. **Los motores al final**, con el robot en un soporte y las ruedas al aire.
