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
10. [Switch de 3 posiciones — selección de equipo](#switch-de-3-posiciones--selección-de-equipo)
11. [Enlace con la Raspberry Pi 4B](#enlace-con-la-raspberry-pi-4b)
12. [Resumen: mapa completo de pines usados](#resumen-mapa-completo-de-pines-usados)
13. [Alimentación — esquema real del equipo (una sola batería, con BEC)](#alimentación--esquema-real-del-equipo-una-sola-batería-con-bec)
14. [Orden sugerido para el montaje y las pruebas](#orden-sugerido-para-el-montaje-y-las-pruebas)

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

Ajustada al carrete real del equipo (**negro, rojo, amarillo, verde,
azul** — 5 colores, no 9; ya **no hay blanco** disponible) y a una decisión
deliberada del equipo (ver el aviso de Rojo abajo), la prioridad es, en
este orden:

1. **GND tiene color propio y exclusivo** — es la única línea de potencia
   que sigue sin compartir con nada.
2. **Rojo se comparte, a propósito, entre LOS TRES dominios de potencia
   positiva del robot** (motores 7.4–12 V crudo, lógica 3.3 V, y servos
   5–6 V regulados por el BEC) — decisión explícita del equipo, no el
   diseño por defecto de este esquema. Es la concesión más riesgosa de
   todo el documento: ver el aviso grande más abajo antes de cablear nada
   en rojo.
3. Eso deja **dos colores completos para el I2C** — Amarillo y Verde,
   uno para cada línea (SDA/SCL), manteniendo la regla de "nunca el mismo
   color en las dos líneas que corren pegadas al mismo conector" a pesar
   de que Rojo se comió tres dominios de golpe.
4. El resto de señales digitales/PWM de bajo amperaje (control de motores,
   salidas de los QTR, LED de equipo, LED de iluminación de cada TCS34725)
   reutiliza **Azul** — no pasa nada porque no conviven en el mismo
   conector que el I2C: están en otra zona del chasis.

| Color | Uso |
|-------|-----|
| **Negro** | GND — todas las masas, sin excepción |
| **Rojo** | 🛑 **Compartido a propósito entre TRES dominios** — potencia de motores (batería 7.4–12 V cruda hacia los L298N), lógica 3.3 V (VIN de los QTRX y los TCS34725) **y** servos (salida 5–6 V del BEC hacia el V+ del PCA9685). Ver el aviso grande abajo: es el color más peligroso de todo el documento. |
| **Amarillo** | **I2C — SDA, en ambos buses.** También: CTRL de los QTR (control de los emisores IR) — no se mezclan porque los QTR están en otra zona del chasis, lejos del I2C. |
| **Verde** | **I2C — SCL, en ambos buses.** Exclusivo: al quedar liberado de la potencia de servos (ahora en Rojo), se dedicó por completo a esto. |
| **Azul** | PWM/salidas analógicas de motores y QTR (ENA/ENB, OUT), señales digitales de control (IN1–IN4 de los L298N), el LED de iluminación de cada TCS34725, y los 3 canales del LED RGB de equipo — no se mezclan con el I2C porque están en otra zona del chasis. |

> 🛑 **AVISO — Rojo lleva 7.4–12 V, 5–6 V Y 3.3 V a la vez: es el color
> donde más fácil se quema algo.** El resto de este documento evita por
> completo mezclar dominios de voltaje bajo un mismo color — es la regla
> nº 1 de todo el esquema — pero el equipo decidió concentrar los TRES
> dominios positivos en Rojo, dejando el resto del carrete libre para
> masa e I2C/señales. **Un solo cruce entre dos tramos rojos de dominios
> distintos mete el voltaje equivocado en un pin que no lo tolera** — de
> los tres cruces posibles, el más destructivo es 12 V motor → 3.3 V
> lógica (quema el TCS34725/QTR al instante), pero 12 V motor → 5–6 V
> servo también daña casi cualquier servo estándar. Mitigación
> obligatoria, no opcional:
> - **Marca la punta de TODO cable rojo, sin excepción, con cuál de los
>   tres dominios lleva**: "MOTOR" (12 V crudo), "3V3" (lógica QTR/TCS) o
>   "SERVO" (5–6 V del BEC). Tres etiquetas, nunca dos cables rojos sin
>   marcar cerca uno del otro.
> - Mantén los tres tramos en **zonas físicamente separadas** del chasis
>   (motores de un lado, sensores de otro, servos/gripper del tercero) —
>   no los agrupes ni los sujetes juntos con la misma cincha.
> - Antes de energizar por primera vez, **verifica con multímetro** cada
>   cable rojo en su destino final: 3.3 V en QTR/TCS34725, 5–6 V en el V+
>   del PCA9685, con la batería conectada y los motores en reposo.

Dos reglas simples que evitan la mayoría de los sustos:

- **Rojo es la única línea que se comparte entre dominios de potencia, y
  solo por decisión explícita del equipo — todas las demás (Negro,
  Amarillo, Verde) siguen siendo exclusivas.** Aunque el LED RGB de equipo
  pueda encender en rojo, sus 3 cables de señal van en **azul** (son PWM
  de bajo amperaje, no una línea de potencia) — así se evita sumar una
  cuarta cosa al color que ya es el más delicado del esquema.
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

### Sensor DELANTERO — bus I2C nº 0 (compartido con el PCA9685 y el VL53L1X)

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
> si lo dejas sin conectar, los LED quedan encendidos siempre.

**Solo el sensor DELANTERO controla su LED por GPIO.** El del TRASERO se
cablea **directo a 3.3V** (no a un GPIO): en la práctica, todo el código del
repo lo pone en `HIGH` una sola vez al arrancar y nunca lo vuelve a tocar
(ni para apagarlo ni para evitar que el LED delantero le meta luz al
trasero — esa mitigación nunca se implementó), así que cablearlo fijo
reproduce EXACTAMENTE el mismo comportamiento y libera GPIO 21 para el
[switch de selección de equipo](#switch-de-3-posiciones--selección-de-equipo).
Si algún día se implementa esa mitigación, hace falta volver a pasar este
LED por un GPIO (cualquiera libre en ese momento).

| Señal | GPIO ESP32-S3 / Conexión |
|-------|:------------------------:|
| LED sensor delantero | **18** |
| LED sensor trasero | **3.3V directo** (ya no es un GPIO) |

El LED delantero va en cable **azul**, igual que el resto de señales
digitales de control de este robot (ver el
[código de colores](#código-de-colores-de-cableado)). En este tramo, cada
cable ya tiene un color distinto (VIN rojo, GND negro, SDA amarillo, SCL
verde, LED azul), así que no hace falta marquilla adicional — no hay dos
cables del mismo color conviviendo en el mismo conector.

El LED trasero, al ir directo a 3.3V, se cablea en **rojo** (como cualquier
otra alimentación de 3.3V de este robot — ver el
[código de colores](#código-de-colores-de-cableado)) y no en azul: ya no es
una señal de control, es una línea de potencia como el VIN del propio
sensor. Márcalo igual que el resto de tramos rojos ("3V3") por la misma
razón que el resto del esquema: Rojo convive con otros dos dominios de
voltaje en este robot.

---

## ToF — VL53L1X (distancia frente al gripper)

Mide la distancia a lo que tenga delante del gripper (la bandera) para que la
Raspberry Pi sepa cuándo cerrar la pinza — la lógica de "cuándo" vive en
`raspberry-pi/src/athena/decision.py`, el ESP32 solo mide y reporta.

**Dirección I2C fija de fábrica: 0x29 — igual que AMBOS TCS34725.** No hay
forma de elegir otra dirección desde el pin ni por strapping. Por eso NO va en
un bus propio: comparte el bus I2C nº0 con el PCA9685 y el TCS34725 delantero
(el PCA9685 no da problema, es 0x40), y su pin **XSHUT** es imprescindible
(no opcional) para poder arrancar sin que las dos direcciones 0x29 choquen.

| Pin VL53L1X | GPIO ESP32-S3 / Conexión | Nota |
|-------------|:------------------------:|------|
| SDA | **8** | Bus I2C nº 0 — compartido con el PCA9685 y el TCS34725 delantero |
| SCL | **9** | Bus I2C nº 0 — compartido con el PCA9685 y el TCS34725 delantero |
| XSHUT | **3** | Reset por software. Ver la secuencia de arranque abajo, y la nota de JTAG más abajo |
| VIN | 3.3 V | |
| GND | GND común | |

> **Por qué GPIO 3 y no otro:** es el único pin que queda físicamente libre
> junto al bus I2C0 en el header J1 del DevKitC-1 (justo debajo de GPIO8), así
> que el cable de XSHUT queda corto, al lado de SDA/SCL, en vez de cruzar la
> placa hasta el otro header. A cambio, GPIO 3 es **strapping de JTAG** — ver
> la tabla de [pines prohibidos](#pines-prohibidos-del-esp32-s3): "se puede
> usar, pero mejor no". Su nivel solo importa en el instante de
> arranque/reset, antes de que corra una sola línea de `setup()`, así que
> manejarlo como salida normal después no rompe el arranque. Lo único a
> vigilar: si la placa del VL53L1X trae su propio pull-up en XSHUT (ver el
> punto 1 de la secuencia abajo), ese pull-up puede dejar JTAG en un estado
> distinto al esperado — no impide arrancar (JTAG no es un modo de boot como
> GPIO 0/45/46), pero puede sorprender si esperabas depurar por JTAG y no
> responde. El canal B del LED RGB, que antes vivía en este pin, se movió a
> GPIO 41 (ver la sección del [LED RGB](#led-rgb-indicador-de-equipo)).

### ⚠️ Por qué hace falta XSHUT, y el riesgo que queda sin resolver

El VL53L1X arranca siempre en 0x29. El TCS34725 delantero, en el mismo bus,
está fijo en esa misma dirección **y no tiene ningún pin de apagado**: en
cuanto tiene alimentación, responde en 0x29 sin que el firmware pueda
callarlo. La secuencia de arranque (implementada en `TofSensorTask`,
`firmware-esp32/src/main.cpp`) es:

1. **GPIO 3 en LOW desde `setup()`**, antes de crear ninguna tarea — el
   VL53L1X queda en reset y no contesta en el bus. Esto es importante hacerlo
   ANTES de arrancar cualquier tarea: algunas placas del sensor traen un
   pull-up en XSHUT que lo deja activo apenas se energiza, así que si el
   firmware tardara en ponerlo en LOW, habría una ventana de arranque con los
   dos chips respondiendo en 0x29 a la vez.
2. GPIO 3 a HIGH: el sensor sale de reset y arranca (~1.2 ms).
3. Se le reasigna la dirección **0x30** con una única escritura corta a su
   registro de dirección. Este es el único instante en que el VL53L1X sigue
   en 0x29 mientras el TCS34725 delantero también está vivo ahí — no se puede
   evitar con el hardware actual sin agregarle un pin de apagado al TCS34725
   (por ejemplo, cortando su alimentación con un transistor). El equipo
   decidió aceptar este riesgo acotado (una sola transacción de 3 bytes, no
   la inicialización completa del sensor) en vez de sumar ese hardware extra.
4. Recién ahora corre la inicialización completa del VL53L1X, ya en 0x30 y
   sin nadie más escuchando ahí.

**Si el TCS34725 delantero empieza a dar lecturas raras justo después de un
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
| OUT izquierdo | **1** | ADC1_CH0 — cable azul |
| OUT derecho | **2** | ADC1_CH1 — cable azul |
| CTRL (ambos) | **42** | Control de los emisores IR, compartido — cable **amarillo** |
| VIN | **3.3 V** | ⚠️ **Nunca 5 V** — ver la tabla de riesgos — cable rojo |
| GND | GND | cable negro |

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

R y G usan dos de los GPIO que quedaban libres para ampliaciones en este
documento (38 y 39). El canal B vivía en el tercero (GPIO 3), pero se movió a
GPIO 41 para cederle el 3 al **XSHUT del VL53L1X** (ver la sección de
[ToF](#tof--vl53l1x-distancia-frente-al-gripper)), que sí se beneficia de
estar físicamente junto al bus I2C0. El GPIO que quedaba libre (**40**) ya no
lo está: ahora es uno de los 2 pines del
[switch de selección de equipo](#switch-de-3-posiciones--selección-de-equipo)
de más abajo — ver esa sección para por qué no quedó ningún GPIO libre.

| Canal | GPIO ESP32-S3 | Nota |
|-------|:-------------:|------|
| R | **39** | |
| G | **38** | |
| B | **41** | |

> **Polaridad: cátodo común**, confirmado con el LED físico. Duty PWM alto =
> canal más brillante, que es el sentido natural del código. Todos los
> sketches del repo ya están en ese modo (`kCommonAnode` / `kRgbCommonAnode`
> en `false`); la constante se conserva en cada uno por si algún día se
> reemplaza el LED por uno de ánodo común.

Cable de control: igual que el resto de señales digitales/PWM de bajo
amperaje de este robot (ENA/ENB, OUT de los QTR), usa **Azul** según el
[código de colores](#código-de-colores-de-cableado) — no comparte zona de
cableado con el I2C, así que no hay riesgo de confundirlo con SDA/SCL.

---

## Switch de 3 posiciones — selección de equipo

Un switch **SPDT ON-OFF-ON** (3 posiciones, el centro es un verdadero
"apagado"/reposo, no solo un punto intermedio) le dice al robot, al puro
inicio de la secuencia, si es el equipo azul o el rojo — antes esto era
`--equipo rojo/azul` por línea de comandos en la Raspberry Pi (o un puente de
un solo pin en `firmware-esp32-standalone/`), sin forma de saberlo con solo
mirar el robot apagado ni de garantizar que no arrancara movido con el
equipo equivocado.

**Solo quedaba 1 GPIO libre en todo el robot (40)** — ver el
[resumen de pines](#resumen-mapa-completo-de-pines-usados) de antes de este
cambio. El segundo pin que hacía falta salió de liberar el LED trasero del
TCS34725 (ver la sección de [LED de iluminación](#led-de-iluminación-del-propio-tcs34725)
más arriba), no de tocar ningún pin de arranque/strapping — GPIO 0 y GPIO 45/46
seguían **prohibidos** aunque técnicamente estuvieran sin usar: un switch en
la posición equivocada justo al energizar podría dejar al ESP32-S3 sin
arrancar el firmware, o (peor, porque el switch normalmente SÍ está en una
posición de equipo durante toda la ronda) meterlo en modo bootloader si un
brownout de motores lo reinicia a mitad de partida. Ver el aviso de
brownout en la sección de [alimentación](#alimentación--esquema-real-del-equipo-una-sola-batería-con-bec)
más abajo — es exactamente el escenario que se evitó no tocando esos pines.

| Terminal del switch | Conexión | Cable | Nota |
|---|---|:---:|---|
| Común (pin del medio) | **GND** | **Negro** | |
| Tiro "AZUL" | **GPIO 40** | **Azul** | Cerrado a GND = equipo azul (buscar bandera roja) |
| Tiro "ROJO" | **GPIO 21** | **Azul** | Cerrado a GND = equipo rojo (buscar bandera azul) |

> ⚠️ Esta tabla quedó **al revés** en el borrador original (GPIO 21 = AZUL,
> GPIO 40 = ROJO) — al cablear el switch de verdad en banco, los tiros
> quedaron invertidos respecto a ese borrador. El firmware (`Pins::
> TEAM_SWITCH_BLUE`/`TEAM_SWITCH_RED` en `firmware-esp32/` y
> `firmware-esp32-standalone/`) ya se corrigió para reflejar ESTA tabla, la
> real. Si vuelves a cablear el switch desde cero y por algún motivo los
> tiros no dan estos GPIO, confirmalo con `TLM_TEAM_SWITCH` (o
> `scripts/prueba_enlace.py --solo-escuchar`) antes de asumir que coincide
> con lo escrito acá.

Los 2 GPIO se leen con **pull-up interno** (`INPUT_PULLUP`, sin resistencias
externas): el ESP32-S3 los sostiene en 3.3 V por su cuenta, y el switch
solo los lleva a GND al cerrar hacia ese lado — tiro abierto = HIGH, tiro
cerrado = LOW. No hace falta alimentar el switch desde 3.3 V directamente
ni poner resistencias externas.

> Los dos tiros van en **Azul** a propósito, no en "Azul para uno y Rojo
> para el otro" (que sería lo intuitivo por el nombre del equipo): en este
> esquema **Rojo es exclusivamente para potencia** (motores + 3.3 V lógica
> + servos — ver el [código de colores](#código-de-colores-de-cableado)) y
> nunca debe llegar a un GPIO. Ambos tiros del switch son señales lógicas
> de bajo amperaje, igual que el resto de lo que ya va en Azul (IN1–IN4,
> canales del LED RGB, etc.) — el nombre del tiro ("AZUL"/"ROJO") describe
> el equipo, no el color físico del cable.

| Posición física | GPIO 40 (AZUL) | GPIO 21 (ROJO) | Significado |
|:---:|:---:|:---:|---|
| **0** (centro) | HIGH | HIGH | Nadie ha elegido equipo. El robot no se mueve. |
| **1** | LOW | HIGH | Equipo **AZUL** — misión: buscar la bandera **roja** |
| **2** | HIGH | LOW | Equipo **ROJO** — misión: buscar la bandera **azul** |

> ⚠️ **Procedimiento de encendido, en este orden — no al revés:** primero el
> switch de 3 posiciones en **0** (centro), y RECIÉN DESPUÉS el switch de la
> batería. El robot energiza todo (motores, sensores, Raspberry Pi) pero se
> queda quieto: el firmware fuerza los motores a detenidos mientras el
> switch de equipo siga en 0 (`firmware-esp32/`, ver `g_switchTeam` en
> `MotorTask`; `firmware-esp32-standalone/` directamente espera en `setup()`
> antes de arrancar ninguna tarea). Recién cuando alguien mueve el switch a
> 1 o 2 el robot sabe qué equipo es y puede empezar a moverse. Esto es
> justo lo que resuelve la posición central de verdad que no tenía el viejo
> selector de un solo pin.

**Etiqueta físicamente las 3 posiciones en el chasis** ("0", "AZUL", "ROJO")
junto al switch — con 3 colores de cable distintos llegando a un conector
tan pequeño, confiar en la memoria de cuál tiro es cuál es la forma más
fácil de anunciarse como el equipo equivocado frente a los jueces.

Cable de control: **Azul**, igual que el resto de señales digitales de bajo
amperaje de este robot (ver el
[código de colores](#código-de-colores-de-cableado)) — el común a GND va en
**Negro**, como toda masa.

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
| 3 | XSHUT del VL53L1X (ToF) | 17 | L298N‑D ENB |
| 4 | L298N‑I IN1 | 18 | LED TCS34725 delantero |
| 5 | L298N‑I IN2 | 21 | Switch equipo — tiro ROJO |
| 6 | L298N‑I ENA | 38 | LED RGB — canal G |
| 7 | L298N‑I IN3 | 39 | LED RGB — canal R |
| 8 | I2C0 SDA (PCA9685 + TCS34725 delantero + VL53L1X) | 40 | Switch equipo — tiro AZUL |
| 9 | I2C0 SCL (PCA9685 + TCS34725 delantero + VL53L1X) | 41 | LED RGB — canal B |
| 10 | L298N‑D IN1 | 42 | QTR emisores (CTRL) |
| 11 | L298N‑D IN2 | 47 | I2C1 SDA (TCS34725 trasero) |
| 12 | L298N‑D ENA | 48 | I2C1 SCL (TCS34725 trasero) |
| 13 | L298N‑D IN3 | | |
| 14 | L298N‑D IN4 | | |

**26 pines usados. No queda ningún GPIO libre para ampliaciones.** El XSHUT
del VL53L1X (ver [ToF](#tof--vl53l1x-distancia-frente-al-gripper)) se movió
al GPIO 3, físicamente junto al I2C0 — eso le cedió el 3 al canal B del LED
RGB (ver [LED RGB](#led-rgb-indicador-de-equipo)), que ahora vive en el GPIO
41. El último GPIO libre (40) y el que liberó el LED trasero del TCS34725
(21) se usaron para el
[switch de selección de equipo](#switch-de-3-posiciones--selección-de-equipo) —
GPIO 0 y GPIO 45/46, aunque también estaban técnicamente libres, se
descartaron por ser pines de strapping de arranque (ver la sección del
switch para el porqué).

---

## Alimentación — esquema real del equipo (una sola batería, con BEC)

A diferencia de lo que recomendaba antes este documento (dos baterías
separadas), el equipo alimenta motores y servos/lógica desde **la misma
batería física** — un BEC/regulador reduce esa misma fuente a 5–6 V para la
rama de los servos. Eléctricamente siguen siendo dos rieles distintos (el
PCA9685 y la Raspberry Pi ven 5–6 V regulados, no el crudo de 7.4–12 V),
pero en el [código de colores](#código-de-colores-de-cableado) **los dos
rieles ahora comparten Rojo** — decisión del equipo, no una consecuencia
automática de compartir batería — así que la distinción entre ellos vive
solo en la marquilla de cada punta ("MOTOR" vs "SERVO"), no en el color.

```
Batería única (7.4–12 V)
   ├──> L298N nº1  (12V crudo)
   ├──> L298N nº2  (12V crudo)
   └──> BEC 5–6 V
           ├──> PCA9685 V+  (servos)
           └──> Raspberry Pi 4B (5 V, 3 A)

ESP32-S3
   └──> alimentado por el cable USB de la Raspberry Pi

TODAS LAS MASAS UNIDAS EN UN SOLO PUNTO
```

> ⚠️ **Esto reintroduce justo el riesgo que la separación evitaba: un pico
> de corriente de los motores puede hundir la tensión de la batería lo
> bastante como para que el BEC ya no tenga margen para sostener 5–6 V a la
> salida** (todo BEC necesita cierta diferencia mínima entre su entrada y
> su salida — *dropout* — para regular bien). Si eso pasa, la caída se
> propaga: Raspberry Pi → USB → ESP32-S3, y el ESP32 se reinicia solo
> (brownout) justo cuando el robot arranca a moverse — el mismo síntoma que
> ya se vio en banco con el firmware autónomo (reinicios y errores de I2C
> intermitentes sin relación aparente con los sensores).
>
> Mitigaciones, de más a menos efectiva:
> 1. **Usar un BEC con margen de corriente de sobra** (al menos el doble de
>    lo que piden servos + Raspberry Pi juntos en el peor caso) y **bajo
>    dropout**, para que aguante los picos de los motores sin que la salida
>    se hunda.
> 2. **Agregar un capacitor electrolítico grande (1000–2200 µF o más) justo
>    a la salida del BEC**, cerca del PCA9685 y de la Raspberry Pi — absorbe
>    los picos rápidos de corriente sin esperar a que el BEC reaccione.
> 3. Si los reinicios persisten en pruebas reales del robot en movimiento
>    (no solo en banco, con el robot quieto), **volver a la batería separada
>    que recomendaba este documento** es la solución definitiva: elimina el
>    acoplamiento por completo en vez de mitigarlo.

---

## Orden sugerido para el montaje y las pruebas

1. **Solo el ESP32** por USB. Cargar el firmware, ver el mensaje de arranque en el
   puerto de depuración a 115200.
2. **Añadir el LED RGB** (GPIO 39/38/41). Comprobar que la Raspberry Pi lo
   enciende en rojo y en azul. La polaridad ya está confirmada (cátodo
   común), así que no hay nada que ajustar acá.
3. **Añadir el I2C**, un chip a la vez. El firmware avisa por la consola si el
   PCA9685 o algún TCS34725 no responde. **Dejar el VL53L1X para el final de
   este paso**, después de confirmar que el TCS34725 delantero ya funciona
   bien por su cuenta: así, si algo se rompe al conectar el ToF, es fácil
   saber que fue por eso.
4. **Añadir los servos** con el gripper DESACOPLADO del mecanismo, para no
   forzarlo contra un tope mientras se calibran los ángulos.
5. **Añadir los QTR.** Verificar en la telemetría que el valor sube al poner
   cinta negra debajo.
6. **Añadir el switch de equipo** (GPIO 21/40, ver la
   [sección del switch](#switch-de-3-posiciones--selección-de-equipo)).
   Con el robot en el soporte, mover el switch a cada posición y confirmar
   con la telemetría `TLM_TEAM_SWITCH` (o el log de consola, en
   `firmware-esp32-standalone/`) que reporta 0/1/2 correctamente antes de
   pasar al siguiente paso — es más fácil depurarlo con las ruedas quietas.
7. **Los motores al final**, con el robot en un soporte y las ruedas al aire.
   Confirmar primero que, con el switch de equipo en 0, los motores NO
   responden a ningún comando — es la comprobación del segundo failsafe
   antes de dejar que el robot pueda moverse de verdad.
