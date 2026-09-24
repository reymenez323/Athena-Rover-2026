# Lógica de Athena — Athena Rover 2026

Versión 1 (2026-09-24). Reorganiza [`logicaATHENA.pdf`](logicaATHENA.pdf) sin quitarle nada y le suma las decisiones tomadas después.
Etiquetas: **[DEFINIDO]** decidido por Montse · **[POR DEFINIR]** falta decidir · **[PROPUESTA]** sugerencia sin aprobar.

**Flujo:** encender → elegir equipo → agarrar caja → ir al amarillo → soltar caja → evadir → buscar y centrar la bandera → agarrarla → salir de la zona rival → volver → soltar.

---

## 1. Alcance de la versión 1

- Toda la secuencia, de punta a punta, en su forma más simple. Sin rivales ni obstáculos.
- El robot arranca en el medio de su zona, con la caja en el piso, al frente.
- Las mejoras se hacen cuando todo pase al 100 % (ver sección 9).

## 2. Reglas globales

1. **Nunca salirse de la pista.** Con 2 ruedas fuera se pierde la ronda. Vale en todos los pasos.
2. **Depositar la llave antes de buscar la bandera.** Buscarla antes pierde la ronda.
3. **Nunca quedarse inactivo.** Si algo no se ve, pausa breve y sigue buscando hasta soltar la bandera.
4. **Cámara y ToF son obligatorios** para agarrar la bandera.
5. **La caja y la bandera no se botan ni se arrastran.**
6. Movimiento suave y continuo; nada de zigzag ni giros a lo loco.
7. Máximo 10 minutos por ronda.
8. Los tiempos están calibrados a **7.60 V** en los motores. Si el voltaje cambia, se recalibran.
9. Todo parámetro ajustable va **al inicio del código, con una explicación breve** (ver sección 7).
10. **La bandera solo se agarra parada (vertical).** La cámara la reconoce en cualquier posición, pero **de momento el código no verifica si está caída**. Si se cae, alguien la vuelve a parar y la deja a la vista de la cámara; el robot no tiene mecanismo para una bandera acostada.

## 3. Equipos y glosario

| Posición del switch | Estado | Zona propia | Busca |
|---|---|---|---|
| 0 (centro) | Espera. Motores quietos, LED parpadea | — | — |
| 1 (AZUL, GPIO 40) | Equipo azul | Zona azul | Bandera roja |
| 2 (ROJO, GPIO 21) | Equipo rojo | Zona roja | Bandera azul |

- **Zona propia:** la del equipo elegido; ahí arranca el robot y ahí deposita la bandera.
- **Zona rival:** la del color contrario; ahí está la bandera que hay que capturar.
- **Zona neutra:** cuadro amarillo del centro; ahí se deposita la llave (caja).
- **Franja:** cinta de color en el borde de cada zona (~18.5 mm, cinta eléctrica).
- **Llave = caja:** cubo de ~20 mm. **Bandera:** cilindro Ø5 × 15 cm.
- Encendido: primero el switch de equipo en 0 y después el de la batería.

## 4. La pista

Planos: [pista acotada](pista_robotica_plano_acotado_A3_escala_1_5.pdf) y [la versión rayada con los recorridos](pista_robotica_plano_acotado_A3_escala_1_5_RAYADA.pdf).

- Pista de **83.5 × 170 cm** con marco negro. Zona azul abajo, zona roja arriba, cada una de 28 cm (incluye marco y franja).
- Cuadro amarillo de **27 × 29 cm** al centro. De cada franja de color al borde del amarillo hay ~43 cm (medido en el plano).
- Franjas reales de ~18.5 mm (el plano dice 20 mm).
- El dibujo está desde el equipo azul. El equipo rojo hace lo mismo girado 180°.

**Recorridos dibujados (equipo azul):**
- **Naranja:** salida recta desde la zona propia hasta el amarillo. La X es el punto de depósito, unos 6 cm dentro del borde cercano.
- **Magenta (ideal, preferido):** retrocede un poco, gira a la derecha, avanza en paralelo al cuadro y sube hacia la zona rival.
- **Verde (alternativa):** sale directo hacia la derecha cruzando el borde amarillo y luego sube.
- **Azul (lo que hace el robot de verdad):** al girar por pivote también se desplaza hacia atrás, así que termina más atrás y a la izquierda de la ruta magenta. Luego avanza en diagonal para librar la esquina del cuadro y gira a la izquierda hasta quedar mirando a la zona rival, a la derecha del cuadro.
- **Cuadrados celestes:** el mismo programa no deja siempre al robot en el mismo punto ni con el mismo ángulo (agarre de las ruedas e irregularidades de la pista). **Los pasos siguientes no pueden asumir una posición o ángulo exactos.**

## 5. Los 12 pasos

### Fase A — Arranque

**1. Encendido** *(PDF fila 1)*
- Se energiza la batería y todo el robot. El robot está en el medio de su zona.

**2. Elegir equipo con el switch** *(fila 2)*
- Con el switch en 0 el robot espera. Al pasar a 1 o 2 sabe su equipo y su color objetivo. La Pi recibe el mismo equipo del ESP32.
- Después hay una cuenta regresiva antes del siguiente paso. Parámetro: `retardo inicial` (hoy 3 s).

**3. Agarrar la caja** *(fila 3)*
- La caja ya está en el piso al frente. El primer movimiento es cerrar el gripper sobre ella (128°).
- Espera a que el servo llegue (400 ms) y un margen extra (600 ms) antes de seguir.
- Nota: el gripper perdió algo de fuerza. Hay que recalibrar el ángulo para asegurar caja y bandera.

### Fase B — Zona neutra

**4. Buscar la zona amarilla** *(fila 4)*
- Avanza en **línea recta** desde el medio de su zona hasta que el sensor de color delantero lee amarillo. Velocidad de crucero (hoy 60 %).
- Sensores: color delantero. La cámara no interviene.
- Zigzag solo si la recta no funciona **[POR DEFINIR]**. Se mejora cuando todo pase al 100 %.

**5. Depositar la llave** *(fila 5)*
- Al leer amarillo: para por completo (400 ms), abre el gripper (0°) y espera (400 ms).
- Reglas: la caja queda 100 % dentro del amarillo, sin rebotar ni caer afuera, ni una esquina. El robot no la mueve después.
- Parámetros: punto de parada, movimiento y fuerza del gripper al soltar.

**6. Evadir la zona amarilla** *(fila 6)*
- No puede seguir recto: se llevaría la caja arrastrada.
- Secuencia: retroceso → giro 1 a la derecha (desvía el frente del amarillo) → avance corto (libra la esquina) → giro 2 a la izquierda hasta mirar a la zona rival.
- Valores actuales (v6/v7, en ms, no en grados): retroceso 900 ms al 70 %; giro 1 de 2000 ms al 100 %; avance 900 ms al 70 %; giro 2 de 2800 ms.
- La cámara puede mirar desde que empieza la evasión, pero **no manda movimiento** hasta que el robot salió de la zona amarilla. En el giro 2, si la cámara confirma la bandera de forma sostenida, corta el giro; si no, termina por tiempo.
- Ver la sección 4: la posición y el ángulo final varían entre corridas.

### Fase C — La bandera

**7. Localizar y acercarse** *(fila 7)*
- La cámara busca la bandera del color rival. Al verla, se centra de forma **proporcional** (línea central, zona muerta de ±0.15) y avanza.
- Si no la ve: pausa breve y sigue buscando; nunca inactivo. Patrón de búsqueda **[POR DEFINIR]** (por ejemplo, giro lento). Siempre dentro de la pista.
- Parámetros: velocidad, confianza mínima de detección (hoy 0.6), ganancia del centrado, cuándo está lo bastante cerca para terminar de centrarse sin botar la bandera.

**8. Terminar de posicionarse** *(fila 8)*
- Centrado (cámara) y distancia (ToF) hasta el rango de agarre: **56–62 mm** por ahora. La bandera debe estar parada (regla 10).
- Se avanza en pasos cortos: para, espera, mide y confirma **3 lecturas seguidas** en rango. Solo se usa el ToF si la bandera está centrada.
- Parámetros actuales: paso al 60 % durante 150 ms, asentamiento 200 ms, máximo 40 pasos.
- **[POR DEFINIR]** En v6/v7, tras 40 pasos cierra igual. Con la regla "cámara y ToF obligatorios" hay que decidir qué hace si nunca se confirma.

**9. Cerrar el gripper** *(fila 9)*
- Con el robot detenido, cierra sobre la bandera (65°). Requiere cámara y ToF confirmados. El gripper solo puede agarrarla parada (regla 10).
- Parámetros: velocidad y fuerza de cierre.
- **[POR DEFINIR]** Cómo comprobar que la bandera quedó agarrada.

### Fase D — Retorno

**10. Salir de la zona rival** *(fila 10)*
- Con la bandera asegurada, da la vuelta y sale sin salirse de la pista.
- Sin sensor trasero, la salida se confirma cuando el sensor delantero **vuelve a leer la franja rival** al salir. Es la única confirmación por ahora.
- **[POR DEFINIR]** ¿Cabe el giro de 180° dentro de la zona (28 cm contando marco y franja)? ¿Qué hace si no lee la franja (18.5 mm, color a 10 Hz)?

**11. Volver a la zona propia** *(fila 11)*
- Movimiento suave y continuo, con la bandera siempre asegurada. Al principio, en línea recta hasta que el sensor delantero lee **su propio color**.
- Puede volver como quiera mientras no se salga de la pista. Se ignora al rival por ahora.

**12. Soltar la bandera** *(fila 12)*
- Al leer su franja: para por completo y suelta la bandera despacio, a velocidad lenta.
- Parámetros: velocidad y fuerza del gripper, y cuánto avanza dentro de la zona antes de soltar **[POR DEFINIR]**.
- Mejora futura: entrar completo a la zona (como estacionarse en paralelo) y confirmar antes de soltar.

## 6. Quién decide qué

| Sensor / actuador | Se usa para | Notas |
|---|---|---|
| Color delantero (TCS34725) | Amarillo (4–5), franja rival (10), franja propia (11–12) | Es el único de color: el trasero está desconectado. Franjas de 18.5 mm. |
| Cámara (Raspberry Pi) | Ver la bandera rival en cualquier posición y en qué lado de la imagen está, para centrarse (6–8) | Obligatoria para agarrar. Modelo de Edge Impulse, con detector de color y forma de respaldo. No se verifica si la bandera está parada o caída (ver regla 10). |
| ToF (VL53L1X) | Distancia a la bandera (8–9) | No puede ver la caja (está montado muy alto). No distingue bandera de obstáculo: por eso se exige la cámara. |
| Reflectancia (QTR) | Borde negro de la pista (todos los pasos) | Todavía sin integrar. El izquierdo está desactivado en el código; hay que verificar su sensibilidad. |
| Gripper (servo) | Caja 128°, bandera 65°, abierto 0° | Recalibrar la fuerza. |
| Switch y LED RGB | Elegir equipo y mostrar el estado | |

## 7. Parámetros a configurar (valores actuales de v6/v7; verificar en el código)

| Parámetro | Paso | Qué hace | Valor |
|---|---|---|---|
| retardo inicial | 2 | Espera tras elegir equipo | 3000 ms |
| espera del servo (caja) | 3 | Tiempo para que el gripper llegue | 400 ms |
| margen tras asegurar caja | 3 | Espera extra antes de moverse | 600 ms |
| velocidad de crucero | 4 | Avance recto | 60 % |
| parada antes de soltar caja | 5 | Full stop antes de abrir | 400 ms |
| retroceso tras la caja | 6 | Duración / velocidad | 900 ms / 70 % |
| giro 1 (esquive) | 6 | Duración / velocidad, a la derecha | 2000 ms / 100 % |
| avance tras esquive | 6 | Duración / velocidad | 900 ms / 70 % |
| giro 2 (recentrado) | 6 | Duración máxima, a la izquierda | 2800 ms |
| señal de cámara fresca / sostenida | 6 | Cuánto vale una detección / cuánto debe durar | 300 ms / 150 ms |
| zona muerta del centrado | 7 | Error horizontal ignorado | ±0.15 |
| confianza mínima | 7 | Umbral de detección del modelo | 0.6 |
| rango de agarre del ToF | 8 | Distancia para cerrar | 56–62 mm |
| paso fino | 8 | Velocidad / duración | 60 % / 150 ms |
| asentamiento tras parar | 8 | Espera antes de medir | 200 ms |
| lecturas seguidas requeridas | 8 | Cuántas en rango para cerrar | 3 |
| máximo de pasos | 8 | Tope de correcciones | 40 |
| ángulos del gripper | 3, 5, 9, 12 | Abierto / caja / bandera | 0° / 128° / 65° |

## 8. Pendientes y decisiones

**Por definir**
- Recta o zigzag para buscar el amarillo (4).
- Cómo cortar la evasión cuando la cámara ya mira (6).
- Patrón de búsqueda cuando la cámara no ve la bandera (7).
- Qué hace tras el máximo de pasos si la cámara no confirmó (8).
- Cómo comprobar que la bandera quedó agarrada (9).
- Cabe el giro de 180° en la zona rival, y qué hace si no lee la franja (10).
- Cuánto avanza en su zona antes de soltar (12).

**Ya decidido (2026-09-24)**
- Zona propia = la del equipo elegido; la bandera se deposita ahí.
- La caja empieza en el piso, al frente ("desde el aire" descartado).
- Cámara y ToF obligatorios; sin verla, pausa breve y sigue buscando.
- Retorno libre sin salirse de la pista; el rival se ignora en v1.
- Evasión: magenta preferido, verde alternativo.
- Rango del ToF 56–62 mm por ahora; franjas de 18.5 mm.
- Los 6 s de espera de v6 eran solo una prueba.
- Sensor de color trasero fuera hasta después de la competencia.
- Bandera caída: no se implementa nada por ahora; en la competencia alguien la vuelve a parar a la vista de la cámara.
- Arquitectura: el ESP32 lleva la máquina de estados y todos los comandos de motor; la Pi solo manda por serial lo que ve (detección y error de centrado). Se valida primero en un standalone nuevo y, ya depurado, se promueve a `firmware-esp32/` y `raspberry-pi/`.

## 9. Fase 2 (después de que la v1 pase al 100 %)

- Evadir al rival sin salirse de la pista, sin chocar con las llaves y sin perder la bandera.
- IMU para giros y rumbo.
- Sensor de color trasero con multiplexor TCA9548A.
- Entrar completo a la zona propia (estacionamiento en paralelo) y confirmar.
- Zigzag en la búsqueda del amarillo.
- Afinar la lectura de negro contra gris (color y QTR).
- Pantalla LCD para ver la cámara.
- Detectar si la bandera está parada o caída (el detector de color y forma ya distingue las proporciones) y decidir qué hacer si está caída.

---

## Apéndice A — PDF original, sin editar (transcrito a mano)

Original: [`logicaATHENA.pdf`](logicaATHENA.pdf). Esta transcripción es a mano; ante cualquier diferencia, manda el PDF.

Acción | Aspectos a considerar | Parámetros a configurar | Otro/Observaciones

1. **Encendido del robot** | Se enciende la batería y energiza todo el robot | N/A |
2. **Integrante del equipo mueve el switch de 3 posiciones.** | depende de la posición se sabe qué equipo somos/es athena | Posición 0 - apagado/idle state/esperando instrucción. Posición 1 - somos equipo azul (y por tanto debo ir a buscar bandera roja). Posición 2 - somos equipo rojo (y por tanto debo ir a buscar bandera azul) | Una vez seleccionado el equipo, el robot debe tener un delay (por definir) antes de pasar a la siguiente acción
3. **Athena debe agarrar la caja** | El equipo debe de terminar de decidir si la caja se le va a dar desde el aire (es decir: que el gripper se vaya cerrando lentamente y un integrante del equipo le posicione la caja hasta que el gripper la asegure del todo), o si se le va a poner la caja en el piso en una posición específica para que el gripper simplemente cierre y la asegure, o ambas opciones. | Si la caja se le dará al gripper "desde el aire", en el piso o cualquiera de las dos opciones | Una vez agarrada la caja, debe de haber un pequeño delay antes de que siga a la otra acción.
4. **Athena debe empezar a moverse y buscar la zona de depósito de la llave (zona amarilla) - Encontrar zona amarilla** | El robot no debe moverse a lo loco. Hay que definir si se va a mover inicialmente solo en línea recta o con un poco de zigzag (para la búsqueda). | Movimiento del robot. Velocidad en la que se mueve | Inicialmente comenzar con un código simple que se mueva en línea recta hasta encontrar la zona amarilla (el robot se debe poner en el medio de la pista para ello), una vez pase todas las pruebas al 100% se puede mejorar el código
5. **Depositar llave ADENTRO de la zona amarilla** | La caja no puede rebotar ni caer afuera (100% de su masa dentro de la zona). El robot debe parar cuando detecte la zona y soltar la llave en la zona. El robot no puede mover la caja luego de depositado | Posición en donde se detuvo. Fuerza y movimiento del gripper para soltar la caja | La caja no puede rebotar ni caer afuera, ni una esquina.
6. **Realizar maniobra de evasión para salir de la zona amarilla** | El robot no puede seguir recto inmediatamente deposita la llave porque se la lleva arrastrada. Salir de zona amarilla (puede ser que tenga que retroceder un poco y doble hacia la derecha para evitar la zona amarilla y luego doble hacia la izquierda para volver a posicionarse de frente a la zona del contrincante) | Movimiento del robot. Velocidad de desplazamiento. Correcta detección de zonas y bordes | En esta parte, la cámara ya debería de empezar a buscar la bandera pero aún no le manda señales de movimiento a los motores hasta que se haya salido de la zona amarilla (por definir//desarrollar mejor)
7. **Localizar y desplazarse hasta la bandera del contrincante** | El robot siempre manteniéndose dentro de la pista, debe de encontrar la bandera del contrincante y empezar a moverse hacia ella | Velocidad. % de confiabilidad de reconocimiento. Programa de centrado | Definir cuándo esté lo suficientemente cerca de la bandera para que se termine de centrar (sin botar la bandera) para pasar a la siguiente acción
8. **Terminar de posicionarse (centrado y distancia) para agarrar la bandera** | El robot debe estar centrado y a una distancia definida que sea la ideal para agarrar la bandera. No puede botar en ningún momento la bandera. | Programa de centrado. Velocidad | Comunicación entre tof / cámara y sensores. Terminar de definir la distancia adecuada para agarrar la bandera
9. **Cerrar el gripper (agarrar bandera)** | La bandera debe estar asegurada | Velocidad de cierre del gripper. Fuerza del cierre del gripper | El robot tiene forma de asegurar que la bandera está agarrada?
10. **Salir de zona del contrincante** | El robot debe salir de la zona del contrincante con la bandera asegurada "en mano" (en el gripper), no se puede salir en ningún momento de la pista. Correcta detección de colores y borde | Velocidad |
11. **Buscar y trasladarse hacia zona de equipo** | Movimiento suave y continuo. La bandera siempre debe ir asegurada, no se puede caer. | Velocidad | Debe de moverse (inicialmente en línea recta) hasta que detecte el color del equipo.
12. **Soltar la bandera de forma segura en el espacio del equipo** | Cuando el sensor de color detecte el color del equipo nuevamente, el robot debe detenerse por completo y soltar suavemente y a una velocidad lenta la bandera en la zona | Velocidad y fuerza del gripper |

**Nota lateral (columna derecha, junto a las filas 10-11):** "En algún lugar de acá tenemos que pensar tmb en la lógica de ambos sensores de colores... tipo que pasa cuando el delantero lee un color y unos ms después el trasero lee otro color, y así... hay que terminar de definirlo, en la zona amarilla por ejemplo, solo el delantero debería leer amarillo en todo momento, pero cuando estemos en la zona del contrincante muy probablemente el robot tenga que entrar completamente y ahí ambos sensores deben leer el mismo color (en diferentes momentos). Y por ejemplo para cuando el robot se debe devolver a nuestra zona de equipo, si está completamente en la zona del contrincante tanto el sensor delantero como el trasero deben leer el color del contrincante para saber que ya salieron de esa zona (el trasero lo confirma), E inicialmente podemos poner en el sensor delantero que cuando vuelva a detectar nuestro color de equipo deposite la llave pero después si todo sale bien, podemos hacer que el robot entre por completo en nuestra zona (posiblemente tipo parqueo en paralelo porque no cabe) y una vez confirmado que está dentro pues suelta la llave (pero tiene que ser dentro de la pista)".

**Notas inferiores:** "Primero vamos a hacer los códigos con la lógica simple y sin posibles obstáculos enemigos y demás posibles retos, vamos a hacer una prueba inicial con toda la lógica y secuencia pero de la forma más sencilla. Toma en cuenta que pondremos el robot para que inicie en el medio de nuestra zona." · "Para después: pensar en programa para evadir contrincante sin salirse de la pista, no chocar con las llaves y sin perder la bandera".

## Apéndice B — Dónde quedó cada parte del PDF

| PDF | Está en |
|---|---|
| Filas 1–12 | Pasos 1–12 de la sección 5 (cada uno indica su fila) |
| Parámetros de cada fila | Sección 5 y tabla de la sección 7 |
| Nota lateral (dos sensores de color) | Sección 6, pasos 10–12 y "por definir" del paso 10. Como no hay sensor trasero, la salida se confirma con el delantero |
| "Primero lógica simple…", inicio en el medio de la zona | Secciones 1 y 2 |
| "Para después: evadir contrincante…" | Sección 9 |
| Opciones de entrega de la caja (desde el aire / en el piso) | Paso 3 (decidido: en el piso) |
