# Raspberry Pi 4B — visión y decisión

La Raspberry Pi es el cerebro: mira por la cámara web USB, decide qué hacer, y
le manda órdenes al ESP32-S3 por USB. El ESP32 no piensa, solo ejecuta.

## Cómo está organizado

```
raspberry-pi/
├── src/athena/          paquete principal
│   ├── protocol.py       contrato binario con el ESP32 (¡espejo de main.cpp!)
│   ├── link.py           enlace serial con reconexión automática
│   ├── camera.py         captura en hilo aparte, descartando frames viejos
│   ├── proposals.py      etapa 1: candidatos por color y forma (barato)
│   ├── classifier.py     etapa 2: CNN INT8 en TFLite (solo sobre candidatos)
│   ├── tracker.py        seguimiento por IoU, evita reclasificar cada frame
│   ├── detector.py       orquesta todo lo anterior
│   ├── geometry.py       distancia y ángulo a partir de la caja
│   ├── decision.py       máquina de estados de la misión + control visual
│   ├── config.py         parámetros ajustables
│   └── types.py          tipos comunes
├── scripts/
│   ├── capture_dataset.py   tomar fotos para entrenar
│   ├── train_classifier.py  entrenar y exportar el .tflite (en tu laptop)
│   ├── benchmark.py         medir el rendimiento real en TU Pi
│   ├── run_rover.py         el bucle principal del robot
│   └── auto_label_ei.py     APARTE, no forma parte de este flujo — ver nota abajo
├── data/                 dataset de entrenamiento (ver data/README.md)
├── models/               el .tflite entrenado
└── tests/                tests que corren sin robot
```

## Instalación en la Raspberry Pi

```bash
sudo apt install python3-opencv python3-numpy python3-serial
pip install --break-system-packages tflite-runtime
```

Se usa `apt` para OpenCV y NumPy a propósito: los paquetes del sistema vienen
compilados con las optimizaciones NEON del ARM. Los de `pip` son ruedas
genéricas y corren bastante más lento.

## Uso

```bash
# Ver que todo funciona, sin mover motores
python3 scripts/run_rover.py --equipo rojo --simular --ver

# En competencia
python3 scripts/run_rover.py --equipo rojo

# Medir el rendimiento real
python3 scripts/benchmark.py
```

---

## Cómo entrenar el clasificador (paso a paso)

Esto es lo que hace falta para que el robot reconozca bandera_roja,
bandera_azul, llave y fondo. Sin este modelo, el robot **igual se mueve**: cae
a reglas de color/forma ("modo degradado", ver más abajo), menos preciso pero
funcional — así que no es un bloqueante para probar mecánica y control.

**1. Tomar fotos** (en la laptop o en la Pi, donde tengas la cámara conectada):

```bash
python3 scripts/capture_dataset.py --clase bandera_roja
python3 scripts/capture_dataset.py --clase bandera_azul
python3 scripts/capture_dataset.py --clase llave
python3 scripts/capture_dataset.py --clase fondo
```

`ESPACIO` guarda foto, `a` activa captura automática, `q` sale. Apunta a
150–300 fotos por clase, variando distancia/ángulo/luz/fondo — detalle
completo en [data/README.md](data/README.md). **`fondo` es la clase más
importante**: mete ahí cinta roja/azul del piso, reflejos, el LED del otro
robot — todo lo que se pueda confundir con una bandera. Las fotos quedan en
`data/raw/<clase>/` y sí se versionan en git (son irremplazables).

**2. Entrenar** — esto va en tu **laptop, NO en la Raspberry Pi** (tardaría
horas ahí):

```bash
pip install "tensorflow>=2.15" pillow
python3 scripts/train_classifier.py
```

Recorta los objetos de las fotos con el mismo detector que usa el robot en
producción, entrena una CNN chica, y exporta `models/athena_cls.tflite`
cuantizado a INT8.

**3. Llevar el modelo a la Raspberry Pi**: copia el archivo
`models/athena_cls.tflite` de tu laptop a la misma ruta relativa en la Pi
(scp, USB, lo que tengas a mano). No se versiona en git — hay que copiarlo a
mano cada vez que se reentrena.

**4. Verificar que la Pi lo está usando**:

```bash
python3 scripts/run_rover.py --equipo rojo --simular --ver
```

`--simular` no mueve motores; `--ver` abre una ventana con lo que detecta y
las cajas dibujadas. En consola, al arrancar debe aparecer algo como
`Clasificador listo: athena_cls.tflite (...)`. Si en cambio dice `Sin modelo
CNN: corriendo en modo degradado`, el `.tflite` no está donde `config.py` lo
busca (`models/athena_cls.tflite`, relativo a esta carpeta `raspberry-pi/`).

> ### ⚠️ `scripts/auto_label_ei.py` es un flujo APARTE, no este
>
> Ese script pre-etiqueta fotos que llegaron de **otro equipo** para subirlas
> a **Edge Impulse Studio** (un servicio externo), reusando las etiquetas
> `CILINDRO_ROJO`/`CILINDRO_AZUL` de un proyecto de Edge Impulse que ya
> existía por fuera de este repo. Su salida (`bounding_boxes.labels`) no se
> conecta con `train_classifier.py` ni con `models/`: sirve para entrenar en
> Edge Impulse, no para generar el `.tflite` que usa `run_rover.py`. Si lo que
> quieres es que la Pi reconozca objetos, el flujo de los 4 pasos de arriba es
> el que necesitas — no este script.

---

## Cómo funciona la visión (y por qué así)

El objetivo era aprovechar los cuatro núcleos de la Pi 4B **sin ahogarlos**,
porque la CPU que gasta la visión es CPU que le falta al control.

### Lo que NO se hizo, y por qué

Lo obvio sería lanzar un detector tipo YOLO o SSD sobre el frame completo. En
una Raspberry Pi 4B eso cuesta **50–80 ms por frame** y ocupa los cuatro
núcleos: 12–20 FPS, sin CPU sobrante para nada más.

### Lo que se hizo: dos etapas

El reglamento dice que las banderas son **cilindros rojos o azules**. Eso no es
un dato menor, es información fuerte sobre el problema. Aprovecharla:

```
frame 320x240
   │
   ├─ 1. PROPUESTAS  (~2-4 ms, 1 núcleo)
   │     umbral HSV + morfología + componentes conexas
   │     → de 76.800 píxeles a ~5 cajas candidatas
   │
   ├─ 2. TRACKING  (<1 ms)
   │     empareja por solapamiento con lo que ya se vio
   │     → la mayoría de las cajas YA están clasificadas
   │
   ├─ 3. CNN INT8  (~1-3 ms por recorte nuevo)
   │     solo sobre lo nuevo, en recortes de 64x64, en lote
   │
   └─ 4. GEOMETRÍA  (despreciable)
         distancia y ángulo por modelo estenopeico
```

El umbral de color hace la **localización** casi gratis. La red neuronal se
encarga solo de lo difícil: distinguir una bandera de la cinta roja del piso,
del LED del robot rival o de una camisa de color.

### Decisiones concretas de optimización

| Decisión | Por qué |
|----------|---------|
| Procesar a 320×240, no 640×480 | Cuesta la cuarta parte. Una bandera a 1.5 m sigue midiendo ~20 px: de sobra. |
| Cuantización INT8 + XNNPACK | XNNPACK usa las instrucciones NEON del Cortex-A72. INT8 procesa 4 valores por instrucción donde float32 procesa 1. |
| 3 hilos, no 4 | Hay que dejarle un núcleo a la captura y al control. Pedir los 4 hace que compitan y sale más lento. |
| Clasificar en lote | Invocar el intérprete tiene costo fijo. Un lote de 6 recortes es mucho más rápido que 6 invocaciones. |
| Reclasificar cada 8 frames | El tracking conserva la identidad. Baja las llamadas a la red casi un 90 %. |
| Buffers reservados una vez | Reservar arrays a 30 FPS le da trabajo constante al recolector de basura, y esas pausas se notan en el control. |
| Captura en hilo con descarte | `read()` devuelve el frame más VIEJO del buffer. Sin descartar, el robot reacciona a lo que vio hace medio segundo. |

**No te fíes de estos números: mide los tuyos** con `scripts/benchmark.py`.
Cambian con el modelo de Pi, la temperatura, la fuente y hasta con cuántos
objetos de color haya en el encuadre.

### Modo degradado

Si no hay `.tflite`, el sistema **sigue funcionando**: clasifica por reglas de
color y proporción (una bandera es 3 veces más alta que ancha; la llave es
casi cuadrada). Detecta peor y se traga más falsos positivos, pero el robot
rueda. Eso permite probar mecánica y control desde el primer día, sin esperar
a tener el dataset.

---

## Cómo se decide qué mandarle al ESP32

`decision.py` es una máquina de estados que sigue el orden del reglamento, con
las prioridades muy claras:

1. **No salirse de la pista.** Sacar dos ruedas pierde la ronda de inmediato.
   La evasión del borde negro pisa cualquier otra decisión, aunque la bandera
   esté justo delante.
2. **No adelantar la secuencia.** Buscar la bandera antes de depositar la llave
   también descalifica. La máquina no puede pasar a buscar hasta que la llave
   esté depositada, y hay un test que lo verifica.
3. **Cumplir la misión.**

```
INICIO → BUSCAR_ZONA_NEUTRA → DEPOSITAR_LLAVE → BUSCAR_BANDERA
       → APROXIMAR_BANDERA → AGARRAR_BANDERA → RETORNAR_A_ZONA
       → ENTREGAR → TERMINADO
```

El seguimiento del objetivo es un **control proporcional** sobre el ángulo: se
gira hacia la bandera y se avanza, bajando la velocidad al acercarse. Es un P
puro, sin I ni D, a propósito: el objetivo está quieto y el lazo corre a 30 Hz,
así que un PID completo solo añadiría dos ganancias más que calibrar.

`DecisionMaker.step()` es una **función pura** de sus entradas: no toca el
puerto serial ni guarda estado escondido. Por eso toda la lógica de misión se
prueba sin robot, sin cámara y sin ESP32.

---

## Tests

```bash
python3 -m pytest tests/ -q
```

28 tests, ninguno necesita hardware. El más importante es
`test_constantes_coinciden_con_el_firmware`: abre el `main.cpp` real, extrae
los números del protocolo y los compara con los de Python. Si alguien cambia un
código de paquete en un lado y olvida el otro, **falla el test** en vez de
fallar el robot en plena competencia.

---

## Lo que falta (hueco conocido)

En `RETORNAR_A_ZONA` el robot avanza recto y solo reconoce su zona cuando ya
pisa la línea de color. Funciona si quedó orientado hacia su lado, pero **no se
recupera si quedó girado**. Resolverlo necesita odometría (encoders en los
motores) o reconocer visualmente la línea de la zona propia. Es el hueco más
grande que queda en la lógica de misión, y está marcado como TODO en el código.

También hay que **calibrar** antes de competir:

- La focal de la cámara en `GeometryConfig` (hoy es una estimación; ver
  `geometry.py` para el procedimiento).
- Los rangos HSV en `ProposalConfig`, con la luz real del salón.
- Los umbrales de color del TCS34725 y de los QTR, del lado del firmware.
