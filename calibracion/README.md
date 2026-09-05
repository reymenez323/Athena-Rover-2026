# Calibración de sensores

Herramientas de **banco**, no de vuelo. Sirven para caracterizar un sensor con
datos reales **antes** de fijar sus umbrales en el firmware, en vez de
ajustarlos a ojo hasta que "parezca que anda".

Nada de acá corre en competencia. Son sketches que se suben al **mismo
ESP32-S3** del robot, pero con solo el sensor bajo prueba conectado, para poder
pasarlo a mano sobre distintas superficies sin desarmar nada.

## Las dos carpetas

| Carpeta | Sensor | Qué separa | Umbrales que alimenta |
|---|---|---|---|
| [`color/`](color/) | TCS34725 ×2 | Las **zonas de color** del piso: negro, amarillo, rojo, azul, gris | `ClassifyColor()` en `firmware-esp32/src/main.cpp` |
| [`reflectancia/`](reflectancia/) | QTRX-HD-01A ×2 | El **borde negro** de la pista del **fondo gris** | `kDarkThreshold` en `firmware-esp32/src/main.cpp` |

Ese reparto no es casual, y conviene tenerlo claro antes de tocar umbrales: el
sensor de color decide **en qué zona está** el robot; la reflectancia decide
**si se está por salir de la pista**. Son preguntas distintas y por eso hay dos
juegos de sensores.

## El flujo, en las dos carpetas

Ambas siguen el mismo patrón:

1. **`firmware/`** — sketch de captura. Se sube al ESP32 con solo el sensor
   conectado y va escupiendo lecturas crudas por el puerto serie.
2. **`calibrar_*.py`** — corre en la computadora, habla con ese sketch y
   guarda un CSV por superficie en `data_logs/`.
3. **Análisis** — busca los umbrales que menos errores cometen sobre esos
   datos, en vez de moverlos a ojo.
4. **`detector-*/`** — sketch de verificación: aplica los umbrales encontrados
   y los muestra en vivo (por consola y por LED) para comprobarlos con el
   sensor en la mano.

## Los datos ya capturados sí se versionan

`data_logs/` está en el repo a propósito. Son mediciones sobre la pista real,
irremplazables sin volver a montar todo, y son la razón por la que los
umbrales actuales se pueden defender con un número en vez de con una
corazonada:

- **Color:** 970 muestras del sensor delantero. Los umbrales recalibrados
  bajaron el error del 58.1 % al 14.3 %.
- **Reflectancia:** 740 muestras de negro y gris, usadas para elegir el umbral
  y para descubrir que el sensor B separa mejor que el A.

## Lo que falta

- **El sensor de color trasero nunca se caracterizó.** Hoy usa los mismos
  umbrales que el delantero, y eso es un supuesto sin verificar: cada uno
  tiene su propio LED de iluminación, están montados en posiciones distintas
  del chasis, y hay variación normal de fábrica entre dos unidades. Peor aún,
  `ColorSensorTask` **no recalibra en vivo** — los umbrales son fijos, así que
  nada corrige un desajuste durante la competencia. Ver
  [`color/README.md`](color/README.md).
- **`NEGRO` es la clase que peor acierta** por umbrales (58.5 %): se solapa con
  azul y gris. Es un límite estructural de clasificar con reglas encadenadas,
  no algo que otra vuelta de ajuste arregle. El clasificador K-NN de
  `pruebas-platformio/05-evitador-linea/` le acierta bastante mejor con el
  mismo dataset; si el negro necesita ser confiable, vale la pena portar ese
  enfoque al firmware.
