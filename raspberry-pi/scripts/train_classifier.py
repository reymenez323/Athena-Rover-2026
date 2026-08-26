#!/usr/bin/env python3
"""Entrena el clasificador y lo exporta cuantizado a INT8 para la Raspberry Pi.

IMPORTANTE: esto se corre en una COMPUTADORA, no en la Raspberry Pi. Entrenar
en la Pi funcionaría, pero tardaría horas. Entrena en tu laptop, y copia el
``.tflite`` resultante a ``models/`` de la Pi.

Uso::

    pip install "tensorflow>=2.15" pillow
    python3 scripts/train_classifier.py

Qué hace, en orden:

1. Recorta objetos de ``data/raw/`` usando el MISMO generador de propuestas que
   corre en el robot. Esto no es un detalle: si entrenas con recortes hechos a
   mano y en producción el modelo recibe recortes del detector automático, le
   estás dando en producción imágenes distintas a las que aprendió. Usando el
   mismo generador, entrenamiento y ejecución ven exactamente lo mismo.
2. Entrena una CNN pequeña con aumento de datos.
3. Cuantiza a INT8 con un conjunto representativo y exporta el .tflite.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from athena.config import Config  # noqa: E402
from athena.proposals import ProposalGenerator  # noqa: E402
from athena.types import CLASS_ORDER, ObjectClass  # noqa: E402


def generar_recortes(cfg: Config, verbose: bool = True) -> tuple[np.ndarray, np.ndarray]:
    """Convierte data/raw/<clase>/*.jpg en recortes etiquetados de NxN."""
    generator = ProposalGenerator(cfg.proposals)
    size = cfg.classifier.input_size
    crops_dir = REPO / "data" / "crops"

    X: list[np.ndarray] = []
    y: list[int] = []

    for idx, cls in enumerate(CLASS_ORDER):
        carpeta = REPO / "data" / "raw" / cls.value
        imagenes = sorted(carpeta.glob("*.jpg")) + sorted(carpeta.glob("*.png"))
        salida = crops_dir / cls.value
        salida.mkdir(parents=True, exist_ok=True)

        n_recortes = 0
        for ruta in imagenes:
            img = cv2.imread(str(ruta))
            if img is None:
                continue
            img = cv2.resize(
                img, (cfg.camera.process_width, cfg.camera.process_height),
                interpolation=cv2.INTER_AREA,
            )

            propuestas = generator.generate(img)

            # Nota: para la clase 'fondo' el generador propone justamente lo
            # que confunde al robot (cinta roja, reflejos, LEDs). Eso es
            # exactamente lo que queremos como negativos.
            for i, p in enumerate(propuestas):
                b = p.box
                recorte = img[b.y : b.y + b.h, b.x : b.x + b.w]
                if recorte.size == 0:
                    continue
                interp = cv2.INTER_AREA if recorte.shape[0] > size else cv2.INTER_LINEAR
                recorte = cv2.resize(recorte, (size, size), interpolation=interp)
                cv2.imwrite(str(salida / f"{ruta.stem}_{i}.jpg"), recorte)
                X.append(cv2.cvtColor(recorte, cv2.COLOR_BGR2RGB))
                y.append(idx)
                n_recortes += 1

        if verbose:
            print(f"  {cls.value:15s} {len(imagenes):4d} imágenes -> {n_recortes:5d} recortes")

    if not X:
        raise SystemExit(
            "No se generó ningún recorte. ¿Están vacías las carpetas de data/raw/?\n"
            "Captura imágenes primero con scripts/capture_dataset.py"
        )

    return np.asarray(X, dtype=np.uint8), np.asarray(y, dtype=np.int32)


def construir_modelo(input_size: int, n_clases: int):
    import tensorflow as tf
    from tensorflow.keras import layers, models

    # Arquitectura deliberadamente pequeña. Cuatro clases visualmente muy
    # distintas no necesitan una MobileNet: una red de ~100k parámetros aprende
    # esto de sobra, entrena en minutos y corre en 1-2 ms por recorte en la Pi.
    # Una red grande aquí solo serviría para sobreajustar un dataset pequeño.
    return models.Sequential([
        layers.Input(shape=(input_size, input_size, 3)),
        layers.Rescaling(1.0 / 255),

        # Aumento de datos DENTRO del modelo: solo se activa en entrenamiento,
        # así que no hay riesgo de dejarlo puesto por error al inferir.
        layers.RandomFlip("horizontal"),
        layers.RandomRotation(0.08),
        layers.RandomZoom(0.15),
        layers.RandomBrightness(0.25),   # la luz del salón es lo que más varía
        layers.RandomContrast(0.2),

        layers.Conv2D(16, 3, activation="relu", padding="same"),
        layers.MaxPooling2D(),
        layers.Conv2D(32, 3, activation="relu", padding="same"),
        layers.MaxPooling2D(),
        layers.Conv2D(64, 3, activation="relu", padding="same"),
        layers.MaxPooling2D(),

        layers.GlobalAveragePooling2D(),
        layers.Dropout(0.3),
        layers.Dense(64, activation="relu"),
        layers.Dense(n_clases, activation="softmax"),
    ])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--epocas", type=int, default=40)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--lote-inferencia", type=int, default=8,
                        help="tamaño de lote fijo del modelo exportado")
    args = parser.parse_args()

    cfg = Config.load(None)

    print("1) Generando recortes con el mismo detector que usa el robot...")
    X, y = generar_recortes(cfg)
    print(f"   Total: {len(X)} recortes, {len(CLASS_ORDER)} clases\n")

    # Aviso honesto: un dataset desbalanceado entrena un modelo sesgado.
    conteos = np.bincount(y, minlength=len(CLASS_ORDER))
    if conteos.min() * 4 < conteos.max():
        print("   AVISO: el dataset está desbalanceado.")
        for cls, n in zip(CLASS_ORDER, conteos):
            print(f"     {cls.value:15s} {n:5d}")
        print("   Captura más imágenes de las clases con menos ejemplos.\n")

    import tensorflow as tf

    # Mezcla y partición 80/20. La semilla fija hace el experimento repetible.
    rng = np.random.default_rng(42)
    orden = rng.permutation(len(X))
    X, y = X[orden], y[orden]
    corte = int(len(X) * 0.8)
    X_train, X_val = X[:corte], X[corte:]
    y_train, y_val = y[:corte], y[corte:]

    print(f"2) Entrenando ({len(X_train)} train / {len(X_val)} validación)...")
    modelo = construir_modelo(cfg.classifier.input_size, len(CLASS_ORDER))
    modelo.compile(
        optimizer=tf.keras.optimizers.Adam(1e-3),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"],
    )
    modelo.fit(
        X_train, y_train,
        validation_data=(X_val, y_val),
        epochs=args.epocas,
        batch_size=args.batch,
        callbacks=[
            # Se queda con los mejores pesos, no con los de la última época:
            # el final del entrenamiento suele estar sobreajustado.
            tf.keras.callbacks.EarlyStopping(
                monitor="val_accuracy", patience=8, restore_best_weights=True
            ),
            tf.keras.callbacks.ReduceLROnPlateau(monitor="val_loss", patience=4, factor=0.5),
        ],
        verbose=2,
    )

    print("\n3) Cuantizando a INT8 y exportando...")

    # El modelo que se exporta NO lleva las capas de aumento de datos, y tiene
    # el lote fijo que espera classifier.py.
    inferencia = tf.keras.Sequential([
        tf.keras.layers.Input(shape=(cfg.classifier.input_size, cfg.classifier.input_size, 3),
                              batch_size=args.lote_inferencia),
        *[capa for capa in modelo.layers if not capa.name.startswith("random")],
    ])

    def dataset_representativo():
        # La cuantización necesita ver datos reales para elegir los rangos de
        # cada capa. Con datos sintéticos, los rangos salen mal y la precisión
        # se desploma.
        for i in range(0, min(len(X_train), 200), args.lote_inferencia):
            lote = X_train[i : i + args.lote_inferencia].astype(np.float32)
            if len(lote) < args.lote_inferencia:
                break
            yield [lote]

    convertidor = tf.lite.TFLiteConverter.from_keras_model(inferencia)
    convertidor.optimizations = [tf.lite.Optimize.DEFAULT]
    convertidor.representative_dataset = dataset_representativo
    convertidor.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    convertidor.inference_input_type = tf.int8
    convertidor.inference_output_type = tf.int8

    tflite = convertidor.convert()
    destino = REPO / cfg.classifier.model_path
    destino.parent.mkdir(parents=True, exist_ok=True)
    destino.write_bytes(tflite)

    print(f"\nListo: {destino}  ({len(tflite) / 1024:.0f} KB)")
    print("Cópialo a la carpeta models/ de la Raspberry Pi y vuelve a correr benchmark.py.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
