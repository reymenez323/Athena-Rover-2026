"""Etapa 2: clasificador CNN cuantizado a INT8 sobre TensorFlow Lite.

La red no busca objetos: eso ya lo hizo ``proposals.py``. Su único trabajo es
mirar un recorte de 64x64 y decidir si es una bandera roja, una bandera azul,
la llave, o nada de eso. Es un problema mucho más fácil que la detección
completa, y por eso puede resolverlo un modelo diminuto.

DECISIONES DE RENDIMIENTO
-------------------------
* **INT8 con XNNPACK.** El delegado XNNPACK usa las instrucciones vectoriales
  NEON del Cortex-A72. Con pesos y activaciones en entero de 8 bits procesa
  4 valores por instrucción donde float32 procesaría uno.
* **Lote (batch) de todos los recortes de una vez.** Invocar el intérprete
  tiene un costo fijo por llamada; agrupar 6 recortes en una invocación es
  bastante más rápido que 6 invocaciones de un recorte.
* **3 hilos, no 4.** Hay que dejar un núcleo libre para la captura de cámara y
  el bucle de control. Pedir los cuatro hace que los hilos se peleen entre sí
  y el resultado es más lento, no más rápido.
* **Buffers reservados una sola vez.** Nada de asignar arrays por frame.

Si el archivo del modelo no existe, la clase entera se desactiva sola y el
detector cae a las reglas geométricas. Es a propósito: el robot debe poder
rodar antes de que el dataset esté listo.
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

from .config import ClassifierConfig
from .types import CLASS_ORDER, ObjectClass

log = logging.getLogger(__name__)


def _load_interpreter(model_path: str, num_threads: int):
    """Carga el intérprete de TFLite.

    Se intenta primero ``tflite_runtime``, que es un paquete de ~2 MB con solo
    el intérprete. Instalar TensorFlow completo en una Raspberry Pi son cientos
    de MB y varios minutos de arranque para usar exactamente la misma función.
    """
    try:
        from tflite_runtime.interpreter import Interpreter  # type: ignore
    except ImportError:
        try:
            from tensorflow.lite.python.interpreter import Interpreter  # type: ignore
        except ImportError:
            log.warning(
                "Ni tflite_runtime ni tensorflow están instalados. "
                "El clasificador queda desactivado (modo degradado)."
            )
            return None

    return Interpreter(model_path=model_path, num_threads=num_threads)


class Classifier:
    def __init__(self, cfg: ClassifierConfig, base_dir: Path | None = None) -> None:
        self._cfg = cfg
        self._interpreter = None
        self._input_index = -1
        self._output_index = -1
        self._input_scale = 1.0
        self._input_zero = 0
        self._output_scale = 1.0
        self._output_zero = 0
        self._batch_capacity = 0
        self._is_int8 = False

        path = Path(cfg.model_path)
        if base_dir is not None and not path.is_absolute():
            path = base_dir / path

        if not path.exists():
            log.warning("No se encontró el modelo en %s. Modo degradado activo.", path)
            return

        interpreter = _load_interpreter(str(path), cfg.num_threads)
        if interpreter is None:
            return

        interpreter.allocate_tensors()
        inp = interpreter.get_input_details()[0]
        out = interpreter.get_output_details()[0]

        self._interpreter = interpreter
        self._input_index = inp["index"]
        self._output_index = out["index"]
        self._batch_capacity = int(inp["shape"][0])
        self._is_int8 = inp["dtype"] in (np.int8, np.uint8)

        # Parámetros de cuantización: el modelo INT8 espera enteros, así que
        # hay que mapear el rango float [0,1] al rango entero con los mismos
        # scale/zero_point que se fijaron al cuantizar.
        if self._is_int8:
            self._input_scale, self._input_zero = inp["quantization"]
            self._output_scale, self._output_zero = out["quantization"]

        log.info(
            "Clasificador listo: %s (int8=%s, lote=%d, hilos=%d)",
            path.name, self._is_int8, self._batch_capacity, cfg.num_threads,
        )

    @property
    def available(self) -> bool:
        return self._interpreter is not None

    @property
    def batch_capacity(self) -> int:
        return self._batch_capacity

    def classify(self, crops: np.ndarray) -> list[tuple[ObjectClass, float]]:
        """Clasifica un lote de recortes.

        ``crops``: array (N, input_size, input_size, 3), float32 en [0, 1], RGB.
        Devuelve una lista de (clase, confianza) del mismo largo que la entrada.
        """
        if self._interpreter is None or len(crops) == 0:
            return []

        results: list[tuple[ObjectClass, float]] = []
        batch = max(1, self._batch_capacity)

        # Si llegan más recortes que la capacidad del lote fijo del modelo, se
        # procesan por tandas en vez de fallar.
        for start in range(0, len(crops), batch):
            chunk = crops[start : start + batch]
            results.extend(self._run_batch(chunk, batch))
        return results

    def _run_batch(self, chunk: np.ndarray, batch: int) -> list[tuple[ObjectClass, float]]:
        n = len(chunk)

        # El modelo tiene un lote de tamaño fijo; si la tanda viene corta, se
        # rellena con ceros y luego se descartan esas salidas.
        if n < batch:
            padded = np.zeros((batch, *chunk.shape[1:]), dtype=chunk.dtype)
            padded[:n] = chunk
            chunk = padded

        if self._is_int8:
            quantized = chunk / self._input_scale + self._input_zero
            dtype = self._interpreter.get_input_details()[0]["dtype"]
            info = np.iinfo(dtype)
            tensor = np.clip(np.round(quantized), info.min, info.max).astype(dtype)
        else:
            tensor = chunk.astype(np.float32)

        self._interpreter.set_tensor(self._input_index, tensor)
        self._interpreter.invoke()
        raw = self._interpreter.get_tensor(self._output_index)

        if self._is_int8:
            scores = (raw.astype(np.float32) - self._output_zero) * self._output_scale
        else:
            scores = raw.astype(np.float32)

        out: list[tuple[ObjectClass, float]] = []
        for row in scores[:n]:
            idx = int(np.argmax(row))
            confidence = float(row[idx])
            cls = CLASS_ORDER[idx] if idx < len(CLASS_ORDER) else ObjectClass.FONDO
            if confidence < self._cfg.min_confidence:
                cls = ObjectClass.FONDO
            out.append((cls, confidence))
        return out
