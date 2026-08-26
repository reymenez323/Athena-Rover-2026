"""Orquestador del pipeline de percepción.

    frame  ->  propuestas por color  ->  tracking  ->  CNN (solo si toca)
                                                   ->  geometría
                                                   ->  Perception

Cada etapa quita trabajo a la siguiente, que es la idea completa del diseño:

1. **Propuestas** (2-4 ms): reduce el frame entero a un puñado de cajas.
2. **Tracking** (<1 ms): de esas cajas, la mayoría ya se clasificaron antes.
3. **CNN** (1-3 ms por recorte): solo sobre lo nuevo o lo que toca revisar.
4. **Geometría** (despreciable): pura aritmética.

En la práctica, en una Raspberry Pi 4B esto ronda los 8-15 ms por frame usando
alrededor de un núcleo, contra los 50-80 ms y los cuatro núcleos que costaría
un detector completo tipo SSD. Mide el tuyo con ``scripts/benchmark.py``.

MODO DEGRADADO: si no hay modelo entrenado, se salta el paso 3 y se clasifica
por reglas de color y forma. Menos preciso, pero el robot rueda.
"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

from .classifier import Classifier
from .config import Config
from .geometry import Geometry
from .proposals import ProposalGenerator
from .tracker import Track, Tracker
from .types import BBox, Detection, ObjectClass, Perception


class Detector:
    def __init__(self, cfg: Config, base_dir: Path | None = None) -> None:
        self._cfg = cfg
        self._proposals = ProposalGenerator(cfg.proposals)
        self._tracker = Tracker(cfg.tracker)
        self._classifier = Classifier(cfg.classifier, base_dir=base_dir)
        self._geometry = Geometry(
            cfg.geometry, cfg.camera.process_width, cfg.camera.process_height
        )

        # Buffer del lote reservado una sola vez. Reservar un array por frame a
        # 30 FPS le da trabajo constante al recolector de basura, y en una Pi
        # esas pausas se notan en el control.
        size = cfg.classifier.input_size
        self._crop_buffer = np.zeros(
            (cfg.proposals.max_proposals, size, size, 3), dtype=np.float32
        )

    @property
    def model_active(self) -> bool:
        return self._classifier.available

    def process(self, frame_id: int, frame_bgr: np.ndarray) -> Perception:
        start = time.perf_counter()

        # --- 1. Propuestas -------------------------------------------------
        proposals = self._proposals.generate(frame_bgr)

        # --- 2. Tracking ---------------------------------------------------
        boxes = [(p.box, p.color_hint) for p in proposals]
        alive, pending = self._tracker.update(boxes)

        # --- 3. Clasificación (solo lo que hace falta) ---------------------
        if pending:
            if self._classifier.available:
                self._classify_tracks(frame_bgr, pending)
            else:
                for track in pending:
                    cls, conf = _classify_by_rules(track, frame_bgr.shape[0])
                    self._tracker.apply_classification(track, cls, conf)

        # --- 4. Geometría y salida -----------------------------------------
        detections: list[Detection] = []
        for track in alive:
            # El fondo no es una detección, y un track de un solo frame todavía
            # no es fiable: puede ser un reflejo.
            if track.cls is ObjectClass.FONDO or not track.confirmed:
                continue
            detections.append(
                Detection(
                    cls=track.cls,
                    box=track.box,
                    confidence=track.confidence,
                    distance_mm=self._geometry.distance_mm(track.cls, track.box),
                    angle_deg=self._geometry.angle_deg(track.box),
                    track_id=track.track_id,
                )
            )

        return Perception(
            frame_id=frame_id,
            timestamp=time.time(),
            detections=tuple(detections),
            latency_ms=(time.perf_counter() - start) * 1000.0,
            model_active=self._classifier.available,
        )

    def _classify_tracks(self, frame_bgr: np.ndarray, pending: list[Track]) -> None:
        size = self._cfg.classifier.input_size
        usable = pending[: len(self._crop_buffer)]

        for i, track in enumerate(usable):
            box = track.box
            crop = frame_bgr[box.y : box.y + box.h, box.x : box.x + box.w]
            if crop.size == 0:
                self._crop_buffer[i].fill(0.0)
                continue
            # INTER_AREA al reducir, INTER_LINEAR al ampliar: usar el correcto
            # en cada caso evita introducir artefactos que el modelo no vio
            # durante el entrenamiento.
            interp = cv2.INTER_AREA if crop.shape[0] > size else cv2.INTER_LINEAR
            resized = cv2.resize(crop, (size, size), interpolation=interp)
            # El modelo se entrena en RGB; OpenCV trabaja en BGR.
            self._crop_buffer[i] = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

        results = self._classifier.classify(self._crop_buffer[: len(usable)])
        for track, (cls, confidence) in zip(usable, results):
            self._tracker.apply_classification(track, cls, confidence)

    def reset(self) -> None:
        self._tracker.reset()


def _classify_by_rules(track: Track, frame_height: int) -> tuple[ObjectClass, float]:
    """Respaldo sin red neuronal: decide por color y proporción.

    La bandera es un cilindro de 50 x 150 mm, o sea tres veces más alta que
    ancha. La llave es un cubo: relación cercana a 1. Con eso y el color
    dominante se separan bastante bien los tres casos.

    La confianza se reporta baja (0.5) a propósito: es una estimación, no una
    medición, y quien la use debe saberlo.
    """
    box = track.box
    aspect = box.w / box.h if box.h > 0 else 0.0

    # Claramente más alto que ancho -> bandera. El umbral es 0.75 y no 0.33
    # porque una bandera vista de lejos o inclinada no llega a su proporción
    # ideal, y es preferible proponer de más y que el track lo confirme.
    if aspect < 0.75:
        if track.color_hint == "rojo":
            return ObjectClass.BANDERA_ROJA, 0.5
        return ObjectClass.BANDERA_AZUL, 0.5

    # Casi cuadrado y pequeño -> podría ser la llave.
    if 0.75 <= aspect <= 1.4 and box.h < frame_height * 0.25:
        return ObjectClass.LLAVE, 0.5

    return ObjectClass.FONDO, 0.5
