"""Puente hacia el modelo de banderas entrenado en Edge Impulse (FOMO).

Es la ÚNICA fuente de percepción por cámara del robot: corre el modelo FOMO
entrenado en Edge Impulse (con fotos tomadas con la cámara del robot) y
devuelve directamente las cajas de las banderas rojas y azules sobre el
frame. El entrenamiento se hizo FUERA de este repo, en Edge Impulse Studio;
acá solo se ejecuta el ``.eim`` ya exportado.

La llave no pasa por acá ni por ninguna cámara: el robot arranca con ella en
la pinza y la suelta cuando el sensor de color del ESP32 ve el amarillo de la
zona neutra (ver ``decision.py``).

REQUISITOS en la Raspberry Pi (no hacen falta en una laptop de desarrollo)::

    pip3 install edge_impulse_linux
    chmod +x models/athena_ei_banderas.eim   # el .eim necesita permiso de ejecución

⚠️ HACE FALTA RASPBERRY PI OS DE **64 BITS**. El ``.eim`` no es un archivo de
datos: es un ejecutable ELF compilado para AArch64 (así se exportó de Edge
Impulse, target "Linux (AARCH64)"). En un Raspberry Pi OS de 32 bits —que el
instalador sigue ofreciendo— no corre, y el fallo es confuso de dos maneras a
la vez: ``requirements.txt`` marca ``edge_impulse_linux`` con
``platform_machine == "aarch64"``, así que pip lo salta en silencio, y lo que
se ve es "falta el SDK" en vez de "el sistema es de 32 bits". Por eso se
comprueba explícitamente al construir el detector.

COORDENADAS -- esto es lo más fácil de meter la pata: las cajas que devuelve
``detect()`` NO están en la resolución de la cámara. El SDK de Edge Impulse
reescala/recorta el frame al tamaño que configuraste en el Impulse (120x120,
modo "Squash" en este proyecto) antes de correr el modelo, y las cajas salen
en ESE tamaño reducido. Por eso ``frame_width``/``frame_height`` se actualizan
en cada ``detect()``: quien las use para calcular un error de centrado tiene
que medirlo contra esas dimensiones, no contra las de la cámara.
"""

from __future__ import annotations

import logging
import platform
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .types import BBox

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class EiDetection:
    """Una detección del modelo de Edge Impulse, en píxeles del frame reducido."""

    label: str
    confidence: float
    box: BBox


class EiFlagDetector:
    """Envuelve ``ImageImpulseRunner`` del SDK ``edge_impulse_linux``."""

    def __init__(self, model_path: str | Path, min_confidence: float = 0.6) -> None:
        # Se comprueba ANTES de importar el SDK: en un sistema de 32 bits ese
        # import falla por "no instalado" (pip lo saltó por el marcador de
        # plataforma) y ese mensaje mandaría a instalar algo que tampoco iba a
        # funcionar. Mejor decir la causa de verdad.
        #
        # Se listan las arquitecturas MALAS, no las buenas: así una laptop de
        # desarrollo (x86_64, AMD64, Apple Silicon) no queda bloqueada por no
        # estar en una lista blanca.
        maquina = platform.machine().lower()
        if maquina.startswith(("armv6", "armv7", "armv8l")):
            raise RuntimeError(
                f"El modelo de Edge Impulse es un ejecutable de 64 bits (AArch64) "
                f"y este sistema es '{platform.machine()}', de 32 bits. Hace falta "
                f"Raspberry Pi OS de 64 BITS: reinstalá eligiendo 'Raspberry Pi OS "
                f"(64-bit)' en Raspberry Pi Imager."
            )

        try:
            from edge_impulse_linux.image import ImageImpulseRunner
        except ImportError as exc:
            raise RuntimeError(
                "Falta el SDK de Edge Impulse para Linux. Instálalo con:\n"
                "    pip3 install edge_impulse_linux"
            ) from exc

        path = Path(model_path)
        if not path.exists():
            raise RuntimeError(
                f"No se encontró el modelo en {path}. "
                "Copia el .eim exportado de Edge Impulse (target 'Linux (AARCH64)') "
                "a esa ruta y dale permiso de ejecución (chmod +x)."
            )

        self._min_confidence = min_confidence
        self._runner = ImageImpulseRunner(str(path))
        info = self._runner.init()

        project = info.get("project", {})
        self.labels: tuple[str, ...] = tuple(info["model_parameters"]["labels"])
        log.info(
            "Modelo Edge Impulse cargado: %s / %s (clases: %s)",
            project.get("owner", "?"), project.get("name", path.name), ", ".join(self.labels),
        )

        # Tamaño real del último frame que el SDK recortó/ajustó; se actualiza
        # en cada detect() porque es el sistema de referencia de esas cajas.
        self.frame_width = 0
        self.frame_height = 0
        self.last_timing_ms = 0.0

    def __enter__(self) -> "EiFlagDetector":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        self._runner.stop()

    def detect(self, frame_bgr: np.ndarray) -> list[EiDetection]:
        """Corre el modelo sobre un frame BGR (el que entrega ``Camera``)."""
        # El SDK espera RGB; OpenCV (y por lo tanto Camera) trabaja en BGR.
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        features, cropped = self._runner.get_features_from_image_auto_studio_settings(frame_rgb)
        self.frame_height, self.frame_width = cropped.shape[:2]

        res = self._runner.classify(features)
        timing = res.get("timing", {})
        self.last_timing_ms = float(timing.get("dsp", 0) + timing.get("classification", 0))

        detections: list[EiDetection] = []
        for bb in res["result"].get("bounding_boxes", []):
            if bb["value"] < self._min_confidence:
                continue
            detections.append(EiDetection(
                label=bb["label"],
                confidence=float(bb["value"]),
                box=BBox(int(bb["x"]), int(bb["y"]), int(bb["width"]), int(bb["height"])),
            ))
        return detections

    @staticmethod
    def best(detections: list[EiDetection], label: str) -> EiDetection | None:
        """La detección más confiable de una clase puntual, o None."""
        candidatas = [d for d in detections if d.label == label]
        return max(candidatas, key=lambda d: d.confidence) if candidatas else None
