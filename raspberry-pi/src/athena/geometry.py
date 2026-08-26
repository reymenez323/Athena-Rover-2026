"""Estimación de distancia y ángulo a partir de la caja de una detección.

Modelo de cámara estenopeica (pinhole). Si conocemos el tamaño real del objeto
y cuántos píxeles ocupa, la distancia sale de una regla de tres::

    Z = f * H_real / h_píxeles

donde ``f`` es la distancia focal expresada en píxeles. El ángulo sale de lo
desplazado que esté el objeto respecto al centro de la imagen::

    theta = atan((cx - centro) / f)

PRECISIÓN REAL, sin adornos: con una webcam sin calibrar esto da un error del
20-30 % en distancia. Sirve de sobra para decidir "está lejos, avanza" o "está
cerca, cierra la pinza", que es todo lo que necesita la máquina de estados. NO
sirve para posicionar con precisión milimétrica. Para eso está el contacto
físico del gripper y los sensores de reflectancia.

Para mejorarlo hay que medir la focal real: pon la bandera a exactamente
1000 mm, mira cuántos píxeles de alto mide, y despeja f = h_px * 1000 / 150.
"""

from __future__ import annotations

import math

from .config import GeometryConfig
from .types import BBox, ObjectClass


class Geometry:
    def __init__(self, cfg: GeometryConfig, frame_width: int, frame_height: int) -> None:
        self._cfg = cfg
        self._cx = frame_width / 2.0
        self._frame_height = frame_height

    def distance_mm(self, cls: ObjectClass, box: BBox) -> float | None:
        """Distancia estimada al objeto, en milímetros."""
        if cls.es_bandera:
            # Se usa la altura: un cilindro se ve igual de alto desde cualquier
            # lado, mientras que su ancho aparente cambia con la perspectiva.
            if box.h <= 0:
                return None
            # Si la bandera está cortada por el borde del frame, su altura
            # aparente es menor que la real y la distancia saldría inflada.
            if box.y <= 1 or (box.y + box.h) >= self._frame_height - 1:
                return None
            return self._cfg.focal_px * self._cfg.bandera_altura_mm / box.h

        if cls is ObjectClass.LLAVE:
            if box.h <= 0:
                return None
            return self._cfg.focal_px * self._cfg.llave_lado_mm / box.h

        return None

    def angle_deg(self, box: BBox) -> float:
        """Ángulo horizontal al objeto. Positivo = a la derecha del centro."""
        offset_px = box.cx - self._cx
        return math.degrees(math.atan2(offset_px, self._cfg.focal_px))
