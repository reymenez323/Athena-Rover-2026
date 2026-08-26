"""Tipos comunes del pipeline de percepción."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ObjectClass(str, Enum):
    """Clases que el clasificador distingue.

    Los valores coinciden con los nombres de las carpetas en ``data/raw/``,
    para que el entrenamiento no necesite ninguna tabla de traducción.
    """

    BANDERA_ROJA = "bandera_roja"
    BANDERA_AZUL = "bandera_azul"
    LLAVE = "llave"
    FONDO = "fondo"

    @property
    def es_bandera(self) -> bool:
        return self in (ObjectClass.BANDERA_ROJA, ObjectClass.BANDERA_AZUL)


# Orden fijo de las clases para el modelo. Los índices de salida del .tflite
# corresponden a esta tupla, así que NO se reordena sin reentrenar.
CLASS_ORDER: tuple[ObjectClass, ...] = (
    ObjectClass.BANDERA_ROJA,
    ObjectClass.BANDERA_AZUL,
    ObjectClass.LLAVE,
    ObjectClass.FONDO,
)


@dataclass(frozen=True)
class BBox:
    """Caja en píxeles, sobre el frame ya reducido de trabajo."""

    x: int
    y: int
    w: int
    h: int

    @property
    def cx(self) -> float:
        return self.x + self.w / 2.0

    @property
    def cy(self) -> float:
        return self.y + self.h / 2.0

    @property
    def area(self) -> int:
        return self.w * self.h

    def iou(self, other: "BBox") -> float:
        ix1, iy1 = max(self.x, other.x), max(self.y, other.y)
        ix2 = min(self.x + self.w, other.x + other.w)
        iy2 = min(self.y + self.h, other.y + other.h)
        iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
        inter = iw * ih
        union = self.area + other.area - inter
        return inter / union if union > 0 else 0.0


@dataclass(frozen=True)
class Detection:
    """Un objeto reconocido en el frame actual."""

    cls: ObjectClass
    box: BBox
    confidence: float
    distance_mm: float | None = None   # None si no hay calibración de cámara
    angle_deg: float = 0.0             # + = a la derecha del centro de la imagen
    track_id: int = -1


@dataclass(frozen=True)
class Perception:
    """Todo lo que el robot cree ver en este instante."""

    frame_id: int
    timestamp: float
    detections: tuple[Detection, ...]
    latency_ms: float = 0.0
    model_active: bool = False   # False = corriendo en modo degradado, sin CNN

    def best(self, cls: ObjectClass) -> Detection | None:
        """La detección más confiable de una clase, o None."""
        matches = [d for d in self.detections if d.cls is cls]
        return max(matches, key=lambda d: d.confidence) if matches else None

    def best_flag(self, team: ObjectClass) -> Detection | None:
        return self.best(team)
