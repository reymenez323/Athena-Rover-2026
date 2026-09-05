"""Tipos comunes de la percepción.

Son el idioma interno de la Raspberry Pi: ``ei_flag_detector.py`` traduce lo
que devuelve Edge Impulse a estos tipos, y ``decision.py`` solo conoce estos.
Gracias a eso, cambiar de modelo (o de detector entero) no obliga a tocar la
máquina de estados de la misión.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ObjectClass(str, Enum):
    """Lo que la cámara tiene que reconocer.

    Solo las dos banderas. La llave NO está aquí a propósito: nunca dependió
    de la cámara — el robot arranca con ella ya en la pinza y la suelta cuando
    el sensor de color del ESP32 ve el amarillo de la zona neutra (ver
    ``decision.py``). Meterla como clase de visión sería resolver por el
    camino difícil algo que ya está resuelto por el fácil.
    """

    BANDERA_ROJA = "bandera_roja"
    BANDERA_AZUL = "bandera_azul"


@dataclass(frozen=True)
class BBox:
    """Caja en píxeles.

    OJO con el sistema de referencia: las cajas que salen del modelo de Edge
    Impulse están en píxeles del frame YA reducido por el SDK (120x120), no de
    la resolución de la cámara. Ver ``ei_flag_detector.EiFlagDetector``.
    """

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
    model_active: bool = False   # False = el modelo no está corriendo

    def best(self, cls: ObjectClass) -> Detection | None:
        """La detección más confiable de una clase, o None."""
        matches = [d for d in self.detections if d.cls is cls]
        return max(matches, key=lambda d: d.confidence) if matches else None
