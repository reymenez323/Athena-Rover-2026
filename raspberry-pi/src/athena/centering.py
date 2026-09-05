"""Control proporcional de giro a partir de dónde cae una caja en el frame.

Función pura, sin cámara ni serial: se puede probar entera sin robot (ver
``tests/test_centering.py``), mismo criterio que ``decision.py``.

MODELO: se traza una línea vertical de referencia en el centro del frame. El
"error" es cuánto se desvía el centro de la caja de esa línea, normalizado a
[-1, 1] (0 = centrado, +1 = pegado al borde derecho, -1 = al borde izquierdo).
Dentro de una zona muerta alrededor del centro se considera "centrado" y el
robot avanza recto; fuera de ella, el giro es proporcional a qué tan lejos
está del centro -- control P puro, mismo espíritu que ``decision._perseguir``
pero sobre un error normalizado en vez de un ángulo en grados.
"""

from __future__ import annotations

from dataclasses import dataclass

from .types import BBox


@dataclass(frozen=True)
class Giro:
    error: float             # -1 (borde izq.) .. 0 (centro) .. 1 (borde der.)
    centrado: bool
    left: int
    right: int


def error_horizontal(box: BBox, frame_width: int) -> float:
    """Qué tan lejos está el centro de la caja de la línea central, en [-1, 1].

    Positivo = la caja está a la derecha del centro del frame.
    """
    if frame_width <= 0:
        return 0.0
    centro_frame = frame_width / 2.0
    error = (box.cx - centro_frame) / centro_frame
    return max(-1.0, min(1.0, error))


def calcular_giro(
    error: float,
    *,
    zona_muerta: float = 0.15,
    velocidad_base: int = 35,
    kp: float = 60.0,
    correccion_max: int = 40,
) -> Giro:
    """Traduce el error horizontal en velocidades de rueda izquierda/derecha.

    Error positivo (objetivo a la derecha) -> gira a la derecha, es decir la
    rueda izquierda queda más rápida que la derecha. Mismo convenio de signos
    que ``Detection.angle_deg`` y ``decision._perseguir``, para no meter una
    inversión de sentido a la hora de integrarlo con el resto del robot.
    """
    if abs(error) <= zona_muerta:
        return Giro(error=error, centrado=True, left=velocidad_base, right=velocidad_base)

    correccion = int(kp * error)
    correccion = max(-correccion_max, min(correccion_max, correccion))

    left = max(-100, min(100, velocidad_base + correccion))
    right = max(-100, min(100, velocidad_base - correccion))
    return Giro(error=error, centrado=False, left=left, right=right)
