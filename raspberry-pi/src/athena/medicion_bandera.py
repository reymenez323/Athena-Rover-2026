"""Qué ve el robot: la bandera contraria como (fuente, error, área) -- UNA sola vez para todos.

Lo usan ``scripts/avisar_bandera_v8.py`` (el que le manda ``B <error> <area>`` / ``N`` al
ESP32) y ``scripts/visor_camara.py`` (para calibrar a ojo desde la PC). Estar en un solo
lugar garantiza que el visor muestra EXACTAMENTE lo que manda el robot: calibrar en uno
vale para el otro.

Prioridad de las fuentes (la primera que vea algo gana):
  1. ``color-cerca``: bandera MUY cerca (10-25 cm), la mancha grande de su color. A esa
     distancia el modelo marca cajas mal centradas y el detector por forma la rechaza
     porque el borde del cuadro la corta.
  2. ``modelo``: Edge Impulse.
  3. ``color+forma``: respaldo por color y proporción.

Todas las cajas se llevan al cuadro COMPLETO de la cámara antes de medir. Con el Impulse en
"Fit shortest axis" el modelo NO ve el cuadro entero: recorta un cuadrado centrado del lado
del eje corto (480x480 de un 640x480) y lo reduce. Medir contra ese cuadrado agranda el error
por ancho_cámara/lado (~1.33 en 4:3) y dibuja las cajas del modelo corridas.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .centering import error_horizontal
from .color_shape_detector import ColorShapeDetection, ColorShapeDetector
from .ei_flag_detector import EiDetection, EiFlagDetector
from .types import BBox

FUENTE_CERCA = "color-cerca"
FUENTE_MODELO = "modelo"
FUENTE_FORMA = "color+forma"


@dataclass(frozen=True)
class Medicion:
    fuente: str | None            # una de las FUENTE_*, o None si no ve la bandera
    error: int                    # -100 (borde izq.) .. 0 (centrada) .. 100 (borde der.), SIN offset
    area: int                     # % del cuadro completo que ocupa la caja
    caja: BBox | None             # en píxeles del cuadro completo de la cámara


SIN_BANDERA = Medicion(None, 0, 0, None)


def mapear_caja_ei(box: BBox, ei_w: int, ei_h: int, frame_w: int, frame_h: int,
                   recorte_centrado: bool = True) -> BBox:
    """Lleva una caja del modelo (píxeles del cuadro reducido) al cuadro completo de la cámara."""
    if recorte_centrado and frame_w != frame_h:
        lado = min(frame_w, frame_h)
        x0 = (frame_w - lado) / 2.0
        y0 = (frame_h - lado) / 2.0
        ex = lado / max(1, ei_w)
        ey = lado / max(1, ei_h)
    else:
        x0 = y0 = 0.0
        ex = frame_w / max(1, ei_w)
        ey = frame_h / max(1, ei_h)
    return BBox(int(round(x0 + box.x * ex)), int(round(y0 + box.y * ey)),
                int(round(box.w * ex)), int(round(box.h * ey)))


def medir_caja(caja: BBox, frame_w: int, frame_h: int) -> tuple[int, int]:
    """(error -100..100, área % del cuadro) de una caja ya en el cuadro completo."""
    error = int(round(100 * error_horizontal(caja, frame_w)))
    area = int(round(100 * caja.area / max(1, frame_w * frame_h)))
    return max(-100, min(100, error)), max(0, min(100, area))


def medir_bandera(
    frame: np.ndarray,
    etiqueta: str,
    ei_detector: EiFlagDetector,
    color_detector: ColorShapeDetector,
    *,
    area_cerca: float = 0.10,
    ei_recorte: bool = True,
) -> tuple[Medicion, list[EiDetection], list[ColorShapeDetection]]:
    """Mide la bandera ``etiqueta`` en un cuadro BGR.

    Devuelve además las detecciones crudas de los dos detectores, por si quien llama
    (el visor) las quiere dibujar; el robot solo usa la ``Medicion``.
    """
    alto, ancho = frame.shape[:2]
    detecciones_ei = ei_detector.detect(frame)
    detecciones_color = color_detector.detect(frame)

    cerca = color_detector.detect_cerca(frame, etiqueta, area_cerca)
    if cerca is not None:
        caja, fraccion = cerca
        error, _ = medir_caja(caja, ancho, alto)
        return Medicion(FUENTE_CERCA, error, max(0, min(100, int(round(100 * fraccion)))), caja), \
            detecciones_ei, detecciones_color

    ei = EiFlagDetector.best(detecciones_ei, etiqueta)
    if ei is not None:
        caja = mapear_caja_ei(ei.box, ei_detector.frame_width, ei_detector.frame_height,
                              ancho, alto, ei_recorte)
        error, area = medir_caja(caja, ancho, alto)
        return Medicion(FUENTE_MODELO, error, area, caja), detecciones_ei, detecciones_color

    forma = ColorShapeDetector.best(detecciones_color, etiqueta)
    if forma is not None:
        error, area = medir_caja(forma.box, ancho, alto)
        return Medicion(FUENTE_FORMA, error, area, forma.box), detecciones_ei, detecciones_color

    return SIN_BANDERA, detecciones_ei, detecciones_color
