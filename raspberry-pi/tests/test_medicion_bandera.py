"""Tests de la medición compartida entre el robot (avisar_bandera_v8.py) y el visor.

Sin cámara ni modelo: detectores de mentira que devuelven lo que se les diga.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

import numpy as np  # noqa: E402

from athena.color_shape_detector import ETIQUETA_ROJO, ColorShapeDetection  # noqa: E402
from athena.ei_flag_detector import EiDetection  # noqa: E402
from athena.medicion_bandera import (  # noqa: E402
    FUENTE_CERCA,
    FUENTE_FORMA,
    FUENTE_MODELO,
    mapear_caja_ei,
    medir_bandera,
    medir_caja,
)
from athena.types import BBox  # noqa: E402


class FakeEi:
    frame_width = 96
    frame_height = 96

    def __init__(self, detecciones):
        self._d = detecciones

    def detect(self, frame):
        return self._d


class FakeColor:
    def __init__(self, detecciones=(), cerca=None):
        self._d = list(detecciones)
        self._cerca = cerca

    def detect(self, frame):
        return self._d

    def detect_cerca(self, frame, etiqueta, area_min_frac=0.10, altura_ancho_min=0.8):
        return self._cerca


FRAME = np.zeros((480, 640, 3), dtype=np.uint8)


def test_una_caja_centrada_del_modelo_da_error_cero_aunque_el_modelo_vea_un_recorte():
    # Recorte cuadrado de 480 centrado en un 640x480: la caja centrada del modelo (x 40..56 de 96)
    # cae en el centro del cuadro completo.
    caja = mapear_caja_ei(BBox(40, 20, 16, 40), 96, 96, 640, 480, True)
    assert abs(caja.cx - 320) <= 2
    error, _ = medir_caja(caja, 640, 480)
    assert error == 0


def test_el_recorte_reduce_el_error_del_modelo_por_ancho_de_lado_a_ancho_de_camara():
    # Caja a la derecha del cuadro reducido: cx = 80 de 96 -> +0.667 sin corregir.
    sin = mapear_caja_ei(BBox(76, 20, 8, 40), 96, 96, 640, 480, False)
    con = mapear_caja_ei(BBox(76, 20, 8, 40), 96, 96, 640, 480, True)
    e_sin, _ = medir_caja(sin, 640, 480)
    e_con, _ = medir_caja(con, 640, 480)
    assert e_sin == 67
    assert e_con == 50      # 67 * 480/640


def test_la_bandera_muy_cerca_gana_sobre_el_modelo_y_sobre_el_respaldo():
    cerca = (BBox(140, 0, 360, 480), 0.5625)
    ei = [EiDetection(ETIQUETA_ROJO, 0.9, BBox(80, 10, 10, 60))]   # caja "mal centrada" del modelo
    m, _, _ = medir_bandera(FRAME, ETIQUETA_ROJO, FakeEi(ei), FakeColor(cerca=cerca))
    assert m.fuente == FUENTE_CERCA
    assert abs(m.error) <= 1
    assert m.area == 56


def test_sin_bandera_muy_cerca_usa_el_modelo():
    ei = [EiDetection(ETIQUETA_ROJO, 0.9, BBox(40, 20, 16, 40))]
    m, _, _ = medir_bandera(FRAME, ETIQUETA_ROJO, FakeEi(ei), FakeColor())
    assert m.fuente == FUENTE_MODELO
    assert m.error == 0


def test_sin_modelo_usa_el_respaldo_por_forma():
    forma = ColorShapeDetection(ETIQUETA_ROJO, 0.8, BBox(400, 100, 30, 90), 3.0, "de_pie")
    m, _, _ = medir_bandera(FRAME, ETIQUETA_ROJO, FakeEi([]), FakeColor([forma]))
    assert m.fuente == FUENTE_FORMA
    assert m.error > 0          # a la derecha del centro


def test_sin_nada_no_ve_la_bandera():
    m, _, _ = medir_bandera(FRAME, ETIQUETA_ROJO, FakeEi([]), FakeColor())
    assert m.fuente is None
    assert m.caja is None
