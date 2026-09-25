"""Tests del detector de banderas por color + forma.

Sin cámara, sin robot: se dibujan rectángulos de color a mano sobre un frame
en blanco y se corre el detector encima, igual que test_centering.py hace con
las cajas de Edge Impulse.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from athena.color_shape_detector import (  # noqa: E402
    ETIQUETA_AZUL,
    ETIQUETA_ROJO,
    ColorShapeDetector,
)
from athena.config import GeometryConfig  # noqa: E402

# Rojo y azul "de libro" en BGR (los que devuelve cv2.cvtColor(HSV->BGR) para
# H=0 y H=115, S=200, V=200), suficientes para caer dentro de los rangos por
# defecto sin depender de una foto real.
BGR_ROJO = (36, 28, 237)
BGR_AZUL = (200, 70, 40)


def _frame_con_rectangulo(color_bgr, ancho: int, alto: int, lienzo: int = 200) -> np.ndarray:
    """Frame gris parejo con un rectángulo centrado del color y forma dados."""
    frame = np.full((lienzo, lienzo, 3), 40, dtype=np.uint8)
    x0 = (lienzo - ancho) // 2
    y0 = (lienzo - alto) // 2
    cv2.rectangle(frame, (x0, y0), (x0 + ancho, y0 + alto), color_bgr, thickness=-1)
    return frame


def _detector(**kwargs) -> ColorShapeDetector:
    # Geometría explícita en vez de la real del reglamento: así el test no se
    # rompe si algún día cambia bandera_altura_mm/diametro_mm del robot real.
    geometry = GeometryConfig(bandera_altura_mm=150.0, bandera_diametro_mm=50.0)
    return ColorShapeDetector(geometry=geometry, **kwargs)


def test_rectangulo_rojo_con_el_aspecto_correcto_se_detecta():
    # 150/50 = 3.0 de relación alto/ancho, igual que el cilindro real.
    frame = _frame_con_rectangulo(BGR_ROJO, ancho=30, alto=90)
    detecciones = _detector().detect(frame)
    objetivo = ColorShapeDetector.best(detecciones, ETIQUETA_ROJO)
    assert objetivo is not None
    assert objetivo.confidence > 0.5


def test_rectangulo_azul_con_el_aspecto_correcto_se_detecta():
    frame = _frame_con_rectangulo(BGR_AZUL, ancho=30, alto=90)
    detecciones = _detector().detect(frame)
    objetivo = ColorShapeDetector.best(detecciones, ETIQUETA_AZUL)
    assert objetivo is not None


def test_bandera_caida_de_lado_tambien_se_detecta():
    # Misma proporción 3:1 pero invertida (ancho=90, alto=30): el cilindro
    # tumbado, visto de lado -- simula que se cayó a mitad de ronda.
    frame = _frame_con_rectangulo(BGR_ROJO, ancho=90, alto=30)
    detecciones = _detector().detect(frame)
    objetivo = ColorShapeDetector.best(detecciones, ETIQUETA_ROJO)
    assert objetivo is not None
    assert objetivo.orientation == "caida"


def test_bandera_de_pie_se_marca_con_la_orientacion_correcta():
    frame = _frame_con_rectangulo(BGR_ROJO, ancho=30, alto=90)
    objetivo = ColorShapeDetector.best(_detector().detect(frame), ETIQUETA_ROJO)
    assert objetivo is not None
    assert objetivo.orientation == "de_pie"


def test_no_confunde_un_color_con_el_otro():
    frame = _frame_con_rectangulo(BGR_ROJO, ancho=30, alto=90)
    detecciones = _detector().detect(frame)
    assert ColorShapeDetector.best(detecciones, ETIQUETA_AZUL) is None


def test_blob_cuadrado_del_color_correcto_se_descarta_por_forma():
    # Mismo color, pero 1:1 en vez de 3:1 -- una pared roja, no un cilindro.
    frame = _frame_con_rectangulo(BGR_ROJO, ancho=60, alto=60)
    detecciones = _detector().detect(frame)
    assert ColorShapeDetector.best(detecciones, ETIQUETA_ROJO) is None


def test_blob_muy_alargado_del_color_correcto_tambien_se_descarta():
    # 6:1, el doble de alargado que el cilindro real.
    frame = _frame_con_rectangulo(BGR_ROJO, ancho=15, alto=90)
    detecciones = _detector().detect(frame)
    assert ColorShapeDetector.best(detecciones, ETIQUETA_ROJO) is None


def test_frame_sin_ningun_color_de_equipo_no_detecta_nada():
    frame = np.full((200, 200, 3), 40, dtype=np.uint8)  # gris parejo
    detecciones = _detector().detect(frame)
    assert detecciones == []


def test_blob_mas_chico_que_el_area_minima_se_descarta():
    frame = _frame_con_rectangulo(BGR_ROJO, ancho=6, alto=18)  # área ~ misma relación, diminuto
    detecciones = _detector(area_min_px=150).detect(frame)
    assert ColorShapeDetector.best(detecciones, ETIQUETA_ROJO) is None


def test_tolerancia_de_aspecto_mas_ancha_deja_pasar_lo_que_antes_se_rechazaba():
    frame = _frame_con_rectangulo(BGR_ROJO, ancho=60, alto=60)  # 1:1, muy lejos de 3:1
    # min_confidence en 0 a propósito: lo que se prueba acá es el filtro de
    # aspecto en aislado, no la confianza compuesta (que también cae con un
    # aspecto tan alejado del esperado, y taparía lo que se quiere ver).
    estricto = _detector(tolerancia_aspecto=0.35, min_confidence=0.0).detect(frame)
    laxo = _detector(tolerancia_aspecto=1.5, min_confidence=0.0).detect(frame)
    assert ColorShapeDetector.best(estricto, ETIQUETA_ROJO) is None
    assert ColorShapeDetector.best(laxo, ETIQUETA_ROJO) is not None


def test_deteccion_reporta_la_relacion_de_aspecto_medida():
    frame = _frame_con_rectangulo(BGR_ROJO, ancho=30, alto=90)
    objetivo = ColorShapeDetector.best(_detector().detect(frame), ETIQUETA_ROJO)
    assert objetivo is not None
    assert abs(objetivo.aspect_ratio - 3.0) < 0.2


# --- detect_cerca: bandera MUY cerca, llena el cuadro (2026-09-24) ---------

def _frame_ancho(color_bgr, x0: int, x1: int, y0: int, y1: int, ancho: int = 640, alto: int = 480) -> np.ndarray:
    frame = np.full((alto, ancho, 3), 40, dtype=np.uint8)
    cv2.rectangle(frame, (x0, y0), (x1, y1), color_bgr, thickness=-1)
    return frame


def test_detect_cerca_ve_la_bandera_que_llena_el_cuadro_y_ya_no_cumple_la_proporcion():
    # 360 de ancho x 480 de alto, cortada por arriba y por abajo: proporción ~1.3, no 3.
    frame = _frame_ancho(BGR_ROJO, 140, 500, 0, 479)
    assert ColorShapeDetector.best(_detector().detect(frame), ETIQUETA_ROJO) is None
    resultado = _detector().detect_cerca(frame, ETIQUETA_ROJO)
    assert resultado is not None
    caja, fraccion = resultado
    assert abs(caja.cx - 320) < 3          # centrada
    assert fraccion > 0.3


def test_detect_cerca_mide_bien_el_lado_cuando_no_esta_centrada():
    frame = _frame_ancho(BGR_AZUL, 400, 600, 0, 479)
    caja, _ = _detector().detect_cerca(frame, ETIQUETA_AZUL)
    assert caja.cx > 480                    # a la derecha del centro (320)


def test_detect_cerca_descarta_una_franja_ancha_y_baja():
    # La franja de color de la pista vista de cerca: 600 x 90, alto/ancho = 0.15.
    frame = _frame_ancho(BGR_ROJO, 20, 620, 300, 390)
    assert _detector().detect_cerca(frame, ETIQUETA_ROJO) is None


def test_detect_cerca_ignora_manchas_pequenas():
    frame = _frame_ancho(BGR_ROJO, 300, 340, 200, 300)   # 40x100: ~1 % del cuadro
    assert _detector().detect_cerca(frame, ETIQUETA_ROJO) is None
