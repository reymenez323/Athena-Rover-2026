"""Tests del control de centrado sobre la caja de Edge Impulse.

Sin cámara, sin modelo, sin robot: ``calcular_giro``/``error_horizontal`` son
funciones puras, igual que ``decision.py``.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from athena.centering import calcular_giro, error_horizontal  # noqa: E402
from athena.types import BBox  # noqa: E402


def caja_centrada_en(x_centro: int, frame_width: int = 120, ancho: int = 20, alto: int = 40) -> BBox:
    return BBox(x=x_centro - ancho // 2, y=10, w=ancho, h=alto)


def test_caja_en_el_centro_da_error_cero():
    box = caja_centrada_en(60, frame_width=120)
    assert error_horizontal(box, 120) == 0.0


def test_caja_en_el_borde_derecho_da_error_positivo_uno():
    box = caja_centrada_en(120, frame_width=120)
    assert error_horizontal(box, 120) == 1.0


def test_caja_en_el_borde_izquierdo_da_error_negativo_uno():
    box = caja_centrada_en(0, frame_width=120)
    assert error_horizontal(box, 120) == -1.0


def test_error_se_recorta_a_menos_uno_uno_aunque_la_caja_se_salga_del_frame():
    box = BBox(x=200, y=0, w=20, h=20)   # muy a la derecha, fuera del frame
    assert error_horizontal(box, 120) == 1.0


def test_frame_width_invalido_da_error_cero_en_vez_de_explotar():
    box = caja_centrada_en(60, frame_width=120)
    assert error_horizontal(box, 0) == 0.0


def test_dentro_de_la_zona_muerta_el_robot_va_recto():
    giro = calcular_giro(0.10, zona_muerta=0.15, velocidad_base=35)
    assert giro.centrado is True
    assert giro.left == giro.right == 35


def test_justo_en_el_borde_de_la_zona_muerta_todavia_cuenta_como_centrado():
    giro = calcular_giro(0.15, zona_muerta=0.15, velocidad_base=35)
    assert giro.centrado is True


def test_objetivo_a_la_derecha_hace_girar_a_la_derecha():
    # Rueda izquierda más rápida que la derecha = gira a la derecha, mismo
    # convenio de signos que decision._perseguir.
    giro = calcular_giro(0.5, zona_muerta=0.15, velocidad_base=35, kp=60.0, correccion_max=40)
    assert giro.centrado is False
    assert giro.left > giro.right


def test_objetivo_a_la_izquierda_hace_girar_a_la_izquierda():
    giro = calcular_giro(-0.5, zona_muerta=0.15, velocidad_base=35, kp=60.0, correccion_max=40)
    assert giro.centrado is False
    assert giro.left < giro.right


def test_la_correccion_no_supera_el_maximo_configurado():
    giro = calcular_giro(1.0, zona_muerta=0.15, velocidad_base=35, kp=60.0, correccion_max=40)
    assert giro.left == 35 + 40
    assert giro.right == 35 - 40


def test_las_velocidades_de_rueda_nunca_salen_del_rango_de_pwm():
    giro = calcular_giro(1.0, zona_muerta=0.0, velocidad_base=90, kp=200.0, correccion_max=100)
    assert -100 <= giro.left <= 100
    assert -100 <= giro.right <= 100


def test_giro_es_proporcional_al_error_no_solo_todo_o_nada():
    chico = calcular_giro(0.2, zona_muerta=0.15, velocidad_base=35, kp=60.0, correccion_max=40)
    grande = calcular_giro(0.6, zona_muerta=0.15, velocidad_base=35, kp=60.0, correccion_max=40)
    correccion_chica = chico.left - 35
    correccion_grande = grande.left - 35
    assert 0 < correccion_chica < correccion_grande
