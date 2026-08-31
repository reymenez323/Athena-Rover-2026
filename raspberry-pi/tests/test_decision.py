"""Tests de la capa de decisión.

Toda la lógica de misión se prueba sin robot, sin cámara y sin ESP32: es la
razón de que ``DecisionMaker.step()`` sea una función pura de sus entradas.

Los dos tests que más importan son los que verifican las dos reglas que
descalifican de inmediato:
  · buscar la bandera antes de depositar la llave,
  · salirse de la pista.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from athena.config import ControlConfig  # noqa: E402
from athena.decision import Commands, DecisionMaker, Phase, RobotState  # noqa: E402
from athena.protocol import (  # noqa: E402
    ColorLabel,
    ColorTelemetry,
    GripperAction,
    ReflectTelemetry,
    TeamColor,
    ToFTelemetry,
)
from athena.types import BBox, Detection, ObjectClass, Perception  # noqa: E402

CFG = ControlConfig()


def percepcion(*detecciones: Detection) -> Perception:
    return Perception(frame_id=1, timestamp=0.0, detections=tuple(detecciones))


def deteccion(cls: ObjectClass, *, angulo: float = 0.0, distancia: float | None = 800.0) -> Detection:
    return Detection(
        cls=cls,
        box=BBox(150, 100, 20, 60),
        confidence=0.9,
        distance_mm=distancia,
        angle_deg=angulo,
        track_id=1,
    )


def color(front: ColorLabel) -> ColorTelemetry:
    return ColorTelemetry(timestamp_ms=0, front=front, back=ColorLabel.FLOOR,
                          front_valid=True, back_valid=True)


def reflect(izq: bool = False, der: bool = False) -> ReflectTelemetry:
    return ReflectTelemetry(timestamp_ms=0, left_raw=0, right_raw=0,
                            left_on_line=izq, right_on_line=der)


def tof(distancia_mm: int, *, valido: bool = True) -> ToFTelemetry:
    return ToFTelemetry(timestamp_ms=0, distance_mm=distancia_mm, valid=valido)


# ---------------------------------------------------------------------------
# Reglas que descalifican
# ---------------------------------------------------------------------------


def test_no_se_busca_la_bandera_antes_de_depositar_la_llave():
    """Es descalificación inmediata según el reglamento."""
    d = DecisionMaker(CFG, RobotState(phase=Phase.BUSCAR_BANDERA, team=TeamColor.RED))
    # llave_depositada es False; aunque la bandera esté a la vista...
    d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL)), None, None)
    # ...la máquina se devuelve a la fase de la llave en vez de perseguirla.
    assert d.state.phase is Phase.BUSCAR_ZONA_NEUTRA


def test_la_secuencia_completa_respeta_el_orden_del_reglamento():
    d = DecisionMaker(CFG, RobotState(team=TeamColor.RED))
    assert d.state.phase is Phase.INICIO

    d.step(percepcion(), None, None)
    assert d.state.phase is Phase.BUSCAR_ZONA_NEUTRA
    assert not d.state.llave_depositada

    # Llega al amarillo de la zona neutra
    d.step(percepcion(), color(ColorLabel.YELLOW), None)
    assert d.state.phase is Phase.DEPOSITAR_LLAVE

    cmd = d.step(percepcion(), None, None)
    assert cmd.gripper is GripperAction.OPEN
    assert d.state.llave_depositada
    assert d.state.phase is Phase.BUSCAR_BANDERA   # recién ahora se habilita


def test_el_borde_de_la_pista_tiene_prioridad_sobre_todo():
    """Sacar dos ruedas pierde la ronda: la evasión pisa cualquier otra cosa."""
    d = DecisionMaker(
        CFG, RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED, llave_depositada=True)
    )
    # Bandera perfectamente centrada y cerca... pero el borde está debajo.
    cmd = d.step(
        percepcion(deteccion(ObjectClass.BANDERA_AZUL, distancia=100.0)),
        None,
        reflect(izq=True, der=True),
    )
    assert cmd.left < 0 and cmd.right < 0          # retrocede
    assert "borde" in cmd.motivo


def test_el_borde_de_un_solo_lado_hace_girar_al_retroceder():
    d = DecisionMaker(CFG, RobotState(phase=Phase.BUSCAR_BANDERA, llave_depositada=True))

    izquierda = d.step(percepcion(), None, reflect(izq=True))
    assert izquierda.left < 0 and izquierda.right < 0
    assert izquierda.left != izquierda.right       # gira mientras retrocede


# ---------------------------------------------------------------------------
# Selección de objetivo
# ---------------------------------------------------------------------------


def test_cada_equipo_persigue_la_bandera_contraria():
    assert RobotState(team=TeamColor.RED).bandera_objetivo is ObjectClass.BANDERA_AZUL
    assert RobotState(team=TeamColor.BLUE).bandera_objetivo is ObjectClass.BANDERA_ROJA


def test_se_ignora_la_bandera_propia():
    d = DecisionMaker(CFG, RobotState(phase=Phase.BUSCAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    # Solo se ve la bandera ROJA, que es la nuestra: hay que seguir buscando.
    d.step(percepcion(deteccion(ObjectClass.BANDERA_ROJA)), None, None)
    assert d.state.phase is Phase.BUSCAR_BANDERA


def test_al_ver_la_bandera_contraria_se_pasa_a_aproximar():
    d = DecisionMaker(CFG, RobotState(phase=Phase.BUSCAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL)), None, None)
    assert d.state.phase is Phase.APROXIMAR_BANDERA


# ---------------------------------------------------------------------------
# Control visual
# ---------------------------------------------------------------------------


def test_el_robot_gira_hacia_el_lado_donde_esta_el_objetivo():
    d = DecisionMaker(CFG, RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))

    derecha = d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL, angulo=20.0)), None, None)
    assert derecha.left > derecha.right      # rueda izquierda más rápida -> gira a la derecha

    d.state = RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED, llave_depositada=True)
    izquierda = d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL, angulo=-20.0)), None, None)
    assert izquierda.right > izquierda.left


def test_se_frena_al_acercarse():
    d = DecisionMaker(CFG, RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    lejos = d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL, distancia=1500.0)), None, None)

    d.state = RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED, llave_depositada=True)
    cerca = d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL, distancia=300.0)), None, None)

    assert cerca.left < lejos.left


def test_las_velocidades_nunca_se_salen_del_rango_del_protocolo():
    """El protocolo manda int8: cualquier valor fuera de -100..100 sería un error."""
    d = DecisionMaker(CFG, RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    for angulo in (-180.0, -90.0, -45.0, 0.0, 45.0, 90.0, 180.0):
        d.state = RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED,
                             llave_depositada=True)
        cmd = d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL, angulo=angulo)), None, None)
        assert -100 <= cmd.left <= 100
        assert -100 <= cmd.right <= 100


def test_se_agarra_solo_cuando_esta_cerca_Y_centrada():
    base = RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED, llave_depositada=True)

    # Cerca pero descentrada: todavía no.
    d = DecisionMaker(CFG, base)
    d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL, distancia=100.0, angulo=25.0)), None, None)
    assert d.state.phase is Phase.APROXIMAR_BANDERA

    # Cerca y centrada: ahora sí.
    d = DecisionMaker(CFG, base)
    d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL, distancia=100.0, angulo=0.5)), None, None)
    assert d.state.phase is Phase.AGARRAR_BANDERA


def test_el_tof_manda_sobre_la_vision_cuando_esta_centrada():
    """El ToF es una medición física directa: si dice que ya se llegó, se cierra."""
    d = DecisionMaker(CFG, RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    # La cámara todavía la ve lejos, pero el ToF (centrada) dice que ya se llegó.
    d.step(
        percepcion(deteccion(ObjectClass.BANDERA_AZUL, distancia=800.0, angulo=0.5)),
        None,
        None,
        tof(50),
    )
    assert d.state.phase is Phase.AGARRAR_BANDERA


def test_el_tof_se_ignora_si_no_esta_centrada():
    """Un ToF de un solo punto, sin centrar, puede estar midiendo cualquier cosa."""
    d = DecisionMaker(CFG, RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    d.step(
        percepcion(deteccion(ObjectClass.BANDERA_AZUL, distancia=800.0, angulo=25.0)),
        None,
        None,
        tof(50),
    )
    assert d.state.phase is Phase.APROXIMAR_BANDERA


def test_el_tof_invalido_no_bloquea_el_agarre_por_vision():
    """Sensor caído o sin lectura confiable: se sigue con la estimación visual."""
    d = DecisionMaker(CFG, RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    d.step(
        percepcion(deteccion(ObjectClass.BANDERA_AZUL, distancia=100.0, angulo=0.5)),
        None,
        None,
        tof(50, valido=False),
    )
    assert d.state.phase is Phase.AGARRAR_BANDERA


def test_perder_la_bandera_un_instante_no_reinicia_la_busqueda():
    """Un parpadeo del tracker no debe hacer que el robot se rinda."""
    d = DecisionMaker(CFG, RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    for _ in range(5):
        d.step(percepcion(), None, None)          # sin detecciones
    assert d.state.phase is Phase.APROXIMAR_BANDERA

    for _ in range(10):
        d.step(percepcion(), None, None)
    assert d.state.phase is Phase.BUSCAR_BANDERA  # perdida sostenida: a buscar


def test_al_llegar_a_la_zona_propia_se_suelta_la_bandera():
    d = DecisionMaker(CFG, RobotState(phase=Phase.RETORNAR_A_ZONA, team=TeamColor.RED,
                                      llave_depositada=True, bandera_capturada=True))
    d.step(percepcion(), color(ColorLabel.RED), None)     # equipo rojo ve su línea roja
    assert d.state.phase is Phase.ENTREGAR

    cmd = d.step(percepcion(), None, None)
    assert cmd.gripper is GripperAction.OPEN
    assert d.state.phase is Phase.TERMINADO


def test_la_busqueda_alterna_de_sentido_para_no_dar_vueltas_siempre_igual():
    d = DecisionMaker(CFG, RobotState(phase=Phase.BUSCAR_BANDERA, llave_depositada=True))
    sentido_inicial = d.state.sentido_busqueda
    for _ in range(90):
        d.step(percepcion(), None, None)
    assert d.state.sentido_busqueda == -sentido_inicial
