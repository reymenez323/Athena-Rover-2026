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
)
from athena.types import BBox, Detection, ObjectClass, Perception  # noqa: E402

CFG = ControlConfig()


def percepcion(*detecciones: Detection) -> Perception:
    return Perception(frame_id=1, timestamp=0.0, detections=tuple(detecciones))


def deteccion(
    cls: ObjectClass, *, angulo: float = 0.0, distancia: float | None = 800.0,
    area: float | None = 0.1,
) -> Detection:
    """``area`` es la fracción del frame (ver ``Detection.area_fraccion``) --

    0.1 por defecto: chica, para no disparar el agarre sin que un test lo
    pida a propósito con ``area=`` (ver ``deteccion_al_alcance``).
    """
    return Detection(
        cls=cls,
        box=BBox(150, 100, 20, 60),
        confidence=0.9,
        distance_mm=distancia,
        area_fraccion=area,
        angle_deg=angulo,
        track_id=1,
    )


def deteccion_al_alcance(cls: ObjectClass, *, angulo: float = 0.0) -> Detection:
    """Detección que SÍ cumple el umbral de agarre (``area_agarre_fraccion``)."""
    return deteccion(cls, angulo=angulo, area=CFG.area_agarre_fraccion + 0.1)


def color(back: ColorLabel) -> ColorTelemetry:
    return ColorTelemetry(timestamp_ms=0, front=ColorLabel.UNKNOWN, back=back,
                          front_valid=False, back_valid=True)


def reflect(izq: bool = False, der: bool = False) -> ReflectTelemetry:
    return ReflectTelemetry(timestamp_ms=0, left_raw=0, right_raw=0,
                            left_on_line=izq, right_on_line=der)


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


def _avanzar(d: DecisionMaker, n: int, *, color_: ColorTelemetry | None = None) -> None:
    """Llama a ``step`` ``n`` veces seguidas, para cruzar un delay de fase."""
    for _ in range(n):
        d.step(percepcion(), color_, None)


def test_la_secuencia_completa_respeta_el_orden_del_reglamento():
    d = DecisionMaker(CFG, RobotState(team=TeamColor.RED))
    assert d.state.phase is Phase.INICIO

    # INICIO: el primer cuadro manda CLOSE_LLAVE y el resto es el delay de
    # asentamiento del servo -- no se avanza a BUSCAR_ZONA_NEUTRA de un tirón.
    cmd = d.step(percepcion(), None, None)
    assert cmd.gripper is GripperAction.CLOSE_LLAVE
    assert d.state.phase is Phase.INICIO
    _avanzar(d, CFG.frames_asentamiento_gripper - 1)
    assert d.state.phase is Phase.BUSCAR_ZONA_NEUTRA
    assert not d.state.llave_depositada

    # Llega al amarillo de la zona neutra -- debounce: hace falta sostenerlo
    # frames_debounce_deteccion cuadros seguidos, no uno solo.
    _avanzar(d, CFG.frames_debounce_deteccion, color_=color(ColorLabel.YELLOW))
    assert d.state.phase is Phase.DEPOSITAR_LLAVE

    # DEPOSITAR_LLAVE: igual que INICIO, primer cuadro abre y suelta la
    # llave; el resto es el delay antes de maniobrar para no arrastrarla.
    cmd = d.step(percepcion(), None, None)
    assert cmd.gripper is GripperAction.OPEN
    assert d.state.llave_depositada          # ya se habilita _buscar_bandera...
    assert d.state.phase is Phase.DEPOSITAR_LLAVE  # ...pero todavía no se pasa a buscarla
    _avanzar(d, CFG.frames_asentamiento_gripper - 1)
    assert d.state.phase is Phase.EVADIR_LLAVE

    # EVADIR_LLAVE: retrocede, gira a la derecha, gira a la izquierda, y
    # solo AHORA se habilita la búsqueda de la bandera. +1 porque la
    # transición ocurre recién en el cuadro SIGUIENTE al último de la
    # maniobra (ver el "else" implícito al final de ``_evadir_llave``).
    total_evasion = CFG.frames_retroceso_evasion + 2 * CFG.frames_giro_evasion + 1
    _avanzar(d, total_evasion)
    assert d.state.phase is Phase.BUSCAR_BANDERA   # recién ahora se habilita


# ---------------------------------------------------------------------------
# Maniobra de evasión de la llave y giro de retorno (PDF de lógica)
# ---------------------------------------------------------------------------


def test_evadir_llave_retrocede_luego_gira_derecha_luego_izquierda():
    """La maniobra pedida en el PDF: retroceder, doblar derecha, doblar izq."""
    d = DecisionMaker(CFG, RobotState(phase=Phase.EVADIR_LLAVE, llave_depositada=True))

    cmd = d.step(percepcion(), None, None)
    assert cmd.left < 0 and cmd.right < 0 and cmd.left == cmd.right   # retrocede recto

    _avanzar(d, CFG.frames_retroceso_evasion - 1)
    cmd = d.step(percepcion(), None, None)
    assert cmd.left > 0 and cmd.right > 0 and cmd.left > cmd.right    # avanza girando a la derecha

    _avanzar(d, CFG.frames_giro_evasion - 1)
    cmd = d.step(percepcion(), None, None)
    assert cmd.left > 0 and cmd.right > 0 and cmd.right > cmd.left    # corrige hacia la izquierda

    # +1 extra cuadro: la transición ocurre recién cuando se SUPERA el total
    # de la maniobra (ver el "else" implícito al final de ``_evadir_llave``).
    _avanzar(d, CFG.frames_giro_evasion)
    assert d.state.phase is Phase.BUSCAR_BANDERA


def test_agarrar_bandera_espera_el_gripper_antes_de_girar():
    d = DecisionMaker(CFG, RobotState(phase=Phase.AGARRAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    cmd = d.step(percepcion(), None, None)
    assert cmd.gripper is GripperAction.CLOSE_BANDERA
    assert d.state.bandera_capturada
    assert d.state.phase is Phase.AGARRAR_BANDERA   # todavía asentando

    _avanzar(d, CFG.frames_asentamiento_gripper - 1)
    assert d.state.phase is Phase.GIRO_RETORNO


def test_giro_retorno_gira_a_tiempo_fijo_y_pasa_a_retornar():
    d = DecisionMaker(CFG, RobotState(phase=Phase.GIRO_RETORNO, team=TeamColor.RED,
                                      llave_depositada=True, bandera_capturada=True))
    cmd = d.step(percepcion(), None, None)
    assert cmd.left != cmd.right                    # está girando, no avanzando recto
    assert d.state.phase is Phase.GIRO_RETORNO

    _avanzar(d, CFG.frames_giro_retorno - 1)
    assert d.state.phase is Phase.RETORNAR_A_ZONA


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


def test_se_agarra_solo_cuando_ocupa_suficiente_area_Y_esta_centrada():
    """Sin ToF: lo que decide el agarre es cuánto del frame ocupa la caja."""
    base = RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED, llave_depositada=True)

    # Ocupa lo suficiente pero descentrada: nunca, sin importar cuánto se repita.
    d = DecisionMaker(CFG, base)
    for _ in range(CFG.frames_debounce_deteccion + 2):
        d.step(percepcion(deteccion_al_alcance(ObjectClass.BANDERA_AZUL, angulo=25.0)), None, None)
    assert d.state.phase is Phase.APROXIMAR_BANDERA

    # Chica (lejos) pero centrada: tampoco.
    d = DecisionMaker(CFG, base)
    for _ in range(CFG.frames_debounce_deteccion + 2):
        d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL, angulo=0.5, area=0.1)), None, None)
    assert d.state.phase is Phase.APROXIMAR_BANDERA

    # Ocupa lo suficiente Y centrada, sostenido el debounce: ahora sí.
    d = DecisionMaker(CFG, base)
    for _ in range(CFG.frames_debounce_deteccion):
        d.step(percepcion(deteccion_al_alcance(ObjectClass.BANDERA_AZUL, angulo=0.5)), None, None)
    assert d.state.phase is Phase.AGARRAR_BANDERA


def test_el_agarre_necesita_varios_cuadros_seguidos_al_alcance():
    """Debounce: una sola lectura con la caja grande no alcanza para cerrar la pinza."""
    d = DecisionMaker(CFG, RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    assert CFG.frames_debounce_deteccion >= 2   # si esto cambia, el test de abajo no prueba nada

    d.step(percepcion(deteccion_al_alcance(ObjectClass.BANDERA_AZUL)), None, None)
    assert d.state.phase is Phase.APROXIMAR_BANDERA   # todavía no, falta sostenerlo

    for _ in range(CFG.frames_debounce_deteccion - 1):
        d.step(percepcion(deteccion_al_alcance(ObjectClass.BANDERA_AZUL)), None, None)
    assert d.state.phase is Phase.AGARRAR_BANDERA


def test_el_debounce_del_agarre_se_reinicia_si_se_deja_de_cumplir():
    d = DecisionMaker(CFG, RobotState(phase=Phase.APROXIMAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    d.step(percepcion(deteccion_al_alcance(ObjectClass.BANDERA_AZUL)), None, None)
    # Un cuadro donde deja de estar al alcance reinicia el conteo a 0.
    d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL, area=0.1)), None, None)
    d.step(percepcion(deteccion_al_alcance(ObjectClass.BANDERA_AZUL)), None, None)
    assert d.state.phase is Phase.APROXIMAR_BANDERA   # va en 1 cuadro consecutivo, no en 2


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
    # equipo rojo ve su línea roja, sostenida el debounce -- ver el test de arriba
    _avanzar(d, CFG.frames_debounce_deteccion, color_=color(ColorLabel.RED))
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


# ---------------------------------------------------------------------------
# Señalizar la bandera del oponente (reto de la demostración)
# ---------------------------------------------------------------------------
#
# El ESP32 no tiene cámara: esta señal es la única forma de que sepa que hay
# algo que anunciar con el LED. Va en Commands.bandera_a_la_vista y sale al
# cable como CMD_FLAG_SIGNAL (ver run_rover.py).


def test_ver_la_bandera_contraria_levanta_la_senal():
    d = DecisionMaker(CFG, RobotState(phase=Phase.BUSCAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    cmd = d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL)), None, None)
    assert cmd.bandera_a_la_vista is True


def test_sin_bandera_a_la_vista_la_senal_queda_baja():
    d = DecisionMaker(CFG, RobotState(phase=Phase.BUSCAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    cmd = d.step(percepcion(), None, None)
    assert cmd.bandera_a_la_vista is False


def test_la_bandera_propia_no_levanta_la_senal():
    """Solo cuenta la del oponente: ver la propia no es detectar nada."""
    d = DecisionMaker(CFG, RobotState(phase=Phase.BUSCAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    cmd = d.step(percepcion(deteccion(ObjectClass.BANDERA_ROJA)), None, None)
    assert cmd.bandera_a_la_vista is False


def test_la_senal_se_levanta_aunque_la_llave_no_este_depositada():
    """Ver la bandera no es buscarla, y el LED tiene que poder anunciarlo.

    Lo que descalifica según el reglamento es IR a buscar la bandera antes de
    depositar la llave. Este test fija las dos mitades de esa distinción: la
    señal sube, pero la fase NO avanza hacia la búsqueda.
    """
    d = DecisionMaker(CFG, RobotState(phase=Phase.BUSCAR_ZONA_NEUTRA, team=TeamColor.RED))
    cmd = d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL)), color(ColorLabel.FLOOR), None)

    assert cmd.bandera_a_la_vista is True
    assert d.state.phase is Phase.BUSCAR_ZONA_NEUTRA
    assert d.state.llave_depositada is False


def test_la_senal_sobrevive_a_la_evasion_de_borde():
    """La evasión corta el paso a todo lo demás; la señal no es 'lo demás'.

    Se calcula fuera de la máquina de estados justamente para esto: el robot
    puede estar retrocediendo del borde y seguir anunciando que ve la bandera.
    """
    d = DecisionMaker(CFG, RobotState(phase=Phase.BUSCAR_BANDERA, team=TeamColor.RED,
                                      llave_depositada=True))
    cmd = d.step(percepcion(deteccion(ObjectClass.BANDERA_AZUL)), None, reflect(izq=True, der=True))

    assert cmd.left < 0 and cmd.right < 0          # sí, está retrocediendo
    assert cmd.bandera_a_la_vista is True           # y sí, sigue señalizando
