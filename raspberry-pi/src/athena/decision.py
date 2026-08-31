"""Capa de decisión: de lo que se ve, a lo que se le manda al ESP32.

Esta es la respuesta a "¿y en base a eso qué le mando al ESP32-S3?".

ORDEN DE PRIORIDADES (de mayor a menor)
---------------------------------------
1. **No salirse de la pista.** El reglamento dice que sacar dos ruedas fuera
   pierde la ronda de inmediato. Es la forma más tonta de perder, así que la
   evasión del borde negro tiene prioridad absoluta y pisa cualquier otra
   decisión, incluso si la bandera está justo delante.
2. **No adelantar la secuencia.** Buscar la bandera antes de depositar la
   llave también pierde la ronda de inmediato. La máquina de estados no puede
   pasar a buscar hasta que la llave esté depositada, y esa condición está
   escrita en un solo sitio para que no se pueda saltar por accidente.
3. **Cumplir la misión**: llave → zona neutra, bandera → zona propia.

Todo el archivo está escrito como una función pura de
``(percepción, telemetría, estado) -> comandos``. Sin efectos secundarios, sin
tocar el puerto serial: así se puede probar entera sin robot, que es lo que
hace ``tests/test_decision.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum, auto

from .config import ControlConfig
from .protocol import (
    ColorLabel,
    ColorTelemetry,
    GripperAction,
    ReflectTelemetry,
    TeamColor,
    ToFTelemetry,
)
from .types import Detection, ObjectClass, Perception


class Phase(Enum):
    """Fases de la ronda, en el orden que exige el reglamento."""

    INICIO = auto()
    BUSCAR_ZONA_NEUTRA = auto()   # llevar la llave al cuadro amarillo
    DEPOSITAR_LLAVE = auto()
    BUSCAR_BANDERA = auto()       # solo permitido tras depositar la llave
    APROXIMAR_BANDERA = auto()
    AGARRAR_BANDERA = auto()
    RETORNAR_A_ZONA = auto()
    ENTREGAR = auto()
    TERMINADO = auto()


@dataclass(frozen=True)
class RobotState:
    phase: Phase = Phase.INICIO
    team: TeamColor = TeamColor.RED
    llave_depositada: bool = False
    bandera_capturada: bool = False
    frames_sin_objetivo: int = 0
    sentido_busqueda: int = 1        # 1 = giro a la derecha, -1 = a la izquierda

    @property
    def bandera_objetivo(self) -> ObjectClass:
        """La bandera que hay que capturar es la del equipo CONTRARIO."""
        return (
            ObjectClass.BANDERA_AZUL
            if self.team is TeamColor.RED
            else ObjectClass.BANDERA_ROJA
        )

    @property
    def color_zona_propia(self) -> ColorLabel:
        return ColorLabel.RED if self.team is TeamColor.RED else ColorLabel.BLUE


@dataclass(frozen=True)
class Commands:
    """Lo que se le manda al ESP32 en este ciclo."""

    left: int = 0
    right: int = 0
    gripper: GripperAction | None = None
    motivo: str = ""                  # para el log; no viaja por el cable

    @property
    def parado(self) -> bool:
        return self.left == 0 and self.right == 0


class DecisionMaker:
    def __init__(self, cfg: ControlConfig, state: RobotState | None = None) -> None:
        self._cfg = cfg
        self.state = state or RobotState()

    # -----------------------------------------------------------------------

    def step(
        self,
        perception: Perception,
        color: ColorTelemetry | None,
        reflect: ReflectTelemetry | None,
        tof: ToFTelemetry | None = None,
    ) -> Commands:
        """Decide los comandos de este ciclo y avanza la máquina de estados."""

        # --- Prioridad 1: no salirse de la pista ---------------------------
        evasion = self._evadir_borde(reflect)
        if evasion is not None:
            return evasion

        # --- Prioridad 2 y 3: la fase que toque ----------------------------
        phase = self.state.phase

        if phase is Phase.INICIO:
            # Se arranca con la llave ya sujeta y levantada, listo para moverse.
            self.state = replace(self.state, phase=Phase.BUSCAR_ZONA_NEUTRA)
            return Commands(gripper=GripperAction.CLOSE, motivo="asegurar la llave")

        if phase is Phase.BUSCAR_ZONA_NEUTRA:
            return self._buscar_zona_neutra(color)

        if phase is Phase.DEPOSITAR_LLAVE:
            return self._depositar_llave()

        if phase is Phase.BUSCAR_BANDERA:
            return self._buscar_bandera(perception)

        if phase is Phase.APROXIMAR_BANDERA:
            return self._aproximar_bandera(perception, tof)

        if phase is Phase.AGARRAR_BANDERA:
            return self._agarrar_bandera()

        if phase is Phase.RETORNAR_A_ZONA:
            return self._retornar(color)

        if phase is Phase.ENTREGAR:
            self.state = replace(self.state, phase=Phase.TERMINADO)
            return Commands(gripper=GripperAction.OPEN, motivo="soltar la bandera en zona propia")

        return Commands(motivo="ronda terminada")

    # -- prioridad 1 --------------------------------------------------------

    def _evadir_borde(self, reflect: ReflectTelemetry | None) -> Commands | None:
        """Retrocede si algún sensor de reflectancia ve la cinta negra del borde.

        Recordatorio de cómo lee el QTR: valor ADC alto = superficie oscura.
        El firmware ya hace esa conversión y nos entrega los booleanos.
        """
        if reflect is None:
            return None

        v = self._cfg.velocidad_aproximacion

        if reflect.left_on_line and reflect.right_on_line:
            # Borde de frente: marcha atrás recta.
            return Commands(-v, -v, motivo="borde al frente, retrocediendo")

        if reflect.left_on_line:
            # Borde a la izquierda: retroceder girando hacia la derecha.
            return Commands(-v, -v // 3, motivo="borde a la izquierda")

        if reflect.right_on_line:
            return Commands(-v // 3, -v, motivo="borde a la derecha")

        return None

    # -- fases de la llave --------------------------------------------------

    def _buscar_zona_neutra(self, color: ColorTelemetry | None) -> Commands:
        """Avanza hasta que el sensor delantero vea el amarillo de la zona neutra."""
        if color is not None and color.front_valid and color.front is ColorLabel.YELLOW:
            self.state = replace(self.state, phase=Phase.DEPOSITAR_LLAVE)
            return Commands(motivo="zona neutra alcanzada")

        # Sin más información que el color del piso, se avanza en línea recta.
        # TODO: cuando la cámara reconozca la zona amarilla, guiarse con ella
        #       en vez de ir a ciegas. Por ahora el sensor de color es quien
        #       manda, y funciona porque la zona neutra está en el centro.
        v = self._cfg.velocidad_crucero
        return Commands(v, v, motivo="avanzando hacia la zona neutra")

    def _depositar_llave(self) -> Commands:
        """Baja el gripper, suelta la llave y solo entonces habilita la búsqueda.

        Este es el único sitio donde ``llave_depositada`` pasa a True. Mientras
        sea False, ``_buscar_bandera`` no puede ejecutarse: así el error que
        descalifica de inmediato es imposible por construcción, no por
        disciplina de quien lea el código.
        """
        self.state = replace(
            self.state, phase=Phase.BUSCAR_BANDERA, llave_depositada=True, frames_sin_objetivo=0
        )
        return Commands(gripper=GripperAction.OPEN, motivo="llave depositada en zona neutra")

    # -- fases de la bandera ------------------------------------------------

    def _buscar_bandera(self, perception: Perception) -> Commands:
        # Cinturón de seguridad: si algún día alguien reordena las fases, esto
        # evita que se busque la bandera antes de tiempo y se pierda la ronda.
        if not self.state.llave_depositada:
            self.state = replace(self.state, phase=Phase.BUSCAR_ZONA_NEUTRA)
            return Commands(motivo="la llave todavía no está depositada")

        objetivo = perception.best(self.state.bandera_objetivo)
        if objetivo is not None:
            self.state = replace(
                self.state, phase=Phase.APROXIMAR_BANDERA, frames_sin_objetivo=0
            )
            return self._perseguir(objetivo)

        # Girar sobre el propio eje barriendo el campo. Se invierte el sentido
        # cada ~3 segundos para no quedarse dando vueltas siempre hacia el
        # mismo lado: si la bandera quedo justo detras, alternar la encuentra antes.
        v = self._cfg.velocidad_busqueda
        frames = self.state.frames_sin_objetivo + 1
        sentido = self.state.sentido_busqueda
        if frames % 90 == 0:            # ~3 s a 30 FPS
            sentido = -sentido
        self.state = replace(self.state, frames_sin_objetivo=frames, sentido_busqueda=sentido)
        return Commands(v * sentido, -v * sentido, motivo="buscando la bandera")

    def _aproximar_bandera(
        self, perception: Perception, tof: ToFTelemetry | None
    ) -> Commands:
        objetivo = perception.best(self.state.bandera_objetivo)

        if objetivo is None:
            # Perderla un frame suelto es normal (un parpadeo del tracker).
            # Solo se vuelve a buscar si se pierde de forma sostenida.
            frames = self.state.frames_sin_objetivo + 1
            self.state = replace(self.state, frames_sin_objetivo=frames)
            if frames > 10:
                self.state = replace(
                    self.state, phase=Phase.BUSCAR_BANDERA, frames_sin_objetivo=0
                )
                return Commands(motivo="objetivo perdido, volviendo a buscar")
            return Commands(motivo="objetivo perdido momentáneamente")

        self.state = replace(self.state, frames_sin_objetivo=0)

        centrado = abs(objetivo.angle_deg) < self._cfg.angulo_muerto_deg

        # El VL53L1X mide en línea recta frente al gripper, así que su
        # lectura solo describe a LA BANDERA cuando además está centrada; si
        # no, el sensor está midiendo cualquier otra cosa que tenga delante
        # (el piso, el aire) y hay que confiar en la estimación de la cámara.
        # Cuando aplica, es la fuente de verdad: es una medición física
        # directa, mucho más confiable a corta distancia que estimar el
        # tamaño aparente de la bandera en el frame.
        if tof is not None and tof.valid and centrado:
            distancia: float | None = float(tof.distance_mm)
        else:
            distancia = objetivo.distance_mm

        if distancia is not None and distancia <= self._cfg.distancia_agarre_mm and centrado:
            self.state = replace(self.state, phase=Phase.AGARRAR_BANDERA)
            return Commands(motivo="bandera al alcance")

        return self._perseguir(objetivo)

    def _agarrar_bandera(self) -> Commands:
        self.state = replace(
            self.state, phase=Phase.RETORNAR_A_ZONA, bandera_capturada=True
        )
        return Commands(gripper=GripperAction.CLOSE, motivo="cerrando la pinza sobre la bandera")

    def _retornar(self, color: ColorTelemetry | None) -> Commands:
        """Vuelve a la zona propia. La línea de color del piso avisa al llegar."""
        if color is not None and color.front_valid and color.front is self.state.color_zona_propia:
            self.state = replace(self.state, phase=Phase.ENTREGAR)
            return Commands(motivo="zona propia alcanzada")

        # TODO: aquí hace falta odometría o una referencia visual para saber
        # LIMITACION CONOCIDA: con los sensores actuales el robot no sabe hacia
        # donde queda su zona, solo reconoce la linea cuando ya la pisa. Avanzar
        # recto funciona si quedo orientado hacia su lado, pero no se recupera si
        # quedo girado. TODO: resolverlo con odometria (encoders) o reconociendo
        # visualmente la linea de color de la zona propia. Es el hueco mas grande
        # que queda en la logica de mision.
        v = self._cfg.velocidad_crucero
        return Commands(v, v, motivo="regresando a la zona propia")

    # -- control visual -----------------------------------------------------

    def _perseguir(self, objetivo: Detection) -> Commands:
        """Control proporcional sobre el ángulo: gira hacia el objetivo y avanza.

        Es un control P puro, sin término integral ni derivativo. A propósito:
        el objetivo está quieto y el lazo corre a 30 Hz, así que un P bien
        ajustado basta. Meterle un PID completo aquí sería añadir dos ganancias
        más que calibrar a cambio de nada.
        """
        cfg = self._cfg
        correccion = int(cfg.kp_angulo * objetivo.angle_deg)
        correccion = max(-cfg.correccion_max, min(cfg.correccion_max, correccion))

        # Cerca del objetivo se baja la velocidad: llegar rápido y pasarse de
        # largo, tumbando la bandera, es peor que llegar despacio.
        distancia = objetivo.distance_mm
        if distancia is not None and distancia < 400.0:
            base = cfg.velocidad_aproximacion
        else:
            base = cfg.velocidad_crucero

        left = max(-100, min(100, base + correccion))
        right = max(-100, min(100, base - correccion))
        return Commands(left, right, motivo=f"persiguiendo {objetivo.cls.value}")
