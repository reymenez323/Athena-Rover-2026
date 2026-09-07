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
)
from .types import Detection, ObjectClass, Perception


class Phase(Enum):
    """Fases de la ronda, en el orden que exige el reglamento."""

    INICIO = auto()
    BUSCAR_ZONA_NEUTRA = auto()   # llevar la llave al cuadro amarillo
    DEPOSITAR_LLAVE = auto()
    EVADIR_LLAVE = auto()         # retroceder y esquivarla para no arrastrarla
    BUSCAR_BANDERA = auto()       # solo permitido tras depositar la llave
    APROXIMAR_BANDERA = auto()
    AGARRAR_BANDERA = auto()
    GIRO_RETORNO = auto()         # giro a tiempo fijo antes de buscar la zona propia
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
    # Cuadros transcurridos DENTRO de la fase actual. Se reinicia a 0 cada vez
    # que la fase cambia (ver DecisionMaker._ir_a_fase) y sirve de reloj
    # aproximado para las fases con un delay o una maniobra a tiempo fijo
    # (asentar el gripper, esquivar la llave, girar al iniciar el regreso).
    frames_en_fase: int = 0
    # Debounce (ControlConfig.frames_debounce_deteccion): cuadros SEGUIDOS
    # que lleva sosteniéndose una detección de color (zona neutra alcanzada
    # en _buscar_zona_neutra, zona propia alcanzada en _retornar -- nunca
    # las dos a la vez, así que comparten un solo contador). Se reinicia a
    # 0 en cuanto la lectura deja de cumplir la condición, no solo al
    # cambiar de fase.
    frames_color_objetivo: int = 0
    # Mismo mecanismo, para "la bandera está centrada y ocupa suficiente
    # fracción del frame" en _aproximar_bandera. Contador aparte porque no
    # comparte fase de vida con frames_color_objetivo.
    frames_agarre: int = 0

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
    # ¿La cámara ve AHORA la bandera contraria? Viaja al ESP32 como
    # CMD_FLAG_SIGNAL y es lo que hace destellar el LED: el ESP32 no tiene
    # cámara, así que sin esto no hay forma de cumplir el reto de
    # demostración "detectar la bandera del oponente y señalizar su
    # detección". Se marca en cuanto se ve, sin esperar a ninguna fase: lo
    # que descalifica según el reglamento es IR a buscar la bandera antes de
    # depositar la llave, no verla de pasada — y de ir o no ir se encarga la
    # máquina de estados, no este campo.
    bandera_a_la_vista: bool = False
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
    ) -> Commands:
        """Decide los comandos de este ciclo y avanza la máquina de estados."""

        # La señal de "veo la bandera contraria" es independiente de la fase y
        # de la evasión de borde: se calcula una sola vez aquí y se pega a los
        # comandos que salgan, sea cual sea el camino que tomen abajo.
        a_la_vista = perception.best(self.state.bandera_objetivo) is not None
        return replace(self._decidir(perception, color, reflect),
                       bandera_a_la_vista=a_la_vista)

    def _ir_a_fase(self, phase: Phase, **kwargs) -> None:
        """Cambia de fase y reinicia el reloj (``frames_en_fase``) de la nueva.

        Único punto de la máquina de estados que cambia ``phase``: así
        ``frames_en_fase`` nunca puede quedar desincronizado de en qué fase
        se está.
        """
        self.state = replace(self.state, phase=phase, frames_en_fase=0, **kwargs)

    def _decidir(
        self,
        perception: Perception,
        color: ColorTelemetry | None,
        reflect: ReflectTelemetry | None,
    ) -> Commands:
        # --- Prioridad 1: no salirse de la pista ---------------------------
        evasion = self._evadir_borde(reflect)
        if evasion is not None:
            return evasion

        # El reloj de la fase actual solo avanza cuando de verdad se ejecuta
        # lógica de esa fase -- si el robot pasó este cuadro evadiendo el
        # borde, un delay de asentamiento del gripper o una maniobra a
        # tiempo fijo no deberían "gastar" ese cuadro.
        self.state = replace(self.state, frames_en_fase=self.state.frames_en_fase + 1)

        # --- Prioridad 2 y 3: la fase que toque ----------------------------
        phase = self.state.phase

        if phase is Phase.INICIO:
            return self._inicio()

        if phase is Phase.BUSCAR_ZONA_NEUTRA:
            return self._buscar_zona_neutra(color)

        if phase is Phase.DEPOSITAR_LLAVE:
            return self._depositar_llave()

        if phase is Phase.EVADIR_LLAVE:
            return self._evadir_llave()

        if phase is Phase.BUSCAR_BANDERA:
            return self._buscar_bandera(perception)

        if phase is Phase.APROXIMAR_BANDERA:
            return self._aproximar_bandera(perception)

        if phase is Phase.AGARRAR_BANDERA:
            return self._agarrar_bandera()

        if phase is Phase.GIRO_RETORNO:
            return self._giro_retorno()

        if phase is Phase.RETORNAR_A_ZONA:
            return self._retornar(color)

        if phase is Phase.ENTREGAR:
            self._ir_a_fase(Phase.TERMINADO)
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

    def _inicio(self) -> Commands:
        """Asegura la llave (ya sujeta y levantada) y espera a que el servo llegue.

        El PDF de lógica pide "un pequeño delay" tras agarrar la caja antes de
        seguir a la siguiente acción -- sin esto, ``BUSCAR_ZONA_NEUTRA`` podía
        empezar a mover el robot en el mismo instante en que se mandaba
        ``CLOSE_LLAVE``, sin que el servo hubiera alcanzado a cerrar de verdad.
        """
        if self.state.frames_en_fase == 1:
            return Commands(gripper=GripperAction.CLOSE_LLAVE, motivo="asegurar la llave")
        if self.state.frames_en_fase >= self._cfg.frames_asentamiento_gripper:
            self._ir_a_fase(Phase.BUSCAR_ZONA_NEUTRA)
            return Commands(motivo="llave asegurada, saliendo a buscar la zona neutra")
        return Commands(motivo="asentando el cierre de la pinza sobre la llave")

    # -- fases de la llave --------------------------------------------------

    def _buscar_zona_neutra(self, color: ColorTelemetry | None) -> Commands:
        """Avanza hasta que el sensor TRASERO vea el amarillo de la zona neutra.

        Sensor TRASERO, no delantero: el TCS34725 delantero se retiró junto
        con el bus I2C nº0 del ESP32 (ver el aviso grande en
        firmware-esp32/src/main.cpp) -- el trasero es ahora el único que
        queda. Sus umbrales de clasificación solo se calibraron contra el
        delantero (ver ``ClassifyColor`` en el firmware), un supuesto sin
        verificar que ahora es obligatorio, no un respaldo.

        Debounce (``frames_debounce_deteccion``): el amarillo tiene que
        sostenerse ese número de cuadros SEGUIDOS antes de depositar la
        llave -- una sola lectura con ruido ya no alcanza, y cualquier
        lectura distinta reinicia el conteo.
        """
        ve_amarillo = color is not None and color.back_valid and color.back is ColorLabel.YELLOW
        if ve_amarillo:
            frames = self.state.frames_color_objetivo + 1
            self.state = replace(self.state, frames_color_objetivo=frames)
            if frames >= self._cfg.frames_debounce_deteccion:
                # frames_color_objetivo=0: se reutiliza este mismo contador
                # más adelante en _retornar (zona propia) -- dejarlo en lo
                # que sea que haya llegado a valer aquí haría que ese
                # debounce arrancara ya a mitad de camino.
                self._ir_a_fase(Phase.DEPOSITAR_LLAVE, frames_color_objetivo=0)
                return Commands(motivo="zona neutra alcanzada")
        else:
            self.state = replace(self.state, frames_color_objetivo=0)

        # Sin más información que el color del piso, se avanza en línea recta.
        # TODO: cuando la cámara reconozca la zona amarilla, guiarse con ella
        #       en vez de ir a ciegas. Por ahora el sensor de color es quien
        #       manda, y funciona porque la zona neutra está en el centro.
        v = self._cfg.velocidad_crucero
        return Commands(v, v, motivo="avanzando hacia la zona neutra")

    def _depositar_llave(self) -> Commands:
        """Abre el gripper, espera a que el servo llegue y solo entonces evade.

        ``llave_depositada`` pasa a True apenas se manda soltar: mientras sea
        False, ``_buscar_bandera`` no puede ejecutarse (ver su cinturón de
        seguridad), así que el error que descalifica de inmediato según el
        reglamento (buscar la bandera antes de depositar la llave) sigue
        siendo imposible por construcción, no por disciplina de quien lea el
        código -- eso no cambia por agregar el delay de asentamiento.
        """
        if self.state.frames_en_fase == 1:
            self.state = replace(self.state, llave_depositada=True)
            return Commands(gripper=GripperAction.OPEN, motivo="soltando la llave en la zona neutra")
        if self.state.frames_en_fase >= self._cfg.frames_asentamiento_gripper:
            self._ir_a_fase(Phase.EVADIR_LLAVE)
            return Commands(motivo="llave depositada, maniobrando para no arrastrarla")
        return Commands(motivo="asentando la apertura de la pinza")

    def _evadir_llave(self) -> Commands:
        """Retrocede y esquiva la llave recién depositada antes de seguir.

        Maniobra pedida explícitamente en el PDF de lógica (y dibujada en el
        plano de la pista): seguir de frente arrastraría la caja recién
        depositada. Es a tiempo fijo, sin sensores -- ``frames_retroceso_
        evasion``/``frames_giro_evasion`` en ``ControlConfig``, A CALIBRAR EN
        CANCHA según el tamaño real de la caja y dónde quede el robot al
        detenerse.
        """
        f = self.state.frames_en_fase
        v = self._cfg.velocidad_aproximacion
        t1 = self._cfg.frames_retroceso_evasion
        t2 = t1 + self._cfg.frames_giro_evasion
        t3 = t2 + self._cfg.frames_giro_evasion

        if f <= t1:
            return Commands(-v, -v, motivo="retrocediendo para no arrastrar la llave")
        if f <= t2:
            # Avanza girando a la derecha (izquierda más rápida) para
            # rodear la caja por su lado derecho -- ver el plano de la pista.
            return Commands(v, v // 3, motivo="esquivando la llave hacia la derecha")
        if f <= t3:
            # Corrige de vuelta hacia la izquierda para volver a quedar de
            # frente a la zona del contrincante.
            return Commands(v // 3, v, motivo="reposicionándose de frente tras esquivar la llave")

        self._ir_a_fase(Phase.BUSCAR_BANDERA)
        return Commands(motivo="maniobra de evasión completada, buscando la bandera")

    # -- fases de la bandera ------------------------------------------------

    def _buscar_bandera(self, perception: Perception) -> Commands:
        # Cinturón de seguridad: si algún día alguien reordena las fases, esto
        # evita que se busque la bandera antes de tiempo y se pierda la ronda.
        if not self.state.llave_depositada:
            self._ir_a_fase(Phase.BUSCAR_ZONA_NEUTRA)
            return Commands(motivo="la llave todavía no está depositada")

        objetivo = perception.best(self.state.bandera_objetivo)
        if objetivo is not None:
            self._ir_a_fase(Phase.APROXIMAR_BANDERA, frames_sin_objetivo=0, frames_agarre=0)
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

    def _aproximar_bandera(self, perception: Perception) -> Commands:
        """Persigue la bandera hasta que ocupe suficiente parte del frame.

        SIN ToF: el VL53L1X se retiró del ESP32 junto con el bus I2C nº0
        (ver el aviso grande en firmware-esp32/src/main.cpp) -- ya no hay
        ninguna medición física de distancia. En vez de estimar milímetros
        con un ``focal_px`` sin calibrar, se compara directamente qué
        fracción del frame ocupa la caja de la bandera
        (``Detection.area_fraccion``) contra
        ``ControlConfig.area_agarre_fraccion`` (pedido explícito: >60%) --
        no depende de ninguna calibración de cámara, solo de contar
        píxeles.

        Debounce (``frames_debounce_deteccion``): "al alcance y centrada"
        tiene que sostenerse varios cuadros seguidos antes de cerrar la
        pinza -- una caja que por un instante se ve más grande de lo normal
        (ruido del detector) ya no basta para disparar el agarre.
        """
        objetivo = perception.best(self.state.bandera_objetivo)

        if objetivo is None:
            # Perderla un frame suelto es normal (un parpadeo del tracker).
            # Solo se vuelve a buscar si se pierde de forma sostenida.
            frames = self.state.frames_sin_objetivo + 1
            self.state = replace(self.state, frames_sin_objetivo=frames, frames_agarre=0)
            if frames > 10:
                self._ir_a_fase(Phase.BUSCAR_BANDERA, frames_sin_objetivo=0)
                return Commands(motivo="objetivo perdido, volviendo a buscar")
            return Commands(motivo="objetivo perdido momentáneamente")

        self.state = replace(self.state, frames_sin_objetivo=0)

        centrado = abs(objetivo.angle_deg) < self._cfg.angulo_muerto_deg
        area = objetivo.area_fraccion

        al_alcance = (
            area is not None
            and area >= self._cfg.area_agarre_fraccion
            and centrado
        )
        if al_alcance:
            frames = self.state.frames_agarre + 1
            self.state = replace(self.state, frames_agarre=frames)
            if frames >= self._cfg.frames_debounce_deteccion:
                self._ir_a_fase(Phase.AGARRAR_BANDERA)
                return Commands(motivo="bandera al alcance")
        else:
            self.state = replace(self.state, frames_agarre=0)

        return self._perseguir(objetivo)

    def _agarrar_bandera(self) -> Commands:
        """Cierra el gripper sobre la bandera y espera a que el servo llegue.

        Igual que ``_inicio``/``_depositar_llave``: sin este delay,
        ``GIRO_RETORNO`` podía empezar a girar con la pinza todavía a medio
        cerrar y tirar la bandera en el proceso.

        LIMITACIÓN CONOCIDA (la señala también el PDF de lógica): no hay
        forma de VERIFICAR que la bandera quedó bien agarrada -- ningún
        sensor confirma que el gripper cerró sobre algo y no sobre el aire.
        Se manda cerrar y se asume que funcionó.
        """
        if self.state.frames_en_fase == 1:
            self.state = replace(self.state, bandera_capturada=True)
            return Commands(gripper=GripperAction.CLOSE_BANDERA, motivo="cerrando la pinza sobre la bandera")
        if self.state.frames_en_fase >= self._cfg.frames_asentamiento_gripper:
            self._ir_a_fase(Phase.GIRO_RETORNO)
            return Commands(motivo="bandera asegurada, girando para volver a la zona propia")
        return Commands(motivo="asentando el cierre de la pinza sobre la bandera")

    def _giro_retorno(self) -> Commands:
        """Gira ~180° a tiempo fijo antes de buscar la zona propia.

        Sin odometría (mismo hueco que anota ``_retornar`` más abajo), el
        robot suele quedar orientado HACIA el fondo de la pista justo
        después de perseguir y agarrar la bandera contraria -- seguir de
        frente en ese momento lo aleja más de su zona, no lo acerca. Un giro
        a tiempo fijo antes de ``_retornar`` le da al menos una oportunidad
        real de apuntar de vuelta. ``frames_giro_retorno`` en
        ``ControlConfig``, A CALIBRAR EN CANCHA -- depende del punto exacto
        donde se agarra la bandera y de la fricción de la pista.
        """
        if self.state.frames_en_fase >= self._cfg.frames_giro_retorno:
            self._ir_a_fase(Phase.RETORNAR_A_ZONA)
            return Commands(motivo="giro completado, regresando a la zona propia")
        v = self._cfg.velocidad_busqueda
        return Commands(v, -v, motivo="girando ~180° para encarar la zona propia")

    def _retornar(self, color: ColorTelemetry | None) -> Commands:
        """Vuelve a la zona propia. El sensor TRASERO avisa al llegar.

        Sensor TRASERO -- ver la nota en ``_buscar_zona_neutra`` sobre por
        qué (se retiró el delantero) y el mismo debounce
        (``frames_debounce_deteccion``) para no confiar en una sola
        lectura con ruido.
        """
        en_zona_propia = color is not None and color.back_valid and color.back is self.state.color_zona_propia
        if en_zona_propia:
            frames = self.state.frames_color_objetivo + 1
            self.state = replace(self.state, frames_color_objetivo=frames)
            if frames >= self._cfg.frames_debounce_deteccion:
                self._ir_a_fase(Phase.ENTREGAR)
                return Commands(motivo="zona propia alcanzada")
        else:
            self.state = replace(self.state, frames_color_objetivo=0)

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
