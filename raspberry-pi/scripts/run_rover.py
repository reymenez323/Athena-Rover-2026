#!/usr/bin/env python3
"""Bucle principal del robot: percibir -> decidir -> enviar al ESP32.

Uso::

    python3 scripts/run_rover.py                         # espera el switch físico de equipo
    python3 scripts/run_rover.py --equipo rojo           # fuerza el equipo, sin esperar el switch
    python3 scripts/run_rover.py --equipo azul --ver     # con ventana de depuración
    python3 scripts/run_rover.py --equipo rojo --simular # sin mover motores

SELECCIÓN DE EQUIPO: el robot tiene un switch físico de 3 posiciones
(ON-OFF-ON, ver ``hardware/conexiones-esp32-s3.md``) que decide el equipo al
puro inicio de la secuencia -- posición 0/central = nadie ha elegido todavía
(el robot no se mueve; ``MotorTask`` en el ESP32 lo impone por su cuenta),
1 = azul, 2 = rojo. Sin ``--equipo``, este script espera esa señal
(``TeamSwitchTelemetry``) antes de arrancar la ronda. ``--equipo`` sigue
existiendo para bancos de prueba sin el switch instalado: si se pasa, se usa
directamente y no se espera nada del ESP32.

APAGADO SEGURO: cuando el switch vuelve al centro DESPUÉS de haber estado en
un equipo real (la señal de "ya terminé"), este script pide un ``sudo
shutdown`` -- no solo termina el proceso -- para no dejar que corten la
energía del robot con la Raspberry Pi todavía con el sistema de archivos
montado (la forma clásica de corromper la tarjeta SD). No se sale del bucle
al completar la misión (``Phase.TERMINADO``) justamente para poder seguir
esperando esa señal después de una ronda. Requiere permiso ``sudo`` sin
contraseña para ese comando puntual -- ver ``deploy/README.md``.

DISEÑO DEL BUCLE: es de un solo hilo a propósito (la captura de cámara sí corre
aparte). Percepción, decisión y envío en el mismo hilo hacen que el orden de
los eventos sea siempre el mismo y que un fallo sea reproducible. Con varios
hilos compartiendo estado, un robot que falla una vez cada veinte rondas es
imposible de depurar.

TOLERANCIA A FALLOS: si se cae la cámara, el enlace serial o lo que sea, el
bucle NO se muere. Manda parada y sigue intentando. Un robot detenido conserva
la posición de su bandera y puede ganar por proximidad al terminar el tiempo;
un proceso caído pierde seguro.

REPARTO DE SENSORES (quién resuelve qué):

* **Cámara USB + modelo FOMO de Edge Impulse** -> encontrar la bandera del
  equipo contrario. Es lo único que hace la cámara. Si el modelo no la ve en
  un cuadro (por ejemplo, iluminación distinta a la de sus fotos de
  entrenamiento), se prueba con ``athena.color_shape_detector`` -- color HSV
  + relación de aspecto del cilindro, de pie o caído -- antes de darse por
  vencido ese cuadro. El modelo manda cuando los dos coinciden.
* **Sensores de color (TCS34725)** -> las zonas de color del piso: el amarillo
  de la zona neutra (dónde soltar la llave) y el rojo/azul de la zona propia
  (dónde termina la misión). Llegan como ``ColorTelemetry``.
* **Sensores de reflectancia (QTR)** -> distinguir el borde negro de la pista
  del fondo gris. Llegan como ``ReflectTelemetry`` y tienen prioridad
  absoluta: salirse pierde la ronda.
* **ToF (VL53L1X) delantero** -> distancia real hasta la bandera cilíndrica,
  para saber cuándo cerrar la pinza -- solo cuando está centrada, ver
  ``decision._aproximar_bandera``. Llega como ``ToFTelemetry``.

La llave y el retorno a zona NO dependen de la cámara en absoluto, así que
cambiar el detector de bandera no les afecta. ``decision.py`` sigue siendo
quien manda en la secuencia obligatoria del reglamento (llave antes que
bandera, etc.).

SEÑALIZAR LA BANDERA: cuando el modelo ve la bandera contraria se le manda
``CMD_FLAG_SIGNAL`` al ESP32, que hace destellar el LED. Es el reto de
demostración "detectar la bandera del oponente y señalizar su detección": el
ESP32 no tiene cámara, así que esa señal solo puede venir de aquí.

CENTRADO: la fase de perseguir la bandera usa ``athena.centering`` (línea
central + zona muerta + giro proporcional al error normalizado) en vez del
control angular en grados que trae ``decision._perseguir``. Ver el bloque
"CORRECCIÓN DE GIRO" más abajo para el porqué de mantener ambos.
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from athena.centering import calcular_giro, error_horizontal  # noqa: E402
from athena.color_shape_detector import ColorShapeDetection, ColorShapeDetector  # noqa: E402
from athena.config import Config  # noqa: E402
from athena.decision import DecisionMaker, Phase, RobotState  # noqa: E402
from athena.ei_flag_detector import EiDetection, EiFlagDetector  # noqa: E402
from athena.link import EspLink  # noqa: E402
from athena.protocol import (  # noqa: E402
    ColorTelemetry,
    HealthTelemetry,
    ReflectTelemetry,
    TeamColor,
    TeamSwitchTelemetry,
    ToFTelemetry,
)
from athena.types import BBox, Detection, ObjectClass, Perception  # noqa: E402


def _apagar_pi_de_forma_segura() -> None:
    """Pide un apagado ORDENADO del sistema operativo, no solo del proceso.

    Por qué existe: cortar la energía de la Raspberry Pi con el sistema de
    archivos montado (en vez de un ``shutdown`` que lo desmonta primero) es
    el mecanismo clásico de corrupción de tarjeta SD. Sin esto, la única
    forma de terminar una ronda es apagar el proceso (Ctrl+C, o que
    ``Phase.TERMINADO`` lo cierre solo) y confiar en que quien opera el
    robot recuerde entrar por SSH a hacer ``sudo shutdown`` antes de
    desenergizar -- exactamente lo que se quiere evitar al no depender de
    monitor/SSH cada vez.

    Requiere que el usuario del servicio tenga permiso ``sudo`` SIN
    contraseña para este comando puntual (ver ``deploy/README.md``, sección
    de apagado seguro) -- el proceso corre sin terminal interactiva, así que
    un ``sudo`` que pida contraseña se quedaría colgado para siempre.
    """
    log.warning(
        "Apagando la Raspberry Pi de forma segura (sudo shutdown). "
        "Esperá a que el sistema termine de apagarse antes de desenergizar el robot."
    )
    try:
        subprocess.run(["sudo", "shutdown", "-h", "now"], check=True)
    except Exception:
        log.exception(
            "No se pudo iniciar el apagado automático (¿falta el permiso sudo "
            "sin contraseña? ver deploy/README.md). Apagá la Raspberry Pi a "
            "mano con:  sudo shutdown -h now"
        )


log = logging.getLogger("rover")
_parar = False

ETIQUETA_ROJO = "CILINDRO_ROJO"
ETIQUETA_AZUL = "CILINDRO_AZUL"


def _signal_handler(signum, frame) -> None:
    global _parar
    _parar = True
    log.info("Señal recibida, deteniendo...")


def _perception_from_box(
    box: BBox,
    confidence: float,
    cls_objetivo: ObjectClass,
    frame_width: int,
    frame_height: int,
    cfg: Config,
    frame_id: int,
    timestamp: float,
    latency_ms: float,
) -> Perception:
    """Traduce una caja (de CUALQUIER detector) a un ``Perception``.

    ``decision.py`` no sabe ni necesita saber cómo se detectó la bandera --
    modelo de Edge Impulse o el detector de color+forma de respaldo, ver
    ``_percepcion_desde_ei``/``_percepcion_desde_color`` -- así que ambos
    caminos terminan acá. El ángulo y la distancia salen del modelo pinhole
    de siempre, con la focal reescalada al ancho real del frame en el que
    está medida ``box`` (``focal_px`` está calibrada contra
    ``CameraConfig.process_width``, ver el docstring de ``GeometryConfig``).

    DE PIE O CAÍDA: se usa el lado MÁS LARGO de la caja (``max(w, h)``) como
    referencia de los 150 mm del cilindro, no siempre ``h`` -- de pie es
    prácticamente lo mismo (el lado largo ya es el alto), pero si la bandera
    se cayó y quedó de costado, el lado largo pasa a ser el ancho, y usar
    ``h`` a ciegas ahí daría una distancia varias veces menor a la real (le
    metería el diámetro donde va la altura).

    La distancia que sale de acá es una ESTIMACIÓN por tamaño aparente, buena
    para decidir cuándo frenar. La que dispara el cierre de la pinza es la del
    ToF del ESP32, que mide de verdad: ver ``decision._aproximar_bandera``.
    """
    if frame_width <= 0:
        return Perception(
            frame_id=frame_id, timestamp=timestamp, detections=(),
            latency_ms=latency_ms, model_active=True,
        )

    escala = frame_width / cfg.camera.process_width
    focal_px = cfg.geometry.focal_px * escala

    angle_deg = math.degrees(math.atan2(box.cx - frame_width / 2.0, focal_px))

    distance_mm: float | None = None
    lado_largo = max(box.w, box.h)
    # Si la bandera está cortada por cualquier borde del frame, su tamaño
    # aparente es menor que el real y la distancia saldría inflada: se
    # descarta en vez de confiar en una medición parcial.
    tocando_borde = box.x <= 1 or box.y <= 1 or (box.x + box.w) >= frame_width - 1 \
        or (box.y + box.h) >= frame_height - 1
    if lado_largo > 0 and not tocando_borde:
        distance_mm = focal_px * cfg.geometry.bandera_altura_mm / lado_largo

    deteccion = Detection(
        cls=cls_objetivo, box=box, confidence=confidence,
        distance_mm=distance_mm, angle_deg=angle_deg, track_id=1,
    )
    return Perception(
        frame_id=frame_id, timestamp=timestamp, detections=(deteccion,),
        latency_ms=latency_ms, model_active=True,
    )


def _percepcion_desde_ei(
    objetivo: EiDetection | None,
    cls_objetivo: ObjectClass,
    frame_width_ei: int,
    frame_height_ei: int,
    cfg: Config,
    frame_id: int,
    timestamp: float,
    latency_ms: float,
) -> Perception:
    """Traduce la mejor detección de Edge Impulse a un ``Perception``."""
    if objetivo is None or frame_width_ei <= 0:
        return Perception(
            frame_id=frame_id, timestamp=timestamp, detections=(),
            latency_ms=latency_ms, model_active=True,
        )
    return _perception_from_box(
        objetivo.box, objetivo.confidence, cls_objetivo,
        frame_width_ei, frame_height_ei, cfg, frame_id, timestamp, latency_ms,
    )


def _percepcion_desde_color(
    objetivo: ColorShapeDetection | None,
    cls_objetivo: ObjectClass,
    frame_width: int,
    frame_height: int,
    cfg: Config,
    frame_id: int,
    timestamp: float,
    latency_ms: float,
) -> Perception:
    """Traduce la mejor detección del respaldo color+forma a un ``Perception``.

    Mismo tratamiento que ``_percepcion_desde_ei``, pero las cajas de
    ``ColorShapeDetector`` están medidas en el frame de captura COMPLETO
    (``Camera.read_full()``), no en el 120x120 recortado por el SDK de Edge
    Impulse -- por eso ``frame_width``/``frame_height`` acá son los de
    ``frame.shape``, no ``EiFlagDetector.frame_width``.
    """
    if objetivo is None or frame_width <= 0:
        return Perception(
            frame_id=frame_id, timestamp=timestamp, detections=(),
            latency_ms=latency_ms, model_active=True,
        )
    return _perception_from_box(
        objetivo.box, objetivo.confidence, cls_objetivo,
        frame_width, frame_height, cfg, frame_id, timestamp, latency_ms,
    )


def _esperar_equipo_del_switch(link: EspLink) -> TeamColor | None:
    """Bloquea hasta que el switch físico de equipo salga de la posición central.

    El switch (``hardware/conexiones-esp32-s3.md``) es la fuente de verdad de
    qué equipo es este robot: se lee en el ESP32 y llega como
    ``TeamSwitchTelemetry``. Mientras reporte ``TeamColor.NONE`` (la posición
    central, el reposo normal al encender), no tiene sentido gastar cámara y
    CPU en una ronda que ``MotorTask`` ya se niega a mover por su cuenta (ver
    ``g_switchTeam`` en el firmware) -- mejor esperar aquí a que alguien
    elija.

    Devuelve ``None`` si se pidió detener (Ctrl+C) mientras se esperaba.
    """
    log.info("Esperando el switch físico de equipo (posición 0 = esperando)...")
    ultimo_aviso = time.monotonic()
    while not _parar:
        for paquete in link.poll():
            if isinstance(paquete, TeamSwitchTelemetry) and paquete.team is not TeamColor.NONE:
                log.info("Switch de equipo -> %s", paquete.team.name)
                return paquete.team
        if time.monotonic() - ultimo_aviso > 5.0:
            log.info("Sigo esperando el switch de equipo...")
            ultimo_aviso = time.monotonic()
        time.sleep(0.05)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--equipo", default=None, choices=["rojo", "azul"],
        help="fuerza el equipo por línea de comandos, sin esperar el switch físico "
             "(pensado para banco, sin el switch instalado). Si se omite, espera a "
             "que el switch físico de 3 posiciones salga de la posición central.",
    )
    parser.add_argument("--modelo", default="models/athena_ei_banderas.eim",
                        help="ruta al .eim exportado de Edge Impulse (target Linux AARCH64)")
    parser.add_argument("--config", default=None)
    parser.add_argument("--zona-muerta", type=float, default=0.15,
                        help="mitad de ancho de la franja central considerada 'centrado', en [0,1]")
    parser.add_argument("--kp", type=float, default=60.0, help="ganancia proporcional del giro al perseguir")
    parser.add_argument("--correccion-max", type=int, default=40)
    parser.add_argument("--min-confianza", type=float, default=0.6)
    parser.add_argument(
        "--delay-inicio", type=float, default=2.0,
        help="segundos de espera tras elegir equipo (switch o --equipo) antes de que "
             "el robot empiece a asegurar la llave -- lo pide el PDF de lógica, para "
             "dar tiempo a alejar la mano del switch/robot",
    )
    parser.add_argument("--ver", action="store_true", help="ventana con lo que ve el robot")
    parser.add_argument("--simular", action="store_true", help="no enviar comandos de motor")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    cfg = Config.load(args.config)

    modelo_path = Path(args.modelo)
    if not modelo_path.is_absolute():
        modelo_path = REPO / modelo_path

    # Import tardío: solo hace falta si se pidió la ventana de depuración, y
    # así el robot puede correr sin entorno gráfico (que es lo normal, por SSH).
    cv2 = None
    if args.ver:
        import cv2 as _cv2
        cv2 = _cv2

    from athena.camera import Camera

    ultimo_color: ColorTelemetry | None = None
    ultimo_reflect: ReflectTelemetry | None = None
    ultimo_tof: ToFTelemetry | None = None
    ultima_fase = None
    # Se manda CMD_FLAG_SIGNAL solo cuando el estado CAMBIA, no en cada
    # cuadro: el LED del ESP32 conserva el último valor recibido, así que
    # repetirlo 30 veces por segundo solo llenaría el cable de tramas
    # idénticas. Arranca en None (nunca enviado) para que el primer ciclo
    # siempre lo mande, aunque sea "no la veo".
    ultima_senal_bandera: bool | None = None
    frames = 0
    t_inicio = time.monotonic()

    try:
        if args.equipo is not None:
            equipo = TeamColor.RED if args.equipo == "rojo" else TeamColor.BLUE
        else:
            # Conexión corta, solo para esperar la elección de equipo: se
            # vuelve a abrir el enlace de verdad más abajo. EspLink reconecta
            # solo, así que abrir/cerrar dos veces seguidas es barato y deja
            # el resto del bucle principal exactamente igual que antes.
            with EspLink(cfg.serial_port, cfg.serial_baud) as switch_link:
                equipo = _esperar_equipo_del_switch(switch_link)
            if equipo is None:
                log.info("Detenido mientras se esperaba el switch de equipo.")
                return 0

        etiqueta_objetivo = ETIQUETA_AZUL if equipo is TeamColor.RED else ETIQUETA_ROJO
        decisor = DecisionMaker(cfg.control, RobotState(team=equipo))
        log.info("Equipo: %s | objetivo: %s (%s)",
                  equipo.name, decisor.state.bandera_objetivo.value, etiqueta_objetivo)
        if args.simular:
            log.warning("MODO SIMULACIÓN: no se enviarán comandos de motor.")

        # Delay pedido por el PDF de lógica: "una vez seleccionado el equipo,
        # el robot debe tener un delay antes de pasar a la siguiente acción".
        # Va aquí, no dentro de DecisionMaker, porque es sobre ELEGIR EQUIPO,
        # no sobre asegurar la llave (ese segundo delay ya lo maneja
        # decision.Phase.INICIO por su cuenta, con el gripper).
        if args.delay_inicio > 0:
            log.info("Equipo elegido -- esperando %.1fs antes de empezar...", args.delay_inicio)
            time.sleep(args.delay_inicio)

        # Detector de respaldo cuando el modelo de Edge Impulse falla por
        # iluminación distinta a la de sus fotos de entrenamiento: no
        # necesita abrir nada (no tiene ciclo de vida propio), así que va
        # aparte del `with` de arriba.
        color_detector = ColorShapeDetector(geometry=cfg.geometry)

        with Camera(cfg.camera) as cam, \
             EiFlagDetector(modelo_path, min_confidence=args.min_confianza) as ei_detector, \
             EspLink(cfg.serial_port, cfg.serial_baud) as link:

            link.send_led(equipo)          # el reglamento exige identificarse
            ultimo_led = time.monotonic()
            ultima_fuente_deteccion = None
            ultimo_switch: TeamSwitchTelemetry | None = None
            apagado_solicitado = False

            while not _parar:
                # --- 1. Telemetría del ESP32 -------------------------------
                for paquete in link.poll():
                    if isinstance(paquete, ColorTelemetry):
                        ultimo_color = paquete
                    elif isinstance(paquete, ReflectTelemetry):
                        ultimo_reflect = paquete
                    elif isinstance(paquete, ToFTelemetry):
                        ultimo_tof = paquete
                    elif isinstance(paquete, TeamSwitchTelemetry):
                        # El switch volviendo al CENTRO después de haber
                        # estado en un equipo real es la señal de "ya
                        # terminé, apagame": dispara un shutdown ordenado en
                        # vez de esperar a que alguien entre por SSH a
                        # hacerlo, o peor, que corten la energía en frío.
                        #
                        # No dispara al arrancar: _esperar_equipo_del_switch
                        # ya garantiza que este bucle solo empieza con el
                        # switch en una posición de equipo, así que hace
                        # falta una lectura PREVIA que ya fuera un equipo
                        # real -- en banco con --equipo (switch sin instalar,
                        # reportando NONE todo el tiempo) esa condición nunca
                        # se cumple y el apagado nunca se dispara solo.
                        if (ultimo_switch is not None
                                and ultimo_switch.team is not TeamColor.NONE
                                and paquete.team is TeamColor.NONE):
                            apagado_solicitado = True
                        ultimo_switch = paquete
                    elif isinstance(paquete, HealthTelemetry) and paquete.faulted_bitmask:
                        # No se aborta la ronda por esto: se registra y se sigue
                        # compitiendo con lo que quede funcionando.
                        log.warning("El ESP32 reporta tareas colgadas: %s",
                                    ", ".join(paquete.faulted_tasks))

                if apagado_solicitado:
                    log.warning("Switch de equipo -> centro (posición 0).")
                    if not args.simular:
                        link.send_stop()
                    _apagar_pi_de_forma_segura()
                    break

                if decisor.state.phase is Phase.TERMINADO:
                    # Ronda completa: ya no hay nada que decidir ni
                    # perseguir, así que se deja de gastar cámara y modelo en
                    # esto (el cambio de fase ya se registró la única vez que
                    # pasó, más abajo). Se sigue dando la vuelta al bucle
                    # -- no se sale del proceso -- solo para poder seguir
                    # drenando telemetría arriba y detectar el switch
                    # volviendo al centro.
                    time.sleep(0.05)
                    continue

                # --- 2. Percepción (cámara USB -> Edge Impulse) ------------
                # read_full() (resolución de captura completa), no read(): el
                # SDK de Edge Impulse ya reescala/recorta al tamaño del
                # Impulse (120x120) por su cuenta, reducir dos veces solo
                # perdería detalle de más antes de esa etapa.
                frame = cam.read_full()
                if frame is None:
                    if not args.simular:
                        link.send_stop()
                    time.sleep(0.01)
                    continue

                detecciones = ei_detector.detect(frame)
                objetivo_ei = EiFlagDetector.best(detecciones, etiqueta_objetivo)

                # Respaldo: si el modelo no vio la bandera este cuadro, se
                # prueba con color+forma antes de darse por vencido. Se
                # calcula siempre (no solo cuando el modelo falla) porque es
                # barato -- OpenCV puro, sin red neuronal -- y así el overlay
                # de --ver también puede mostrarlo aunque el modelo sí haya
                # detectado algo.
                color_detecciones = color_detector.detect(frame)
                objetivo_color = ColorShapeDetector.best(color_detecciones, etiqueta_objetivo)

                fh_frame, fw_frame = frame.shape[:2]
                if objetivo_ei is not None:
                    fuente_deteccion = "modelo"
                    percepcion = _percepcion_desde_ei(
                        objetivo_ei, decisor.state.bandera_objetivo,
                        ei_detector.frame_width, ei_detector.frame_height,
                        cfg, frame_id=frames, timestamp=time.time(),
                        latency_ms=ei_detector.last_timing_ms,
                    )
                elif objetivo_color is not None:
                    fuente_deteccion = "color+forma"
                    percepcion = _percepcion_desde_color(
                        objetivo_color, decisor.state.bandera_objetivo,
                        fw_frame, fh_frame, cfg, frame_id=frames,
                        timestamp=time.time(), latency_ms=ei_detector.last_timing_ms,
                    )
                else:
                    fuente_deteccion = None
                    percepcion = Perception(
                        frame_id=frames, timestamp=time.time(), detections=(),
                        latency_ms=ei_detector.last_timing_ms, model_active=True,
                    )

                if fuente_deteccion != ultima_fuente_deteccion and fuente_deteccion is not None:
                    log.info("Bandera detectada por: %s", fuente_deteccion)
                ultima_fuente_deteccion = fuente_deteccion

                # --- 3. Decisión --------------------------------------------
                comandos = decisor.step(percepcion, ultimo_color, ultimo_reflect, ultimo_tof)

                if decisor.state.phase is not ultima_fase:
                    log.info("Fase -> %s (%s)", decisor.state.phase.name, comandos.motivo)
                    ultima_fase = decisor.state.phase

                # --- 3b. CORRECCIÓN DE GIRO: línea central + zona muerta ---
                # decision.py ya decidió QUÉ hacer (perseguir, agarrar,
                # etc.) usando un ángulo en grados de compatibilidad interna.
                # Para el movimiento en sí, mientras se está persiguiendo la
                # bandera, se recalcula el giro con el error normalizado de
                # Edge Impulse directamente (0.15 de zona muerta, proporcional
                # al error) en vez del control angular -- es la lógica de
                # centrado que se pidió, aplicada sobre la caja real que
                # acaba de devolver el modelo, no sobre el ángulo derivado.
                if objetivo_ei is not None:
                    caja_activa, ancho_activo = objetivo_ei.box, ei_detector.frame_width
                elif objetivo_color is not None:
                    caja_activa, ancho_activo = objetivo_color.box, fw_frame
                else:
                    caja_activa, ancho_activo = None, 0

                if caja_activa is not None and decisor.state.phase is Phase.APROXIMAR_BANDERA:
                    error = error_horizontal(caja_activa, ancho_activo)
                    objetivo_det = percepcion.best(decisor.state.bandera_objetivo)
                    cerca = (
                        objetivo_det is not None
                        and objetivo_det.distance_mm is not None
                        and objetivo_det.distance_mm < 400.0
                    )
                    base = cfg.control.velocidad_aproximacion if cerca else cfg.control.velocidad_crucero
                    giro = calcular_giro(
                        error, zona_muerta=args.zona_muerta, velocidad_base=base,
                        kp=args.kp, correccion_max=args.correccion_max,
                    )
                    if not comandos.parado:   # no pisar una orden de agarre/gripper
                        comandos = replace(comandos, left=giro.left, right=giro.right)

                # --- 4. Envío ----------------------------------------------
                # La señal de bandera se manda SIEMPRE, incluso con --simular:
                # es una luz, no un movimiento, y es justamente lo que se
                # quiere poder verificar sin que el robot ruede.
                if comandos.bandera_a_la_vista != ultima_senal_bandera:
                    link.send_flag_signal(comandos.bandera_a_la_vista)
                    ultima_senal_bandera = comandos.bandera_a_la_vista
                    log.info("Bandera contraria %s",
                             "A LA VISTA -> LED señalizando"
                             if comandos.bandera_a_la_vista else "fuera de vista")

                if not args.simular:
                    if comandos.gripper is not None:
                        link.send_gripper(comandos.gripper)
                    if comandos.parado:
                        link.send_stop()
                    else:
                        link.send_motor(comandos.left, comandos.right)

                # El LED se reafirma cada 2 s: si el ESP32 se reinició a mitad
                # de ronda, volvería a arrancar sin equipo asignado.
                if time.monotonic() - ultimo_led > 2.0:
                    link.send_led(equipo)
                    ultimo_led = time.monotonic()

                # --- 5. Depuración visual opcional -------------------------
                if cv2 is not None:
                    vista = frame.copy()
                    fh, fw = vista.shape[:2]
                    escala_x = fw / max(1, ei_detector.frame_width)
                    escala_y = fh / max(1, ei_detector.frame_height)
                    for d in detecciones:
                        b = d.box
                        x, y = int(b.x * escala_x), int(b.y * escala_y)
                        w, h = int(b.w * escala_x), int(b.h * escala_y)
                        color = (0, 0, 255) if d.label == ETIQUETA_ROJO else (255, 128, 0)
                        cv2.rectangle(vista, (x, y), (x + w, y + h), color, 2)
                        cv2.putText(vista, f"{d.label} {d.confidence:.2f}", (x, max(12, y - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                    # Cajas del respaldo color+forma, en amarillo, ya en el
                    # sistema de referencia del frame completo (sin rescalar):
                    # así se distingue a simple vista cuál detector encontró
                    # qué, y si el color+forma dispara de más con este fondo.
                    for d in color_detecciones:
                        b = d.box
                        cv2.rectangle(vista, (b.x, b.y), (b.x + b.w, b.y + b.h), (0, 255, 255), 1)
                        cv2.putText(vista, f"{d.label} ({d.orientation}) {d.confidence:.2f}",
                                    (b.x, min(fh - 4, b.y + b.h + 12)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
                    cv2.putText(vista, f"{decisor.state.phase.name} | {ei_detector.last_timing_ms:.0f}ms",
                                (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    if ultimo_tof is not None:
                        tof_txt = f"{ultimo_tof.distance_mm}mm" if ultimo_tof.valid else "ToF invalido"
                        cv2.putText(vista, f"ToF: {tof_txt}", (8, 32),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.imshow("Athena Rover", vista)
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break

                frames += 1

    except Exception:
        log.exception("Fallo en el bucle principal")
        return 1
    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()
        transcurrido = time.monotonic() - t_inicio
        if frames:
            log.info("%d frames en %.1f s (%.1f FPS)", frames, transcurrido, frames / transcurrido)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
