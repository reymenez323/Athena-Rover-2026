#!/usr/bin/env python3
"""Bucle principal del robot: percibir -> decidir -> enviar al ESP32.

Uso::

    python3 scripts/run_rover.py --equipo rojo
    python3 scripts/run_rover.py --equipo azul --ver     # con ventana de depuración
    python3 scripts/run_rover.py --equipo rojo --simular # sin mover motores

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
  equipo contrario. Es lo único que hace la cámara.
* **Sensores de color (TCS34725)** -> las zonas de color del piso: el amarillo
  de la zona neutra (dónde soltar la llave) y el rojo/azul de la zona propia
  (dónde termina la misión). Llegan como ``ColorTelemetry``.
* **Sensores de reflectancia (QTR)** -> distinguir el borde negro de la pista
  del fondo gris. Llegan como ``ReflectTelemetry`` y tienen prioridad
  absoluta: salirse pierde la ronda.
* **ToF (VL53L1X) delantero** -> distancia real hasta la bandera cilíndrica,
  para saber cuándo cerrar la pinza. Llega como ``ToFTelemetry``.

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
import sys
import time
from dataclasses import replace
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from athena.centering import calcular_giro, error_horizontal  # noqa: E402
from athena.config import Config  # noqa: E402
from athena.decision import DecisionMaker, Phase, RobotState  # noqa: E402
from athena.ei_flag_detector import EiDetection, EiFlagDetector  # noqa: E402
from athena.link import EspLink  # noqa: E402
from athena.protocol import (  # noqa: E402
    ColorTelemetry,
    HealthTelemetry,
    ReflectTelemetry,
    TeamColor,
    ToFTelemetry,
)
from athena.types import Detection, ObjectClass, Perception  # noqa: E402

log = logging.getLogger("rover")
_parar = False

ETIQUETA_ROJO = "CILINDRO_ROJO"
ETIQUETA_AZUL = "CILINDRO_AZUL"


def _signal_handler(signum, frame) -> None:
    global _parar
    _parar = True
    log.info("Señal recibida, deteniendo...")


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
    """Traduce la mejor detección de Edge Impulse a un ``Perception``.

    ``decision.py`` no sabe ni necesita saber cómo se detectó la bandera:
    recibe siempre el mismo tipo de objeto. El ángulo y la distancia salen del
    modelo pinhole de siempre, pero con la focal reescalada al tamaño de frame
    que usa Edge Impulse (120x120 en este proyecto), porque ``focal_px`` está
    calibrada contra ``CameraConfig.process_width`` (320 px) -- ver el
    docstring de ``GeometryConfig``.

    La distancia que sale de acá es una ESTIMACIÓN por tamaño aparente, buena
    para decidir cuándo frenar. La que dispara el cierre de la pinza es la del
    ToF del ESP32, que mide de verdad: ver ``decision._aproximar_bandera``.
    """
    if objetivo is None or frame_width_ei <= 0:
        return Perception(
            frame_id=frame_id, timestamp=timestamp, detections=(),
            latency_ms=latency_ms, model_active=True,
        )

    escala = frame_width_ei / cfg.camera.process_width
    focal_px = cfg.geometry.focal_px * escala

    box = objetivo.box
    angle_deg = math.degrees(math.atan2(box.cx - frame_width_ei / 2.0, focal_px))

    distance_mm: float | None = None
    # Si la bandera está cortada por el borde del frame, su altura aparente
    # es menor que la real y la distancia saldría inflada: se descarta.
    if box.h > 0 and box.y > 1 and (box.y + box.h) < frame_height_ei - 1:
        distance_mm = focal_px * cfg.geometry.bandera_altura_mm / box.h

    deteccion = Detection(
        cls=cls_objetivo, box=box, confidence=objetivo.confidence,
        distance_mm=distance_mm, angle_deg=angle_deg, track_id=1,
    )
    return Perception(
        frame_id=frame_id, timestamp=timestamp, detections=(deteccion,),
        latency_ms=latency_ms, model_active=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--equipo", required=True, choices=["rojo", "azul"])
    parser.add_argument("--modelo", default="models/athena_ei_banderas.eim",
                        help="ruta al .eim exportado de Edge Impulse (target Linux AARCH64)")
    parser.add_argument("--config", default=None)
    parser.add_argument("--zona-muerta", type=float, default=0.15,
                        help="mitad de ancho de la franja central considerada 'centrado', en [0,1]")
    parser.add_argument("--kp", type=float, default=60.0, help="ganancia proporcional del giro al perseguir")
    parser.add_argument("--correccion-max", type=int, default=40)
    parser.add_argument("--min-confianza", type=float, default=0.6)
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
    equipo = TeamColor.RED if args.equipo == "rojo" else TeamColor.BLUE
    etiqueta_objetivo = ETIQUETA_AZUL if equipo is TeamColor.RED else ETIQUETA_ROJO

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

    decisor = DecisionMaker(cfg.control, RobotState(team=equipo))

    log.info("Equipo: %s | objetivo: %s (%s)",
              equipo.name, decisor.state.bandera_objetivo.value, etiqueta_objetivo)
    if args.simular:
        log.warning("MODO SIMULACIÓN: no se enviarán comandos de motor.")

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
        with Camera(cfg.camera) as cam, \
             EiFlagDetector(modelo_path, min_confidence=args.min_confianza) as ei_detector, \
             EspLink(cfg.serial_port, cfg.serial_baud) as link:

            link.send_led(equipo)          # el reglamento exige identificarse
            ultimo_led = time.monotonic()

            while not _parar:
                # --- 1. Telemetría del ESP32 -------------------------------
                for paquete in link.poll():
                    if isinstance(paquete, ColorTelemetry):
                        ultimo_color = paquete
                    elif isinstance(paquete, ReflectTelemetry):
                        ultimo_reflect = paquete
                    elif isinstance(paquete, ToFTelemetry):
                        ultimo_tof = paquete
                    elif isinstance(paquete, HealthTelemetry) and paquete.faulted_bitmask:
                        # No se aborta la ronda por esto: se registra y se sigue
                        # compitiendo con lo que quede funcionando.
                        log.warning("El ESP32 reporta tareas colgadas: %s",
                                    ", ".join(paquete.faulted_tasks))

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
                percepcion = _percepcion_desde_ei(
                    objetivo_ei, decisor.state.bandera_objetivo,
                    ei_detector.frame_width, ei_detector.frame_height,
                    cfg, frame_id=frames, timestamp=time.time(),
                    latency_ms=ei_detector.last_timing_ms,
                )

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
                if objetivo_ei is not None and decisor.state.phase is Phase.APROXIMAR_BANDERA:
                    error = error_horizontal(objetivo_ei.box, ei_detector.frame_width)
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
                if decisor.state.phase is Phase.TERMINADO:
                    log.info("Misión completada.")
                    break

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
