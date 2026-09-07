#!/usr/bin/env python3
"""Persigue la bandera del equipo contrario usando el modelo FOMO de Edge Impulse.

Usa el mismo detector que el robot en competencia
(``src/athena/ei_flag_detector.py``) y el mismo control proporcional
(``src/athena/centering.py``), pero SIN la máquina de estados de la misión:
solo busca la bandera rival, la centra, avanza hacia ella y señaliza la
detección con el LED.

``run_rover.py`` YA integra este mismo modelo y esta misma lógica de centrado
dentro de la misión completa (llave, zona neutra, gripper, retorno) -- este
script queda como herramienta APARTE para calibrar a mano, con la ventana
``--ver``, la zona muerta y la ganancia de giro sin tener que correr la
misión entera cada vez. No lo uses como el programa que corre en competencia:
no deposita la llave, así que arrancarlo solo pierde la ronda de inmediato
según el reglamento.

REQUISITOS en la Raspberry Pi (una sola vez)::

    pip3 install edge_impulse_linux
    chmod +x models/athena_ei_banderas.eim

USO::

    python3 scripts/run_flag_tracker_ei.py --equipo rojo
    python3 scripts/run_flag_tracker_ei.py --equipo azul --ver     # con ventana de depuración
    python3 scripts/run_flag_tracker_ei.py --equipo rojo --simular # sin mover motores

LÍNEA CENTRAL Y ZONA MUERTA: se traza una línea vertical de referencia en el
centro del frame que entrega el modelo. Si el centro de la caja de la bandera
cae dentro de +-0.15 de esa línea (``--zona-muerta``), se considera centrada y
el robot avanza recto. Fuera de esa franja, gira proporcional a qué tan lejos
está del centro -- ver ``athena.centering.calcular_giro``.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from athena.centering import calcular_giro, error_horizontal  # noqa: E402
from athena.config import Config  # noqa: E402
from athena.ei_flag_detector import EiFlagDetector  # noqa: E402
from athena.link import EspLink  # noqa: E402
from athena.protocol import TeamColor  # noqa: E402

log = logging.getLogger("flag_tracker_ei")
_parar = False

ETIQUETA_ROJO = "CILINDRO_ROJO"
ETIQUETA_AZUL = "CILINDRO_AZUL"


def _signal_handler(signum, frame) -> None:
    global _parar
    _parar = True
    log.info("Señal recibida, deteniendo...")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--equipo", required=True, choices=["rojo", "azul"])
    parser.add_argument("--modelo", default="models/athena_ei_banderas.eim",
                        help="ruta al .eim exportado de Edge Impulse (target Linux AARCH64)")
    parser.add_argument("--config", default=None)
    parser.add_argument("--zona-muerta", type=float, default=0.15,
                        help="mitad de ancho de la franja central considerada 'centrado', en [0,1]")
    parser.add_argument("--kp", type=float, default=60.0, help="ganancia proporcional del giro")
    parser.add_argument("--correccion-max", type=int, default=40)
    parser.add_argument("--velocidad-base", type=int, default=35, help="%% de PWM al avanzar/buscar")
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
    # Se persigue la bandera del CONTRARIO -- mismo criterio que decision.py.
    etiqueta_objetivo = ETIQUETA_AZUL if equipo is TeamColor.RED else ETIQUETA_ROJO

    modelo_path = Path(args.modelo)
    if not modelo_path.is_absolute():
        modelo_path = REPO / modelo_path

    # Import tardío: solo hace falta si se pidió la ventana de depuración, y
    # así el script puede correr sin entorno gráfico (lo normal, por SSH).
    cv2 = None
    if args.ver:
        import cv2 as _cv2
        cv2 = _cv2

    from athena.camera import Camera

    log.info("Equipo: %s | persiguiendo: %s", equipo.name, etiqueta_objetivo)
    if args.simular:
        log.warning("MODO SIMULACIÓN: no se enviarán comandos de motor.")

    frames = 0
    frames_sin_objetivo = 0
    sentido_busqueda = 1
    # Igual que en run_rover.py: CMD_FLAG_SIGNAL solo cuando el estado cambia.
    ultima_senal_bandera: bool | None = None
    t_inicio = time.monotonic()

    try:
        with Camera(cfg.camera) as cam, \
             EiFlagDetector(modelo_path, min_confidence=args.min_confianza) as detector, \
             EspLink(cfg.serial_port, cfg.serial_baud) as link:

            link.send_led(equipo)          # el reglamento exige identificarse
            ultimo_led = time.monotonic()

            while not _parar:
                # read_full() (resolución de captura completa), no read(): el
                # SDK de Edge Impulse ya reescala/recorta al tamaño del
                # Impulse (120x120) por su cuenta, así que reducir dos veces
                # solo perdería detalle antes de esa etapa sin ganar nada.
                frame = cam.read_full()
                if frame is None:
                    if not args.simular:
                        link.send_stop()
                    time.sleep(0.01)
                    continue

                detecciones = detector.detect(frame)
                objetivo = detector.best(detecciones, etiqueta_objetivo)

                # Señalizar la detección con el LED del ESP32 (reto de la
                # demostración). Se manda también con --simular: es una luz,
                # no un movimiento, y así se puede verificar en la mesa sin
                # que el robot ruede.
                a_la_vista = objetivo is not None
                if a_la_vista != ultima_senal_bandera:
                    link.send_flag_signal(a_la_vista)
                    ultima_senal_bandera = a_la_vista

                if objetivo is not None:
                    frames_sin_objetivo = 0
                    error = error_horizontal(objetivo.box, detector.frame_width)
                    giro = calcular_giro(
                        error,
                        zona_muerta=args.zona_muerta,
                        velocidad_base=args.velocidad_base,
                        kp=args.kp,
                        correccion_max=args.correccion_max,
                    )
                    estado = "centrado, avanzando" if giro.centrado else "corrigiendo"
                    log.debug(
                        "%s (%.2f) error=%+.2f %s -> L=%d R=%d",
                        objetivo.label, objetivo.confidence, error, estado, giro.left, giro.right,
                    )
                    if not args.simular:
                        link.send_motor(giro.left, giro.right)
                else:
                    # Sin objetivo: gira sobre el propio eje barriendo el
                    # campo, igual que decision._buscar_bandera, alternando de
                    # sentido cada ~3s para no dar vueltas siempre hacia el
                    # mismo lado.
                    frames_sin_objetivo += 1
                    if frames_sin_objetivo % 90 == 0:
                        sentido_busqueda = -sentido_busqueda
                    v = args.velocidad_base
                    if not args.simular:
                        link.send_motor(v * sentido_busqueda, -v * sentido_busqueda)

                # El LED se reafirma cada 2s: si el ESP32 se reinició a mitad
                # de ronda, volvería a arrancar sin equipo asignado.
                if time.monotonic() - ultimo_led > 2.0:
                    link.send_led(equipo)
                    ultimo_led = time.monotonic()

                if cv2 is not None:
                    vista = frame.copy()
                    fh, fw = vista.shape[:2]
                    escala_x = fw / max(1, detector.frame_width)
                    escala_y = fh / max(1, detector.frame_height)
                    centro_x = fw // 2
                    zona_px = int(args.zona_muerta * (fw / 2))
                    cv2.line(vista, (centro_x, 0), (centro_x, fh), (0, 255, 255), 1)
                    cv2.line(vista, (centro_x - zona_px, 0), (centro_x - zona_px, fh), (0, 180, 180), 1)
                    cv2.line(vista, (centro_x + zona_px, 0), (centro_x + zona_px, fh), (0, 180, 180), 1)
                    for d in detecciones:
                        b = d.box
                        x, y = int(b.x * escala_x), int(b.y * escala_y)
                        w, h = int(b.w * escala_x), int(b.h * escala_y)
                        color = (0, 0, 255) if d.label == ETIQUETA_ROJO else (255, 128, 0)
                        cv2.rectangle(vista, (x, y), (x + w, y + h), color, 2)
                        cv2.putText(vista, f"{d.label} {d.confidence:.2f}", (x, max(12, y - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                    cv2.putText(vista, f"{detector.last_timing_ms:.0f}ms",
                                (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.imshow("Athena Rover - EI flag tracker", vista)
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break

                frames += 1

    except Exception:
        log.exception("Fallo en el bucle de seguimiento")
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
