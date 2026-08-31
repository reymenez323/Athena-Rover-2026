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

from athena.config import Config  # noqa: E402
from athena.decision import DecisionMaker, Phase, RobotState  # noqa: E402
from athena.detector import Detector  # noqa: E402
from athena.link import EspLink  # noqa: E402
from athena.protocol import (  # noqa: E402
    ColorTelemetry,
    HealthTelemetry,
    ReflectTelemetry,
    TeamColor,
    ToFTelemetry,
)

log = logging.getLogger("rover")
_parar = False


def _signal_handler(signum, frame) -> None:
    global _parar
    _parar = True
    log.info("Señal recibida, deteniendo...")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--equipo", required=True, choices=["rojo", "azul"])
    parser.add_argument("--config", default=None)
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

    # Import tardío: solo hace falta si se pidió la ventana de depuración, y
    # así el robot puede correr sin entorno gráfico (que es lo normal, por SSH).
    cv2 = None
    if args.ver:
        import cv2 as _cv2
        cv2 = _cv2

    from athena.camera import Camera

    detector = Detector(cfg, base_dir=REPO)
    decisor = DecisionMaker(cfg.control, RobotState(team=equipo))

    log.info("Equipo: %s | objetivo: %s", equipo.name, decisor.state.bandera_objetivo.value)
    if not detector.model_active:
        log.warning("Sin modelo CNN: corriendo en modo degradado (reglas de color y forma).")
    if args.simular:
        log.warning("MODO SIMULACIÓN: no se enviarán comandos de motor.")

    ultimo_color: ColorTelemetry | None = None
    ultimo_reflect: ReflectTelemetry | None = None
    ultimo_tof: ToFTelemetry | None = None
    ultima_fase = None
    frames = 0
    t_inicio = time.monotonic()

    try:
        with Camera(cfg.camera) as cam, EspLink(cfg.serial_port, cfg.serial_baud) as link:
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

                # --- 2. Percepción -----------------------------------------
                lectura = cam.read()
                if lectura is None:
                    link.send_stop()
                    time.sleep(0.01)
                    continue
                frame_id, frame = lectura
                percepcion = detector.process(frame_id, frame)

                # --- 3. Decisión -------------------------------------------
                comandos = decisor.step(percepcion, ultimo_color, ultimo_reflect, ultimo_tof)

                if decisor.state.phase is not ultima_fase:
                    log.info("Fase -> %s (%s)", decisor.state.phase.name, comandos.motivo)
                    ultima_fase = decisor.state.phase

                # --- 4. Envío ----------------------------------------------
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
                    for d in percepcion.detections:
                        b = d.box
                        color = (0, 0, 255) if "roja" in d.cls.value else (255, 128, 0)
                        cv2.rectangle(vista, (b.x, b.y), (b.x + b.w, b.y + b.h), color, 2)
                        dist = f"{d.distance_mm:.0f}mm" if d.distance_mm else "?"
                        cv2.putText(vista, f"{d.cls.value} {d.confidence:.2f} {dist}",
                                    (b.x, max(12, b.y - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.4, color, 1)
                    cv2.putText(vista, f"{decisor.state.phase.name} | {percepcion.latency_ms:.1f}ms",
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
