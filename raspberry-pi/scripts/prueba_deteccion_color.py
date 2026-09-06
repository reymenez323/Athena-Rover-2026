#!/usr/bin/env python3
"""Experimento aislado: detector de banderas por color + forma, sin ML.

Complementa (no reemplaza) al modelo FOMO de Edge Impulse cuando la luz del
salón no se parece a la de las fotos de entrenamiento: busca blobs del color
de equipo contrario y los filtra por la relación de aspecto real del
cilindro (alto/ancho, ver ``athena.config.GeometryConfig``), no por su
tamaño en píxeles. Ver ``athena/color_shape_detector.py`` para el porqué.

Corre SOLO con la cámara -- no necesita el ESP32 conectado -- para poder
probar bajo distintas luces sin depender del resto del robot.

USO::

    python3 scripts/prueba_deteccion_color.py --equipo rojo
    python3 scripts/prueba_deteccion_color.py --equipo azul --ver
    python3 scripts/prueba_deteccion_color.py --equipo rojo --v-min 40   # luz baja
    python3 scripts/prueba_deteccion_color.py --equipo rojo --s-min 60   # luz muy blanca/deslavada

Si no detecta nada, subí primero ``--verbose`` y mirá si el problema es el
tono (H, no debería cambiar mucho con la luz) o el brillo/saturación (S/V,
los primeros sospechosos). Con ``--ver`` además se abren las ventanas de
máscara binaria de cada color -- blanco es "sí es este color", negro "no".
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from dataclasses import replace
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from athena.color_shape_detector import (  # noqa: E402
    ETIQUETA_AZUL,
    ETIQUETA_ROJO,
    RANGOS_AZUL,
    RANGOS_ROJO,
    ColorShapeDetector,
    HsvRange,
)
from athena.config import Config  # noqa: E402

log = logging.getLogger("prueba_color")
_parar = False


def _signal_handler(signum, frame) -> None:
    global _parar
    _parar = True
    log.info("Señal recibida, deteniendo...")


def _ajustar_rangos(
    rangos: tuple[HsvRange, ...], s_min: int | None, v_min: int | None,
) -> tuple[HsvRange, ...]:
    if s_min is None and v_min is None:
        return rangos
    cambios = {}
    if s_min is not None:
        cambios["s_min"] = s_min
    if v_min is not None:
        cambios["v_min"] = v_min
    return tuple(replace(r, **cambios) for r in rangos)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--equipo", required=True, choices=["rojo", "azul"],
                        help="tu equipo -- se busca el cilindro del CONTRARIO")
    parser.add_argument("--config", default=None)
    parser.add_argument("--s-min", type=int, default=None,
                        help="mínimo de saturación (0-255); subilo si detecta de más con luz blanca")
    parser.add_argument("--v-min", type=int, default=None,
                        help="mínimo de brillo (0-255); bajalo si no detecta nada con poca luz")
    parser.add_argument("--tolerancia-aspecto", type=float, default=0.35,
                        help="margen relativo permitido sobre alto/ancho esperado (0-1)")
    parser.add_argument("--min-confianza", type=float, default=0.5)
    parser.add_argument("--ver", action="store_true", help="ventanas con el frame y las máscaras")
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
    # Igual que en run_flag_tracker_ei.py: se persigue la bandera CONTRARIA.
    etiqueta_objetivo = ETIQUETA_AZUL if args.equipo == "rojo" else ETIQUETA_ROJO

    detector = ColorShapeDetector(
        geometry=cfg.geometry,
        rangos_rojo=_ajustar_rangos(RANGOS_ROJO, args.s_min, args.v_min),
        rangos_azul=_ajustar_rangos(RANGOS_AZUL, args.s_min, args.v_min),
        tolerancia_aspecto=args.tolerancia_aspecto,
        min_confidence=args.min_confianza,
    )

    # Import tardío, igual que en run_flag_tracker_ei.py: solo hace falta si
    # se pidió ventana de depuración, así el script corre sin entorno
    # gráfico (lo normal, por SSH).
    cv2 = None
    if args.ver:
        import cv2 as _cv2
        cv2 = _cv2

    from athena.camera import Camera

    log.info("Equipo: %s | buscando: %s | aspecto esperado (alto/ancho): %.2f",
              args.equipo.upper(), etiqueta_objetivo, detector.aspecto_esperado)

    frames = 0
    frames_con_objetivo = 0
    confianzas: list[float] = []
    t_inicio = time.monotonic()

    try:
        with Camera(cfg.camera) as cam:
            while not _parar:
                frame = cam.read_full()
                if frame is None:
                    time.sleep(0.01)
                    continue

                mascaras: dict = {} if cv2 is not None else None
                detecciones = detector.detect(frame, mascaras=mascaras)
                objetivo = ColorShapeDetector.best(detecciones, etiqueta_objetivo)

                if objetivo is not None:
                    frames_con_objetivo += 1
                    confianzas.append(objetivo.confidence)
                    log.debug(
                        "%s conf=%.2f aspecto=%.2f (esperado %.2f) box=%s",
                        objetivo.label, objetivo.confidence, objetivo.aspect_ratio,
                        detector.aspecto_esperado, objetivo.box,
                    )

                if cv2 is not None:
                    vista = frame.copy()
                    for d in detecciones:
                        b = d.box
                        color = (0, 0, 255) if d.label == ETIQUETA_ROJO else (255, 128, 0)
                        cv2.rectangle(vista, (b.x, b.y), (b.x + b.w, b.y + b.h), color, 2)
                        cv2.putText(vista, f"{d.label} {d.confidence:.2f}", (b.x, max(12, b.y - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                    cv2.imshow("Athena Rover - color+forma", vista)
                    for etiqueta, mascara in (mascaras or {}).items():
                        cv2.imshow(f"mascara {etiqueta}", mascara)
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break

                frames += 1
    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()
        transcurrido = time.monotonic() - t_inicio
        tasa = (frames_con_objetivo / frames * 100) if frames else 0.0
        confianza_prom = (sum(confianzas) / len(confianzas)) if confianzas else 0.0
        print("\n" + "=" * 72)
        print("RESUMEN")
        print("=" * 72)
        fps = f"{frames / transcurrido:.1f} FPS" if transcurrido > 0 else "?"
        print(f"Frames procesados:      {frames}  ({fps})")
        print(f"Frames con detección:   {frames_con_objetivo}  ({tasa:.1f}% del total)")
        print(f"Confianza promedio:     {confianza_prom:.2f}")
        if frames and tasa < 30:
            print("\nDetectó poco. Antes de ajustar --s-min/--v-min a ciegas, corré con")
            print("--ver y mirá la ventana de máscara: si sale casi toda negra, el color")
            print("no está entrando en el rango de H/S/V; si sale blanca pero sin bbox,")
            print("el problema es la relación de aspecto (¿el cilindro está de lado o")
            print("tapado?), no el color.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
