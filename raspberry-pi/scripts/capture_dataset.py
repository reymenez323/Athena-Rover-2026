#!/usr/bin/env python3
"""Captura imágenes para entrenar el clasificador.

Uso::

    python3 scripts/capture_dataset.py --clase bandera_roja
    python3 scripts/capture_dataset.py --clase fondo --auto

Teclas: ESPACIO guarda, `a` alterna captura automática, `q` sale.

CONSEJO que vale más que el código: toma las fotos con la cámara MONTADA EN EL
ROBOT, a la altura y el ángulo reales. Un dataset tomado a mano, de pie, con el
celular, entrena un modelo que funciona de pie y falla montado en el robot.
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

import cv2  # noqa: E402

from athena.camera import Camera  # noqa: E402
from athena.config import Config  # noqa: E402
from athena.types import ObjectClass  # noqa: E402

CLASES = [c.value for c in ObjectClass]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--clase", required=True, choices=CLASES, help="clase a capturar")
    parser.add_argument("--auto", action="store_true", help="empezar en modo automático")
    parser.add_argument("--intervalo", type=float, default=0.5, help="segundos entre fotos en modo automático")
    parser.add_argument("--config", default=None, help="archivo JSON de configuración")
    args = parser.parse_args()

    cfg = Config.load(args.config)
    destino = REPO / "data" / "raw" / args.clase
    destino.mkdir(parents=True, exist_ok=True)

    ya_hay = len(list(destino.glob("*.jpg")))
    print(f"Clase '{args.clase}' -> {destino}")
    print(f"Ya hay {ya_hay} imágenes. ESPACIO=guardar  a=auto  q=salir")

    auto = args.auto
    ultimo = 0.0
    guardadas = 0

    with Camera(cfg.camera) as cam:
        while True:
            frame = cam.read_full()
            if frame is None:
                time.sleep(0.01)
                continue

            vista = frame.copy()
            estado = "AUTO" if auto else "manual"
            cv2.putText(
                vista, f"{args.clase} | {estado} | guardadas: {guardadas} (total {ya_hay + guardadas})",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )
            cv2.imshow("Captura de dataset - Athena Rover", vista)

            key = cv2.waitKey(1) & 0xFF
            ahora = time.monotonic()
            guardar = False

            if key == ord("q"):
                break
            if key == ord("a"):
                auto = not auto
                ultimo = ahora
            if key == ord(" "):
                guardar = True
            if auto and (ahora - ultimo) >= args.intervalo:
                guardar = True
                ultimo = ahora

            if guardar:
                nombre = f"{args.clase}_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg"
                # Calidad 92: por encima de eso el archivo crece sin que el
                # modelo aprenda nada más.
                cv2.imwrite(str(destino / nombre), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
                guardadas += 1

    cv2.destroyAllWindows()
    print(f"\nSe guardaron {guardadas} imágenes nuevas en {destino}")
    print(f"Total de la clase '{args.clase}': {ya_hay + guardadas}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
