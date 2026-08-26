#!/usr/bin/env python3
"""Mide el rendimiento real del pipeline en ESTA Raspberry Pi.

No confíes en los números que trae el README: mídelos. El rendimiento cambia
con el modelo de Pi, la temperatura, la fuente de alimentación y hasta con lo
que haya en el encuadre (más objetos rojos = más propuestas = más trabajo).

Uso::

    python3 scripts/benchmark.py                 # con la cámara real
    python3 scripts/benchmark.py --sintetico     # sin cámara, imágenes generadas
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

import numpy as np  # noqa: E402

from athena.camera import Camera  # noqa: E402
from athena.config import Config  # noqa: E402
from athena.detector import Detector  # noqa: E402


def frame_sintetico(w: int, h: int, rng: np.random.Generator) -> np.ndarray:
    """Frame de prueba con un cilindro rojo y un cubo azul sobre fondo gris."""
    img = np.full((h, w, 3), 90, dtype=np.uint8)
    img[:, :, 0] += rng.integers(0, 12, size=(h, w), dtype=np.uint8)   # algo de ruido
    img[h // 3 : h // 3 + 60, w // 2 - 10 : w // 2 + 10] = (40, 40, 200)   # rojo (BGR)
    img[h // 2 : h // 2 + 18, w // 4 : w // 4 + 18] = (200, 60, 40)        # azul
    return img


def resumen(nombre: str, muestras: list[float]) -> None:
    if not muestras:
        return
    muestras = sorted(muestras)
    p50 = statistics.median(muestras)
    p95 = muestras[int(len(muestras) * 0.95)]
    print(f"  {nombre:22s} media {statistics.mean(muestras):6.2f} ms | "
          f"p50 {p50:6.2f} ms | p95 {p95:6.2f} ms")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--sintetico", action="store_true", help="no usar la cámara")
    parser.add_argument("--config", default=None)
    args = parser.parse_args()

    cfg = Config.load(args.config)
    detector = Detector(cfg, base_dir=REPO)

    print("=" * 62)
    print("Benchmark del pipeline de visión — Athena Rover")
    print("=" * 62)
    print(f"  Resolución de proceso : {cfg.camera.process_width}x{cfg.camera.process_height}")
    print(f"  Modelo CNN            : {'ACTIVO' if detector.model_active else 'AUSENTE (modo degradado)'}")
    print(f"  Hilos del clasificador: {cfg.classifier.num_threads}")
    print()

    latencias: list[float] = []
    detecciones_totales = 0

    if args.sintetico:
        rng = np.random.default_rng(0)
        frames = [frame_sintetico(cfg.camera.process_width, cfg.camera.process_height, rng)
                  for _ in range(30)]
        inicio = time.perf_counter()
        for i in range(args.frames):
            p = detector.process(i, frames[i % len(frames)])
            latencias.append(p.latency_ms)
            detecciones_totales += len(p.detections)
        total = time.perf_counter() - inicio
    else:
        with Camera(cfg.camera) as cam:
            # Descartar los primeros frames: la cámara ajusta exposición y
            # ganancia al arrancar, y esos frames no son representativos.
            for _ in range(30):
                cam.read()
                time.sleep(0.01)

            inicio = time.perf_counter()
            procesados = 0
            while procesados < args.frames:
                lectura = cam.read()
                if lectura is None:
                    continue
                frame_id, frame = lectura
                p = detector.process(frame_id, frame)
                latencias.append(p.latency_ms)
                detecciones_totales += len(p.detections)
                procesados += 1
            total = time.perf_counter() - inicio

    print("Latencia por frame:")
    resumen("pipeline completo", latencias)
    print()
    print(f"  Throughput sostenido  : {args.frames / total:.1f} FPS")
    print(f"  Detecciones por frame : {detecciones_totales / max(1, args.frames):.2f}")
    print()
    print("Cómo leer esto: si el p95 supera los 33 ms, el pipeline no sigue el")
    print("ritmo de una cámara a 30 FPS y se estarán descartando frames. Baja")
    print("process_width/height o max_proposals en la configuración.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
