#!/usr/bin/env python3
"""Captura rápida de fotos con la webcam de la LAPTOP (o cualquier webcam USB).

A diferencia de ``capture_dataset.py`` (que abre la cámara con el backend
V4L2 de Linux, pensado para la Raspberry Pi), este script usa el backend que
OpenCV elija por defecto -- funciona tal cual en Windows/Mac/Linux con
cualquier webcam integrada o USB (incluida la Argom CAM20 conectada por USB).

MODO PRUEBA -- confirmar que la webcam por USB anda antes de capturar en serio::

    python capture_webcam_laptop.py --probar

Abre las cámaras (índice 0, 1, 2...) una por una y muestra el video en vivo,
para identificar cuál índice es cuál cámara. 'n' pasa a la siguiente cámara,
'q' sale del modo prueba sin guardar nada.

MODO CAPTURA -- la clase se elige y se cambia EN VIVO, sin reiniciar el script::

    python capture_webcam_laptop.py
    python capture_webcam_laptop.py --camara 1

Teclas durante la captura:
    1 = clase "fondo"
    2 = clase "cilindro_azul"
    3 = clase "cilindro_rojo"
    ESPACIO = guardar una foto de la clase que esté activa en ese momento
    a = alterna captura automática (una foto cada --intervalo segundos)
    q = salir
"""

from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2

CLASES = {
    ord("1"): "fondo",
    ord("2"): "cilindro_azul",
    ord("3"): "cilindro_rojo",
}


def probar_camaras(max_indices: int) -> None:
    """Abre cámaras 0..max_indices-1 una por una para identificar cuál es cuál."""
    for idx in range(max_indices):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            print(f"Cámara {idx}: no se pudo abrir (probablemente no existe)")
            cap.release()
            continue

        print(f"Cámara {idx}: abierta. 'n' = siguiente cámara, 'q' = salir del todo")
        while True:
            ok, frame = cap.read()
            if not ok:
                print(f"Cámara {idx}: se abrió pero no entrega imagen")
                break
            vista = frame.copy()
            cv2.putText(
                vista, f"camara indice {idx}  ({frame.shape[1]}x{frame.shape[0]})  n=siguiente q=salir",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )
            cv2.imshow("Prueba de camaras", vista)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("n"):
                break
            if key == ord("q"):
                cap.release()
                cv2.destroyAllWindows()
                return
        cap.release()
    cv2.destroyAllWindows()
    print("Listo. Anotá el índice que mostraba la cámara correcta y usalo con --camara <numero>.")


def carpeta_de(base: Path, clase: str) -> Path:
    d = base / clase
    d.mkdir(parents=True, exist_ok=True)
    return d


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--probar", action="store_true", help="solo probar qué índice de cámara es cuál, no guarda fotos")
    parser.add_argument("--indices-a-probar", type=int, default=3, help="cuántos índices probar con --probar")
    parser.add_argument("--salida", default="fotos_laptop", help="carpeta base donde se guardan las fotos")
    parser.add_argument("--camara", type=int, default=0, help="índice de la cámara a usar para capturar")
    parser.add_argument("--intervalo", type=float, default=0.5, help="segundos entre fotos en modo automático")
    args = parser.parse_args()

    if args.probar:
        probar_camaras(args.indices_a_probar)
        return 0

    base = Path(args.salida)
    clase_actual = "fondo"

    cap = cv2.VideoCapture(args.camara)
    if not cap.isOpened():
        raise SystemExit(f"No se pudo abrir la cámara {args.camara}. Probá primero con --probar.")

    conteo = {c: len(list(carpeta_de(base, c).glob("*.jpg"))) for c in set(CLASES.values())}

    print("Clases: 1=fondo  2=cilindro_azul  3=cilindro_rojo  (cambiá de clase en cualquier momento)")
    print("ESPACIO=guardar  a=auto  q=salir")

    auto = False
    ultimo = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("No se pudo leer un frame de la cámara.")
            break

        vista = frame.copy()
        estado = "AUTO" if auto else "manual"
        cv2.putText(
            vista, f"[{clase_actual}] {estado} | guardadas: {conteo[clase_actual]}",
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
        )
        cv2.imshow("Captura rapida - Athena Rover", vista)

        key = cv2.waitKey(1) & 0xFF
        ahora = time.monotonic()
        guardar = False

        if key == ord("q"):
            break
        if key in CLASES:
            clase_actual = CLASES[key]
        if key == ord("a"):
            auto = not auto
            ultimo = ahora
        if key == ord(" "):
            guardar = True
        if auto and (ahora - ultimo) >= args.intervalo:
            guardar = True
            ultimo = ahora

        if guardar:
            nombre = f"{clase_actual}_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg"
            cv2.imwrite(str(carpeta_de(base, clase_actual) / nombre), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            conteo[clase_actual] += 1

    cap.release()
    cv2.destroyAllWindows()
    print("\nResumen de fotos guardadas esta sesión:")
    for c, n in conteo.items():
        print(f"  {c}: {n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
