#!/usr/bin/env python3
"""Calibración de sensores IR — Athena Rover 2026.

Coordina con el firmware de ``firmware/`` (mismo ESP32-S3 del rover, pero
subido aparte con solo los sensores bajo prueba conectados — 2x QTRX-HD-01A
analógicos más un módulo IR genérico): manda el comando ``'R'`` por serial,
recibe ``DATA,<analog_a>,<analog_b>,<generic>``, y guarda las muestras en un
.csv dentro de ``data_logs/``.

Uso::

    python3 calibrar_ir.py --puerto COM5 --superficie NEGRO --puntos 4 --muestras 60
    python3 calibrar_ir.py --puerto /dev/ttyUSB0 --superficie GRIS --puntos 6 --muestras 50 --intervalo-ms 300

Antes de cada punto (incluido el primero), el script cuenta regresivamente
``--pausa-s`` segundos: es el tiempo para mover el sensor a la superficie o
posición que toca medir.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    print("Falta pyserial. Instálalo con: pip install pyserial", file=sys.stderr)
    raise SystemExit(1)

REPO = Path(__file__).resolve().parent
DATA_LOGS = REPO / "data_logs"


def esperar_ready(ser: "serial.Serial", timeout_s: float = 5.0) -> None:
    """Descarta lo que haya en el buffer y espera la línea READY del ESP32."""
    ser.reset_input_buffer()
    limite = time.monotonic() + timeout_s
    while time.monotonic() < limite:
        linea = ser.readline().decode(errors="ignore").strip()
        if linea == "READY":
            return
    raise TimeoutError(
        "El ESP32 no mandó 'READY' a tiempo. Revisa que el puerto sea el "
        "correcto y que el sketch de firmware/ esté cargado (no el "
        "monitor serial de otra ventana ocupando el puerto)."
    )


def leer_muestra(ser: "serial.Serial", timeout_s: float = 2.0) -> tuple[int, int, int]:
    """Manda el comando 'R' y parsea la respuesta DATA,analog_a,analog_b,generic."""
    ser.write(b"R")
    limite = time.monotonic() + timeout_s
    while time.monotonic() < limite:
        linea = ser.readline().decode(errors="ignore").strip()
        if linea.startswith("DATA,"):
            partes = linea.split(",")
            if len(partes) != 4:
                continue  # línea corrupta o de sobra, se ignora y se sigue leyendo
            _, analog_a, analog_b, generic = partes
            return int(analog_a), int(analog_b), int(generic)
    raise TimeoutError("El ESP32 no respondió a 'R' a tiempo.")


def contar(segundos: float) -> None:
    """Cuenta regresiva en la consola mientras el usuario mueve el sensor."""
    restante = segundos
    while restante > 0:
        print(f"\r  Mueve el sensor a la posición/superficie que toca... {restante:4.1f} s ", end="", flush=True)
        paso = min(1.0, restante)
        time.sleep(paso)
        restante -= paso
    print("\r" + " " * 60 + "\r", end="", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--puerto", required=True, help="puerto serial del ESP32 (ej. COM5, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--superficie", required=True, help="nombre de la superficie, ej. NEGRO, GRIS")
    parser.add_argument("--puntos", type=int, default=4, help="cuántos puntos/posiciones medir")
    parser.add_argument("--muestras", type=int, default=60, help="muestras por punto")
    parser.add_argument("--intervalo-ms", type=int, default=250, help="ms entre muestras dentro de un punto")
    parser.add_argument(
        "--pausa-s", type=float, default=10.0,
        help="segundos de pausa antes de cada punto, para reposicionar el sensor",
    )
    args = parser.parse_args()

    DATA_LOGS.mkdir(parents=True, exist_ok=True)
    superficie = args.superficie.strip().upper()
    ahora = datetime.now()
    destino = DATA_LOGS / f"IR_{superficie}_{ahora:%Y-%m-%d_%H-%M-%S}.csv"

    print(f"Conectando a {args.puerto} @ {args.baud}...")
    try:
        ser = serial.Serial(args.puerto, args.baud, timeout=1)
    except serial.SerialException as exc:
        print(f"No se pudo abrir el puerto {args.puerto}: {exc}", file=sys.stderr)
        print(
            "Revisa: ¿es el puerto correcto? ¿hay un pio device monitor u "
            "otra ventana usándolo? ¿el cable es de datos, no solo de carga?",
            file=sys.stderr,
        )
        return 1

    total_muestras = args.puntos * args.muestras

    with ser, destino.open("w", newline="", encoding="utf-8") as f:
        time.sleep(2)  # si el ESP32 se resetea al abrir el puerto, darle tiempo a arrancar
        try:
            esperar_ready(ser)
            print("ESP32 listo.\n")
        except TimeoutError:
            # Con USB nativo (el que usa este sketch) el ESP32 no siempre se
            # resetea al abrir el puerto: si ya estaba corriendo de antes,
            # el "READY" se mandó una sola vez al arrancar y nadie lo
            # escuchaba todavía — no es necesariamente un error. Como
            # respaldo, se prueba un 'R' directo: si responde con datos
            # válidos, el sketch correcto SÍ está corriendo y se continúa.
            print(
                "No llegó 'READY' — probando un comando directo por si el "
                "ESP32 ya estaba corriendo de antes...",
            )
            try:
                leer_muestra(ser)
            except TimeoutError:
                raise TimeoutError(
                    "El ESP32 no respondió ni a 'READY' ni a un comando "
                    "'R' directo. Revisa el puerto, que el sketch de "
                    "firmware/ esté cargado, y que no haya otra ventana "
                    "(pio device monitor, Arduino IDE) usando el puerto."
                ) from None
            print("ESP32 responde bien, sigo aunque no vi 'READY'.\n")

        # Encabezado con los metadatos de la corrida, como comentarios (#) —
        # así el archivo sigue siendo CSV válido (pandas.read_csv(..., comment="#")
        # los ignora solo), pero no se pierde el contexto de cómo se capturó.
        f.write("# ROVER IR SENSOR CHARACTERIZATION\n")
        f.write(f"# Date: {ahora.isoformat(sep=' ')}\n")
        f.write(f"# Surface: {superficie}\n")
        f.write(f"# Number of points: {args.puntos}\n")
        f.write(f"# Samples per point: {args.muestras}\n")
        f.write(f"# Sample interval: {args.intervalo_ms} ms\n")
        f.write(f"# Movement time between points: {args.pausa_s:.0f} s\n")
        writer = csv.writer(f)
        writer.writerow(["surface", "point", "sample", "analog_QTRX_A", "analog_QTRX_B", "generic_IR"])
        f.flush()

        capturadas = 0
        for punto in range(1, args.puntos + 1):
            contar(args.pausa_s)
            print(f"Punto {punto}/{args.puntos} — capturando {args.muestras} muestras...")

            for muestra in range(1, args.muestras + 1):
                analog_a, analog_b, generic = leer_muestra(ser)
                writer.writerow([superficie, punto, muestra, analog_a, analog_b, generic])
                f.flush()  # una muestra por línea en disco, no se pierde nada si algo falla a medio camino
                capturadas += 1
                print(f"\r  [{capturadas}/{total_muestras}] analog_a={analog_a} analog_b={analog_b} generic={generic}   ", end="", flush=True)
                time.sleep(args.intervalo_ms / 1000.0)
            print()

    print(f"\nListo. {capturadas} muestras guardadas en {destino}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
