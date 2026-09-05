#!/usr/bin/env python3
"""Calibración de los sensores de color TCS34725 — Athena Rover 2026.

Coordina con el firmware de ``firmware/`` (mismo ESP32-S3 del rover, pero
subido aparte con los DOS TCS34725 conectados — sin motores, sin PCA9685,
sin QTR): manda el comando ``'R'`` por serial, recibe
``DATA,ok,clear,r,g,b``, y guarda las muestras en un .csv dentro de
``data_logs/``.

POR AHORA SOLO SE CALIBRA UN SENSOR A LA VEZ — el firmware decide cuál con
la constante ``SENSOR_ES_DELANTERO`` en ``firmware/src/main.cpp`` (por
defecto, el delantero). Este script no controla eso; ``--sensor`` es solo
una ETIQUETA para el nombre del archivo y los metadatos — tiene que
coincidir con lo que subiste al ESP32, si no el CSV queda mal rotulado.
Cuando cambies la constante del firmware para calibrar el trasero, pasa
``--sensor TRASERO`` aquí también.

Uso::

    python3 calibrar_color.py --puerto COM5 --superficie AZUL --puntos 4 --muestras 60
    python3 calibrar_color.py --puerto COM5 --superficie AZUL --sensor TRASERO --puntos 4 --muestras 60

Antes de cada punto (incluido el primero), el script cuenta regresivamente
``--pausa-s`` segundos: es el tiempo para acomodar la muestra contra el
sensor que se está calibrando.
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

SENSORES_VALIDOS = ("DELANTERO", "TRASERO")
SENSOR_POR_DEFECTO = "DELANTERO"   # debe coincidir con SENSOR_ES_DELANTERO en firmware/src/main.cpp
SUPERFICIES_ESPERADAS = ("AZUL", "ROJO", "AMARILLO", "NEGRO", "GRIS")


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


def leer_muestra(ser: "serial.Serial", timeout_s: float = 2.0) -> tuple[int, int, int, int, int]:
    """Manda 'R' y parsea DATA,ok,clear,r,g,b."""
    ser.write(b"R")
    limite = time.monotonic() + timeout_s
    while time.monotonic() < limite:
        linea = ser.readline().decode(errors="ignore").strip()
        if linea.startswith("DATA,"):
            partes = linea.split(",")
            if len(partes) != 6:
                continue  # línea corrupta o de sobra, se ignora y se sigue leyendo
            try:
                ok, c, r, g, b = (int(p) for p in partes[1:])
                return ok, c, r, g, b
            except ValueError:
                continue
    raise TimeoutError("El ESP32 no respondió a 'R' a tiempo.")


def contar(segundos: float, sensor: str) -> None:
    """Cuenta regresiva en la consola mientras el usuario acomoda la muestra."""
    restante = segundos
    while restante > 0:
        print(
            f"\r  Sostén la muestra contra el sensor {sensor}... {restante:4.1f} s ",
            end="", flush=True,
        )
        paso = min(1.0, restante)
        time.sleep(paso)
        restante -= paso
    print("\r" + " " * 70 + "\r", end="", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--puerto", required=True, help="puerto serial del ESP32 (ej. COM5, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--sensor", default=SENSOR_POR_DEFECTO, choices=SENSORES_VALIDOS,
        help=(
            "SOLO etiqueta el archivo/metadatos — el firmware es el que decide "
            f"cuál sensor lee de verdad (default: {SENSOR_POR_DEFECTO}, ver el "
            "docstring de este script)"
        ),
    )
    parser.add_argument(
        "--superficie", required=True,
        help=f"nombre de la superficie/color (sugeridos: {', '.join(SUPERFICIES_ESPERADAS)}; cualquier otro texto también sirve)",
    )
    parser.add_argument("--puntos", type=int, default=4, help="cuántos puntos/posiciones medir")
    parser.add_argument("--muestras", type=int, default=60, help="muestras por punto")
    parser.add_argument("--intervalo-ms", type=int, default=250, help="ms entre muestras dentro de un punto")
    parser.add_argument(
        "--pausa-s", type=float, default=10.0,
        help="segundos de pausa antes de cada punto, para reacomodar la muestra",
    )
    args = parser.parse_args()

    DATA_LOGS.mkdir(parents=True, exist_ok=True)
    sensor = args.sensor.strip().upper()
    superficie = args.superficie.strip().upper()
    if superficie not in SUPERFICIES_ESPERADAS:
        print(
            f"Aviso: '{superficie}' no es una de las superficies esperadas "
            f"({', '.join(SUPERFICIES_ESPERADAS)}) — se guarda igual, revisa que no sea un typo.",
        )
    ahora = datetime.now()
    # Un archivo por SUPERFICIE (y por sensor): cada corrida de --superficie
    # distinta cae en su propio CSV, nunca se mezclan colores en un mismo
    # archivo.
    destino = DATA_LOGS / f"COLOR_{superficie}_{sensor}_{ahora:%Y-%m-%d_%H-%M-%S}.csv"

    print(f"Conectando a {args.puerto} @ {args.baud}...")
    print(f"Sensor etiquetado para esta corrida: {sensor} (confirma que coincide con SENSOR_ES_DELANTERO en firmware/src/main.cpp)\n")
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
            # Igual que calibrar_ir.py: si el ESP32 ya estaba corriendo de
            # antes, el 'READY' se mandó una sola vez y nadie lo escuchaba
            # todavía. Se prueba un 'R' directo como respaldo.
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
        f.write("# ROVER COLOR SENSOR CHARACTERIZATION\n")
        f.write(f"# Date: {ahora.isoformat(sep=' ')}\n")
        f.write(f"# Surface: {superficie}\n")
        f.write(f"# Sensor under test (label only, set by firmware): {sensor}\n")
        f.write(f"# Number of points: {args.puntos}\n")
        f.write(f"# Samples per point: {args.muestras}\n")
        f.write(f"# Sample interval: {args.intervalo_ms} ms\n")
        f.write(f"# Movement time between points: {args.pausa_s:.0f} s\n")
        writer = csv.writer(f)
        writer.writerow(["surface", "sensor_under_test", "point", "sample", "ok", "clear", "r", "g", "b"])
        f.flush()

        capturadas = 0
        for punto in range(1, args.puntos + 1):
            contar(args.pausa_s, sensor)
            print(f"Punto {punto}/{args.puntos} — capturando {args.muestras} muestras...")

            for muestra in range(1, args.muestras + 1):
                ok, c, r, g, b = leer_muestra(ser)
                writer.writerow([superficie, sensor, punto, muestra, ok, c, r, g, b])
                f.flush()  # una muestra por línea en disco, no se pierde nada si algo falla a medio camino
                capturadas += 1

                estado = "OK" if ok else "SIN RESPUESTA"
                print(
                    f"\r  [{capturadas}/{total_muestras}] clear={c} r={r} g={g} b={b} ({estado})   ",
                    end="", flush=True,
                )
                time.sleep(args.intervalo_ms / 1000.0)
            print()

    print(f"\nListo. {capturadas} muestras guardadas en {destino}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
