#!/usr/bin/env python3
"""Genera dataset_color.h a partir de las capturas de data_logs/.

Lee TODOS los CSV ``COLOR_*_DELANTERO_*.csv`` en ``data_logs/``, agrupa las
muestras por el color real (columna ``surface`` de cada FILA, no el nombre
del archivo — así una corrida interrumpida o repetida simplemente suma más
muestras válidas a esa clase, no rompe nada ni hay que limpiar archivos a
mano), descarta las filas con ``ok=0`` (el sensor no respondió), y escribe
un header de C++ con el dataset embebido para un clasificador K-NN escrito
a mano.

NOTA: ``pruebas-platformio/05-evitador-linea/`` ya NO usa este header —
se cambió a los umbrales de ``detector-tcs/`` para que las dos
herramientas se comporten igual (ver ``../README.md``). Este script queda
disponible por si hace falta la precisión extra del K-NN en otro lado; por
defecto escribe dentro de esta misma carpeta, usar ``--salida`` para
apuntarlo a un sketch en particular.

CUATRO features por muestra, no tres: además de los canales normalizados
contra ``clear`` (r/clear, g/clear, b/clear — mismo criterio que
``ClassifyColor()`` en firmware-esp32/), se agrega ``clear`` ESCALADO
0..1 (min-max sobre el propio dataset) como cuarta dimensión. Se
comprobó con los datos reales de este equipo que descartar `clear`
tira a la basura la señal que mejor separa AMARILLO del resto (mediana
~4511 contra ~350-950 de los demás colores) — sin él, dos superficies con
el mismo tono pero brillo muy distinto (por ejemplo NEGRO muy oscuro vs.
AMARILLO muy iluminado, si sus RATIOS r/g/b resultan parecidos) podrían
confundirse. Se ESCALA (no se usa crudo) porque su magnitud (cientos a
miles) aplastaría a los ratios (0..1) en la distancia euclidiana si se
mezclaran sin normalizar — kClearMin/kClearMax quedan embebidos en el
header para que el sketch aplique la MISMA escala a las lecturas en vivo.

Volver a correr este script cada vez que cambien los CSV de ``data_logs/``
(nueva captura, recalibración, datos del sensor trasero más adelante) y
volver a subir el sketch — el header generado NO se actualiza solo.

Uso::

    python3 generar_dataset_knn.py
    python3 generar_dataset_knn.py --salida otra/ruta/dataset_color.h
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

REPO = Path(__file__).resolve().parent
DATA_LOGS = REPO / "data_logs"
SALIDA_DEFECTO = REPO / "dataset_color.h"   # usar --salida para apuntar a un sketch en particular

# Nombre del enumerador ColorLabel (definido en el propio header generado)
# que le corresponde a cada superficie capturada.
ETIQUETAS = {
    "NEGRO": "ColorLabel::NEGRO",
    "AMARILLO": "ColorLabel::AMARILLO",
    "ROJO": "ColorLabel::ROJO",
    "AZUL": "ColorLabel::AZUL",
    "GRIS": "ColorLabel::GRIS",
}


def cargar_muestras() -> list[tuple[str, float, float, float, int]]:
    """Devuelve (surface, r_norm, g_norm, b_norm, clear_crudo) por fila válida."""
    archivos = sorted(DATA_LOGS.glob("COLOR_*_DELANTERO_*.csv"))
    if not archivos:
        raise SystemExit(f"No hay CSV en {DATA_LOGS} -- corre calibracion/color/calibrar_color.py primero.")

    muestras: list[tuple[str, float, float, float, int]] = []
    conteo: dict[str, int] = {}

    for archivo in archivos:
        with archivo.open(newline="", encoding="utf-8") as f:
            reader = csv.DictReader(l for l in f if not l.startswith("#"))
            for row in reader:
                if int(row["ok"]) != 1:
                    continue   # el sensor no respondió en esa muestra, no es un "negro" real
                clear = int(row["clear"])
                if clear <= 0:
                    continue   # división por cero / lectura inválida, se descarta
                surface = row["surface"].strip().upper()
                if surface not in ETIQUETAS:
                    print(f"Aviso: '{surface}' en {archivo.name} no es una superficie reconocida, se ignora esa fila.")
                    continue
                r = int(row["r"]) / clear
                g = int(row["g"]) / clear
                b = int(row["b"]) / clear
                muestras.append((surface, r, g, b, clear))
                conteo[surface] = conteo.get(surface, 0) + 1

    print("Muestras cargadas por color (de todos los CSV combinados):")
    for color in ETIQUETAS:
        print(f"  {color:10s} {conteo.get(color, 0)}")

    faltantes = [c for c in ETIQUETAS if c not in conteo]
    if faltantes:
        print(
            f"\nAVISO: falta al menos 1 muestra valida de {faltantes} -- "
            "el clasificador JAMAS podra elegir esa clase, va a confundirla "
            "con la mas parecida de las que si tiene datos.",
        )

    return muestras


def escribir_header(muestras: list[tuple[str, float, float, float, int]], salida: Path) -> None:
    clear_min = min(m[4] for m in muestras)
    clear_max = max(m[4] for m in muestras)
    rango = clear_max - clear_min

    print(f"\nRango de 'clear' en el dataset: {clear_min}..{clear_max} (usado para escalar la 4ta dimension del KNN)")

    salida.parent.mkdir(parents=True, exist_ok=True)
    with salida.open("w", encoding="utf-8", newline="\n") as f:
        f.write("// ===========================================================================\n")
        f.write("//  GENERADO AUTOMATICAMENTE por calibracion/color/generar_dataset_knn.py\n")
        f.write("//  NO EDITAR A MANO. Volver a correr el script si cambian los CSV de\n")
        f.write("//  calibracion/color/data_logs/, y volver a subir el sketch.\n")
        f.write("// ===========================================================================\n")
        f.write(f"//\n//  {len(muestras)} muestras del sensor TCS34725 DELANTERO. 4 features por\n")
        f.write("//  muestra: r/clear, g/clear, b/clear (mismo criterio que ClassifyColor() en\n")
        f.write("//  firmware-esp32/) MAS 'clear' escalado 0..1 (min-max sobre este dataset) --\n")
        f.write("//  sin 'clear' se pierde la señal que mejor separa AMARILLO del resto (ver el\n")
        f.write("//  docstring de generar_dataset_knn.py). kClearMin/kClearMax quedan acá para\n")
        f.write("//  que el sketch escale las lecturas en vivo con la MISMA fórmula.\n")
        f.write("//\n")
        f.write("// ===========================================================================\n\n")
        f.write("#pragma once\n\n")
        f.write("#include <cstddef>\n")
        f.write("#include <cstdint>\n\n")
        f.write("enum class ColorLabel : uint8_t { NEGRO, AMARILLO, ROJO, AZUL, GRIS };\n\n")
        f.write("struct MuestraColor { float r, g, b, clearNorm; ColorLabel label; };\n\n")
        f.write(f"constexpr uint16_t kClearMin = {clear_min};\n")
        f.write(f"constexpr uint16_t kClearMax = {clear_max};\n\n")
        f.write("constexpr MuestraColor kDatasetColor[] = {\n")
        for surface, r, g, b, clear in muestras:
            clear_norm = (clear - clear_min) / rango
            f.write(f"    {{{r:.5f}f, {g:.5f}f, {b:.5f}f, {clear_norm:.5f}f, {ETIQUETAS[surface]}}},\n")
        f.write("};\n\n")
        f.write("constexpr size_t kDatasetColorSize = sizeof(kDatasetColor) / sizeof(kDatasetColor[0]);\n")

    print(f"Escrito: {salida} ({len(muestras)} muestras)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--salida", type=Path, default=SALIDA_DEFECTO, help="dónde escribir el header generado")
    args = parser.parse_args()

    muestras = cargar_muestras()
    escribir_header(muestras, args.salida)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
