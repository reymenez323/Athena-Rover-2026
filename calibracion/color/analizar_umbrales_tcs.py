#!/usr/bin/env python3
"""Ajusta los umbrales de ClassifyColor()/detector-tcs contra datos reales.

Carga TODOS los CSV ``COLOR_*_DELANTERO_*.csv`` de ``data_logs/`` (mismo
criterio que ``generar_dataset_knn.py``: agrupa por la columna ``surface``
de cada fila, descarta ``ok=0``), y busca, por descenso de coordenadas, los
9 umbrales que menos errores dan sobre la MISMA estructura de reglas
secuenciales que ``ClassifyColor()`` en ``firmware-esp32/src/main.cpp`` y
``detector-tcs/src/main.cpp`` (clear bajo -> NEGRO; si no, ROJO/AZUL/AMARILLO
por umbrales de r/g/b normalizados en ese orden; si nada calza -> GRIS).

Es EL MISMO tipo de ejercicio que ``calibracion/reflectancia/detector-negro-gris/`` hizo
para NEGRO/GRIS con el sensor IR — probar combinaciones contra los datos
capturados y quedarse con la que menos errores dé — pero con 9 parámetros
en vez de 1, así que en vez de imprimir una tabla a mano en un comentario,
queda como script para poder repetirlo cuando cambien los datos.

Uso::

    python3 analizar_umbrales_tcs.py

No escribe nada por sí solo — imprime los umbrales encontrados y la matriz
de confusión para copiarlos a mano en ``detector-tcs/src/main.cpp`` y
``firmware-esp32/src/main.cpp`` (ClassifyColor()), documentando de dónde
salieron.
"""

from __future__ import annotations

import csv
import glob
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parent
DATA_LOGS = REPO / "data_logs"

# Umbrales de partida: los que ya estaban en ClassifyColor()/detector-tcs,
# documentados ahí como "un punto de partida razonable, NO valores
# calibrados". Sirven como semilla de la búsqueda.
SEMILLA = {
    "CLEAR_NEGRO_MAX": 300,
    "ROJO_R_MIN": 0.45, "ROJO_G_MAX": 0.30, "ROJO_B_MAX": 0.30,
    "AZUL_B_MIN": 0.40, "AZUL_R_MAX": 0.30,
    "AMARILLO_R_MIN": 0.35, "AMARILLO_G_MIN": 0.35, "AMARILLO_B_MAX": 0.25,
}

# Rango y paso de búsqueda para cada parámetro.
RANGOS = {
    "CLEAR_NEGRO_MAX": (50, 1300, 2),
    "ROJO_R_MIN": (0.20, 0.60, 0.002),
    "ROJO_G_MAX": (0.15, 0.50, 0.002),
    "ROJO_B_MAX": (0.15, 0.50, 0.002),
    "AZUL_B_MIN": (0.15, 0.55, 0.002),
    "AZUL_R_MAX": (0.10, 0.45, 0.002),
    "AMARILLO_R_MIN": (0.20, 0.55, 0.002),
    "AMARILLO_G_MIN": (0.20, 0.55, 0.002),
    "AMARILLO_B_MAX": (0.05, 0.40, 0.002),
}

CLASES = ("NEGRO", "AMARILLO", "ROJO", "AZUL", "GRIS")


def cargar_muestras() -> list[tuple[str, float, float, float, int]]:
    archivos = sorted(DATA_LOGS.glob("COLOR_*_DELANTERO_*.csv"))
    if not archivos:
        raise SystemExit(f"No hay CSV en {DATA_LOGS} -- corre calibracion/color/calibrar_color.py primero.")

    muestras = []
    for archivo in archivos:
        with archivo.open(newline="", encoding="utf-8") as f:
            reader = csv.DictReader(l for l in f if not l.startswith("#"))
            for row in reader:
                if int(row["ok"]) != 1:
                    continue
                c = int(row["clear"])
                if c <= 0:
                    continue
                surface = row["surface"].strip().upper()
                if surface not in CLASES:
                    continue
                r = int(row["r"]) / c
                g = int(row["g"]) / c
                b = int(row["b"]) / c
                muestras.append((surface, r, g, b, c))
    return muestras


def clasificar(r: float, g: float, b: float, c: int, p: dict) -> str:
    """Misma estructura que ClassifyColor() en firmware-esp32/src/main.cpp."""
    if c < p["CLEAR_NEGRO_MAX"]:
        return "NEGRO"
    if r > p["ROJO_R_MIN"] and g < p["ROJO_G_MAX"] and b < p["ROJO_B_MAX"]:
        return "ROJO"
    if b > p["AZUL_B_MIN"] and r < p["AZUL_R_MAX"]:
        return "AZUL"
    if r > p["AMARILLO_R_MIN"] and g > p["AMARILLO_G_MIN"] and b < p["AMARILLO_B_MAX"]:
        return "AMARILLO"
    return "GRIS"


def evaluar(muestras, p: dict):
    errores = 0
    confusion = defaultdict(lambda: defaultdict(int))
    for label, r, g, b, c in muestras:
        pred = clasificar(r, g, b, c, p)
        confusion[label][pred] += 1
        if pred != label:
            errores += 1
    return errores, confusion


def imprimir_confusion(muestras, p: dict, titulo: str) -> int:
    errores, confusion = evaluar(muestras, p)
    print(f"\n=== {titulo}: {errores}/{len(muestras)} errores ({100*errores/len(muestras):.1f}%) ===")
    for real in CLASES:
        fila = confusion[real]
        total = sum(fila.values())
        detalle = ", ".join(f"{k}={v}" for k, v in sorted(fila.items(), key=lambda x: -x[1]))
        print(f"  {real:10s} n={total:4d} -> {detalle}")
    return errores


def barrer(muestras, nombre, p, lo, hi, paso):
    """Prueba todos los valores de UN parametro, deja los demas fijos."""
    mejor_p = dict(p)
    mejor_err, _ = evaluar(muestras, p)
    v = lo
    while v <= hi:
        candidato = dict(p)
        candidato[nombre] = round(v, 4) if isinstance(lo, float) else v
        err, _ = evaluar(muestras, candidato)
        if err < mejor_err:
            mejor_err = err
            mejor_p = candidato
        v += paso
    return mejor_p, mejor_err


def optimizar(muestras, semilla: dict, max_pasadas: int = 10) -> dict:
    """Descenso de coordenadas: barre cada parametro por turno, repite
    hasta que una pasada completa no mejore nada."""
    p = dict(semilla)
    for pasada in range(1, max_pasadas + 1):
        mejoro = False
        for nombre, (lo, hi, paso) in RANGOS.items():
            nuevo_p, nuevo_err = barrer(muestras, nombre, p, lo, hi, paso)
            err_actual, _ = evaluar(muestras, p)
            if nuevo_err < err_actual:
                p = nuevo_p
                mejoro = True
        err_total, _ = evaluar(muestras, p)
        print(f"Pasada {pasada}: {err_total}/{len(muestras)} errores ({100*err_total/len(muestras):.2f}%)")
        if not mejoro:
            break
    return p


def main() -> int:
    muestras = cargar_muestras()
    conteo = defaultdict(int)
    for m in muestras:
        conteo[m[0]] += 1
    print(f"Total muestras: {len(muestras)}")
    for k in CLASES:
        print(f"  {k:10s} {conteo[k]}")

    imprimir_confusion(muestras, SEMILLA, "Umbrales ACTUALES (semilla, copiados de ClassifyColor())")

    print("\nBuscando mejores umbrales (descenso de coordenadas)...")
    optimos = optimizar(muestras, SEMILLA)

    imprimir_confusion(muestras, optimos, "Umbrales OPTIMIZADOS")

    print("\nCopiar a Umbral:: en detector-tcs/src/main.cpp y a ClassifyColor() en firmware-esp32/src/main.cpp:")
    for k, v in optimos.items():
        if isinstance(v, float):
            print(f"    constexpr float {k}{' ' * max(1, 15 - len(k))}= {v:.3f}f;")
        else:
            print(f"    constexpr uint16_t {k} = {v};")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
