#!/usr/bin/env python3
"""Pre-etiqueta fotos de banderas para subir a Edge Impulse.

CONTEXTO: las fotos que llegan de otro equipo no traen ninguna etiqueta, son
.jpg sueltos. Dibujar cientos de cajas a mano en Edge Impulse Studio es lento.
Este script hace un primer pase automático usando el MISMO detector de color
que corre en el robot (``athena.proposals.ProposalGenerator``): busca el blob
rojo o azul más grande de cada foto y genera su bounding box.

No adivina la clase por el nombre de la carpeta, mira el color de verdad. Así
también sirve para carpetas mezcladas (fotos ya exportadas de un proyecto de
Edge Impulse, sin organizar por clase), y de paso avisa cuando una foto que
"debería" tener bandera en realidad no mostró ningún color reconocible: son
justo las que hay que revisar a mano.

Salida (en --output):
    bounding_boxes.labels    súbelo junto con las fotos a Edge Impulse
                             (Data acquisition -> arrastrar fotos + este
                             archivo; Studio reconoce el formato solo)
    previews/                copia de cada foto CON la caja dibujada, para
                             revisar rápido si el auto-etiquetado acertó
    revisar_manualmente.txt  fotos donde no se detectó ningún color confiable
                             en una carpeta donde sí se esperaba encontrarlo

USO::

    python3 scripts/auto_label_ei.py --input "carpeta/con/fotos" --output data/ei_labels
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

import cv2  # noqa: E402

from athena.config import ProposalConfig  # noqa: E402
from athena.proposals import Proposal, ProposalGenerator  # noqa: E402

EXTENSIONES = (".jpg", ".jpeg", ".png")

# Nombres EXACTOS que ya usó el modelo del compañero (model_variables.h del
# export de Edge Impulse: CILINDRO_AZUL, CILINDRO_ROJO). Se reusan para que
# este dataset sea compatible con ese mismo proyecto si se decide seguir
# entrenándolo ahí en vez de empezar uno nuevo.
ETIQUETA_ROJO = "CILINDRO_ROJO"
ETIQUETA_AZUL = "CILINDRO_AZUL"

# Carpetas cuyo nombre ya indica que son negativos: no tener ninguna
# detección ahí es lo esperado, no un fallo que haya que revisar.
CARPETAS_FONDO = {"BACKGROUND+DATA", "FONDO", "BACKGROUND"}


def encontrar_imagenes(root: Path) -> list[Path]:
    return sorted(p for p in root.rglob("*") if p.suffix.lower() in EXTENSIONES)


def mejor_caja(propuestas: list[Proposal], color_hint: str) -> Proposal | None:
    candidatas = [p for p in propuestas if p.color_hint == color_hint]
    if not candidatas:
        return None
    return max(candidatas, key=lambda p: p.box.area)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", required=True, help="carpeta con las fotos (busca en subcarpetas)")
    parser.add_argument("--output", required=True, help="carpeta donde se guardan resultados")
    parser.add_argument("--min-area", type=int, default=None, help="sobrescribe ProposalConfig.min_area_px")
    args = parser.parse_args()

    entrada = Path(args.input)
    salida = Path(args.output)
    previews = salida / "previews"
    previews.mkdir(parents=True, exist_ok=True)

    cfg = ProposalConfig()
    if args.min_area is not None:
        cfg = replace(cfg, min_area_px=args.min_area)
    generator = ProposalGenerator(cfg)

    imagenes = encontrar_imagenes(entrada)
    if not imagenes:
        raise SystemExit(f"No se encontraron fotos en {entrada}")

    bounding_boxes: dict[str, list[dict]] = {}
    sin_deteccion: list[str] = []
    conteo = {ETIQUETA_ROJO: 0, ETIQUETA_AZUL: 0, "fondo": 0}

    for ruta in imagenes:
        img = cv2.imread(str(ruta))
        if img is None:
            continue

        carpeta_origen = ruta.parent.name.strip().upper()
        se_esperaba_objeto = carpeta_origen not in CARPETAS_FONDO

        cajas_foto: list[dict] = []
        if se_esperaba_objeto:
            # Solo se corre el detector de color en carpetas donde SÍ puede
            # haber bandera. En "fondo" confiamos en la carpeta, no en el
            # color: un reflejo o la piel/pelo de alguien puede disparar el
            # detector igual, y eso metería cajas falsas justo en la clase de
            # negativos que más nos importa tener limpia.
            propuestas = generator.generate(img)
            for color_hint, etiqueta in (("rojo", ETIQUETA_ROJO), ("azul", ETIQUETA_AZUL)):
                p = mejor_caja(propuestas, color_hint)
                if p is not None:
                    cajas_foto.append({
                        "label": etiqueta,
                        "x": p.box.x, "y": p.box.y,
                        "width": p.box.w, "height": p.box.h,
                    })

        nombre = ruta.name
        bounding_boxes[nombre] = cajas_foto

        if cajas_foto:
            for c in cajas_foto:
                conteo[c["label"]] += 1
            vista = img.copy()
            for c in cajas_foto:
                color_dibujo = (0, 0, 255) if c["label"] == ETIQUETA_ROJO else (255, 0, 0)
                cv2.rectangle(
                    vista, (c["x"], c["y"]), (c["x"] + c["width"], c["y"] + c["height"]),
                    color_dibujo, 2,
                )
            cv2.imwrite(str(previews / nombre), vista)
        else:
            conteo["fondo"] += 1
            if se_esperaba_objeto:
                sin_deteccion.append(f"{ruta.parent.name}/{nombre}")

    (salida / "bounding_boxes.labels").write_text(
        json.dumps({"version": 1, "type": "bounding-box-labels", "boundingBoxes": bounding_boxes}, indent=2),
        encoding="utf-8",
    )
    (salida / "revisar_manualmente.txt").write_text("\n".join(sin_deteccion), encoding="utf-8")

    print(f"Fotos procesadas: {len(imagenes)}")
    print(f"  {ETIQUETA_ROJO:15s} {conteo[ETIQUETA_ROJO]}")
    print(f"  {ETIQUETA_AZUL:15s} {conteo[ETIQUETA_AZUL]}")
    print(f"  sin objeto (fondo) {conteo['fondo']}")
    print(f"\nPara revisar a mano: {len(sin_deteccion)} fotos -> {salida / 'revisar_manualmente.txt'}")
    print(f"Previsualizaciones con caja dibujada en: {previews}")
    print(f"Archivo para subir a Edge Impulse: {salida / 'bounding_boxes.labels'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
