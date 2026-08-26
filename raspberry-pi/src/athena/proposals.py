"""Etapa 1: proponer regiones candidatas por color y forma.

POR QUÉ ESTE DISEÑO
-------------------
Lo obvio sería tirarle un detector tipo YOLO o SSD al frame completo. En una
Raspberry Pi 4B eso cuesta entre 50 y 80 ms por frame y ocupa los cuatro
núcleos: unos 12-20 FPS, sin CPU sobrante para el control ni para el enlace
serial.

Pero aquí los objetos están *definidos por su color*: el reglamento dice que
las banderas son cilindros rojos o azules. Eso no es un atajo sucio, es
información real del problema. Un umbral en HSV localiza los candidatos en
2-4 ms usando un solo núcleo, y deja que la red neuronal se ocupe solo de lo
que de verdad es difícil: decidir si ese borrón rojo es una bandera o es la
cinta del piso.

Resultado: la localización sale casi gratis y la CNN corre sobre recortes de
64x64 en vez de sobre el frame entero.
"""

from __future__ import annotations

import cv2
import numpy as np

from .config import ColorRange, ProposalConfig
from .types import BBox


class Proposal:
    """Región candidata, con la pista de color que la generó."""

    __slots__ = ("box", "color_hint", "fill_ratio")

    def __init__(self, box: BBox, color_hint: str, fill_ratio: float) -> None:
        self.box = box
        self.color_hint = color_hint      # "rojo" | "azul"
        self.fill_ratio = fill_ratio

    def __repr__(self) -> str:  # pragma: no cover - solo para depurar
        return f"Proposal({self.color_hint}, {self.box}, fill={self.fill_ratio:.2f})"


def _mask_for(hsv: np.ndarray, rng: ColorRange) -> np.ndarray:
    lower = np.array([rng.h_min, rng.s_min, rng.v_min], dtype=np.uint8)
    upper = np.array([rng.h_max, rng.s_max, rng.v_max], dtype=np.uint8)
    return cv2.inRange(hsv, lower, upper)


class ProposalGenerator:
    def __init__(self, cfg: ProposalConfig) -> None:
        self._cfg = cfg
        # El kernel se crea una sola vez: reservarlo en cada frame es basura
        # para el recolector y, a 30 FPS, se nota.
        k = cfg.morph_kernel
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

    def generate(self, frame_bgr: np.ndarray) -> list[Proposal]:
        # Una sola conversión a HSV para todo el frame; las tres máscaras se
        # sacan de ahí. Convertir dos veces sería duplicar el trabajo caro.
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # El rojo está partido en los dos extremos del círculo de tono, por eso
        # necesita dos rangos que luego se unen.
        mask_rojo = cv2.bitwise_or(
            _mask_for(hsv, self._cfg.rojo_bajo),
            _mask_for(hsv, self._cfg.rojo_alto),
        )
        mask_azul = _mask_for(hsv, self._cfg.azul)

        proposals: list[Proposal] = []
        proposals += self._blobs(mask_rojo, "rojo")
        proposals += self._blobs(mask_azul, "azul")

        # Los candidatos grandes primero: un objeto cercano importa más que uno
        # lejano, y si hay que recortar la lista, que se caigan los pequeños.
        proposals.sort(key=lambda p: p.box.area, reverse=True)
        return proposals[: self._cfg.max_proposals]

    def _blobs(self, mask: np.ndarray, color_hint: str) -> list[Proposal]:
        # OPEN quita motas sueltas, CLOSE cierra huecos internos (un reflejo en
        # el centro de la bandera parte el blob en dos si no se hace esto).
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        # connectedComponentsWithStats en vez de findContours: da directamente
        # la caja y el área de cada blob, sin construir listas de puntos que
        # luego habría que recorrer.
        count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

        out: list[Proposal] = []
        for i in range(1, count):        # 0 es el fondo
            x, y, w, h, area = stats[i]

            if area < self._cfg.min_area_px or area > self._cfg.max_area_px:
                continue

            aspect = w / h if h > 0 else 0.0
            if not (self._cfg.min_aspect <= aspect <= self._cfg.max_aspect):
                continue

            # Qué tan lleno está el rectángulo. Un objeto sólido llena su caja;
            # una línea de cinta vista en diagonal, no. Filtro barato y eficaz.
            fill = area / float(w * h) if w * h > 0 else 0.0
            if fill < self._cfg.min_fill_ratio:
                continue

            out.append(Proposal(BBox(int(x), int(y), int(w), int(h)), color_hint, float(fill)))
        return out
