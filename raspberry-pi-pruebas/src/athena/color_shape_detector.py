"""Detector auxiliar de banderas por color + forma, sin aprendizaje automático.

Complementa a ``ei_flag_detector.EiFlagDetector`` (que sí es un modelo
entrenado) para cuando la luz del salón se aleja demasiado de la luz con la
que se entrenó el modelo FOMO: en vez de reconocer la forma del cilindro,
busca blobs del color de equipo contrario en HSV y los filtra por su
**relación de aspecto real** (alto/ancho del cilindro, ver
``GeometryConfig.bandera_altura_mm``/``bandera_diametro_mm``), no por su
tamaño en píxeles — así sirve sin importar a qué distancia esté el cilindro,
y descarta blobs del color correcto pero con forma equivocada (una pared
roja, la casaca de alguien, la zona roja de la pista).

Se aceptan DOS hipótesis de forma, no una: la bandera de pie (alto/ancho ≈
150/50 = 3) y la bandera caída de lado (misma proporción invertida, ≈ 1/3),
por si se cae durante la ronda. Cuál de las dos matcheó queda en
``ColorShapeDetection.orientation``.

NO es un reemplazo del modelo: es más frágil ante fondos del mismo color y
ante un cilindro parcialmente tapado (la relación de aspecto se rompe). La
forma de usarlo es en conjunto — ver ``scripts/prueba_deteccion_color.py``
para probarlo aislado bajo distintas luces antes de mezclarlo con
``EiFlagDetector`` en la lógica de misión.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .config import GeometryConfig
from .types import BBox

ETIQUETA_ROJO = "CILINDRO_ROJO"
ETIQUETA_AZUL = "CILINDRO_AZUL"


@dataclass(frozen=True)
class HsvRange:
    """Un rango en HSV de OpenCV: H en [0,180], S y V en [0,255]."""

    h_min: int
    h_max: int
    s_min: int = 100
    v_min: int = 60
    s_max: int = 255
    v_max: int = 255


# Rangos de partida, pensados para ajustarse por CLI (ver prueba_deteccion_
# color.py --s-min/--v-min) más que para acertar a la primera: el tono (H) no
# cambia mucho entre luces distintas, pero saturación y brillo sí, y son los
# primeros que hay que tocar si el detector no ve nada (luz baja: bajar
# v_min) o ve de más (luz muy blanca/deslavada: subir s_min).
#
# El rojo cruza el 0 en la rueda de HSV, así que necesita dos tramos que
# luego se combinan con OR.
RANGOS_ROJO: tuple[HsvRange, ...] = (
    HsvRange(h_min=0, h_max=10),
    HsvRange(h_min=170, h_max=180),
)
RANGOS_AZUL: tuple[HsvRange, ...] = (
    HsvRange(h_min=100, h_max=130),
)


@dataclass(frozen=True)
class ColorShapeDetection:
    label: str
    confidence: float
    box: BBox
    aspect_ratio: float  # alto/ancho medido, para comparar contra el esperado
    orientation: str     # "de_pie" o "caida" -- cuál de las dos hipótesis ganó


class ColorShapeDetector:
    def __init__(
        self,
        geometry: GeometryConfig | None = None,
        rangos_rojo: tuple[HsvRange, ...] = RANGOS_ROJO,
        rangos_azul: tuple[HsvRange, ...] = RANGOS_AZUL,
        tolerancia_aspecto: float = 0.35,
        area_min_px: int = 150,
        min_confidence: float = 0.5,
    ) -> None:
        geometry = geometry or GeometryConfig()
        # De pie: la caja es más alta que ancha (h/w = altura/diámetro). Caída
        # -- el cilindro tumbado, viéndolo de lado -- es la misma proporción
        # invertida: ahora es el ANCHO el que mide los 150 mm y el alto los
        # 50 mm del diámetro. Se aceptan las dos, sin importar cuál matchee.
        self.aspecto_esperado = geometry.bandera_altura_mm / geometry.bandera_diametro_mm
        self.aspecto_esperado_caida = 1.0 / self.aspecto_esperado
        self._tolerancia = tolerancia_aspecto
        self._area_min = area_min_px
        self._min_confidence = min_confidence
        self._rangos = {ETIQUETA_ROJO: rangos_rojo, ETIQUETA_AZUL: rangos_azul}
        # Elipse en vez de rectángulo: un cilindro visto de lado no tiene
        # esquinas duras, así que la máscara tampoco debería tenerlas.
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def _mascara(self, hsv: np.ndarray, rangos: tuple[HsvRange, ...]) -> np.ndarray:
        mascara = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in rangos:
            mascara |= cv2.inRange(hsv, (r.h_min, r.s_min, r.v_min), (r.h_max, r.s_max, r.v_max))
        # Abrir primero (quita ruido de un píxel) y cerrar después (rellena
        # huecos internos del blob) -- al revés, cerrar primero pegaría ruido
        # suelto al blob real antes de poder quitarlo.
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, self._kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, self._kernel)
        return mascara

    def detect(
        self, frame_bgr: np.ndarray, mascaras: dict[str, np.ndarray] | None = None,
    ) -> list[ColorShapeDetection]:
        """Busca los dos colores de equipo en el frame.

        ``mascaras``, si se pasa un dict vacío, se llena con la máscara
        binaria de cada color -- solo para depuración visual
        (``prueba_deteccion_color.py --ver``), el resto del código lo ignora.
        """
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        detecciones: list[ColorShapeDetection] = []

        for etiqueta, rangos in self._rangos.items():
            mascara = self._mascara(hsv, rangos)
            if mascaras is not None:
                mascaras[etiqueta] = mascara

            contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contorno in contornos:
                area = cv2.contourArea(contorno)
                if area < self._area_min:
                    continue

                x, y, w, h = cv2.boundingRect(contorno)
                if w == 0:
                    continue
                aspecto = h / w
                error_de_pie = abs(aspecto - self.aspecto_esperado) / self.aspecto_esperado
                error_caida = abs(aspecto - self.aspecto_esperado_caida) / self.aspecto_esperado_caida
                if error_de_pie <= error_caida:
                    error_aspecto, orientacion = error_de_pie, "de_pie"
                else:
                    error_aspecto, orientacion = error_caida, "caida"
                if error_aspecto > self._tolerancia:
                    continue

                # Qué tanto del rectángulo llena el contorno: un cilindro
                # real se ve casi como un rectángulo relleno visto de lado;
                # un blob alargado de ruido (un reflejo, un borde de color
                # parecido) tiende a ser mucho más delgado que su caja.
                relleno = area / (w * h)
                confianza = max(0.0, 1.0 - error_aspecto) * relleno
                if confianza < self._min_confidence:
                    continue

                detecciones.append(ColorShapeDetection(
                    label=etiqueta, confidence=confianza, box=BBox(x, y, w, h),
                    aspect_ratio=aspecto, orientation=orientacion,
                ))

        return detecciones

    @staticmethod
    def best(detections: list[ColorShapeDetection], label: str) -> ColorShapeDetection | None:
        candidatas = [d for d in detections if d.label == label]
        return max(candidatas, key=lambda d: d.confidence) if candidatas else None
