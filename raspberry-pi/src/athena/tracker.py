"""Seguimiento de objetos entre frames por solapamiento de cajas (IoU).

Para qué sirve: sin tracking habría que clasificar cada candidato en cada
frame. Con tracking, un objeto se clasifica una vez y conserva su identidad
mientras siga viéndose en el mismo sitio. A 30 FPS y reclasificando cada 8
frames, las llamadas a la red neuronal bajan casi un 90 %.

Además da estabilidad: si en un frame suelto la CNN duda, el track mantiene la
clase que ya tenía en vez de hacer parpadear la decisión — y un robot que
cambia de opinión 30 veces por segundo no llega a ninguna parte.

Es un tracker deliberadamente simple (solo IoU, sin filtro de Kalman): los
objetos de esta competencia están quietos y es la cámara la que se mueve
despacio, así que un emparejamiento por solapamiento sobra.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .config import TrackerConfig
from .types import BBox, ObjectClass


@dataclass
class Track:
    track_id: int
    box: BBox
    cls: ObjectClass
    confidence: float
    color_hint: str
    frames_since_classified: int = 0
    missed: int = 0
    hits: int = 1
    _history: list[ObjectClass] = field(default_factory=list)

    @property
    def confirmed(self) -> bool:
        """Un track recién nacido no se considera fiable hasta verse 2 veces.

        Evita que un destello de un solo frame haga que el robot gire.
        """
        return self.hits >= 2


class Tracker:
    def __init__(self, cfg: TrackerConfig) -> None:
        self._cfg = cfg
        self._tracks: list[Track] = []
        self._next_id = 1

    @property
    def tracks(self) -> list[Track]:
        return [t for t in self._tracks if t.missed == 0]

    def update(self, boxes: list[tuple[BBox, str]]) -> tuple[list[Track], list[Track]]:
        """Empareja las cajas nuevas con los tracks existentes.

        Devuelve (todos_los_tracks_vivos, tracks_que_necesitan_clasificarse).
        """
        unmatched = set(range(len(boxes)))

        for track in self._tracks:
            best_iou = 0.0
            best_idx = -1
            for idx in unmatched:
                iou = track.box.iou(boxes[idx][0])
                if iou > best_iou:
                    best_iou, best_idx = iou, idx

            if best_idx >= 0 and best_iou >= self._cfg.iou_match_threshold:
                track.box = boxes[best_idx][0]
                track.color_hint = boxes[best_idx][1]
                track.missed = 0
                track.hits += 1
                track.frames_since_classified += 1
                unmatched.discard(best_idx)
            else:
                track.missed += 1

        # Cajas que no casaron con nada: son objetos nuevos.
        for idx in sorted(unmatched):
            box, hint = boxes[idx]
            self._tracks.append(
                Track(
                    track_id=self._next_id,
                    box=box,
                    cls=ObjectClass.FONDO,   # sin clasificar todavía
                    confidence=0.0,
                    color_hint=hint,
                    frames_since_classified=self._cfg.reclassify_every,  # clasificar ya
                )
            )
            self._next_id += 1

        # Se descartan los tracks que llevan demasiados frames sin verse.
        self._tracks = [t for t in self._tracks if t.missed <= self._cfg.max_missed_frames]

        alive = [t for t in self._tracks if t.missed == 0]
        needs_classification = [
            t for t in alive if t.frames_since_classified >= self._cfg.reclassify_every
        ]
        return alive, needs_classification

    def apply_classification(self, track: Track, cls: ObjectClass, confidence: float) -> None:
        track.cls = cls
        track.confidence = confidence
        track.frames_since_classified = 0

    def reset(self) -> None:
        self._tracks.clear()
