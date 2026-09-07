"""Captura de la webcam USB en un hilo aparte.

Por qué un hilo: ``cv2.VideoCapture.read()`` es bloqueante y devuelve el frame
MÁS VIEJO del buffer del driver, no el más nuevo. Si el procesamiento tarda más
que un periodo de cámara, la cola crece y el robot termina reaccionando a lo
que vio hace medio segundo — que en un robot que se mueve es la diferencia
entre agarrar la bandera y empujarla.

La solución es un hilo que lee sin parar y se queda solo con el último frame.
Los frames viejos se descartan. Preferimos ver menos imágenes pero recientes.

Nota sobre el GIL: no lo bloqueamos. ``read()`` de OpenCV libera el GIL
mientras espera al driver, así que este hilo realmente corre en paralelo con
el procesamiento en vez de competir con él.
"""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np

from .config import CameraConfig


class Camera:
    def __init__(self, cfg: CameraConfig) -> None:
        self._cfg = cfg
        self._cap: cv2.VideoCapture | None = None
        self._frame: np.ndarray | None = None
        self._frame_id = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._last_error: str | None = None

    # -- ciclo de vida ------------------------------------------------------

    def open(self) -> None:
        # CAP_V4L2 explícito: si se deja que OpenCV elija, en Linux a veces cae
        # en GStreamer y los ajustes de FOURCC y buffer se ignoran en silencio.
        cap = cv2.VideoCapture(self._cfg.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(
                f"No se pudo abrir la cámara {self._cfg.device}. "
                "Revisa que esté conectada y que aparezca en `ls /dev/video*`."
            )

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._cfg.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._cfg.capture_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._cfg.capture_height)
        cap.set(cv2.CAP_PROP_FPS, self._cfg.fps)
        # Buffer de 1: le pedimos al driver que no acumule frames viejos.
        # No todos los drivers lo respetan, por eso además descartamos aquí.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._cap = cap
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._thread.start()

        # Esperar al primer frame para no devolver None nada más arrancar.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self._frame is not None:
                return
            time.sleep(0.02)
        raise RuntimeError(f"La cámara no entregó ningún frame en 5 s. {self._last_error or ''}")

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self) -> "Camera":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- hilo de captura ----------------------------------------------------

    def _loop(self) -> None:
        assert self._cap is not None
        consecutive_failures = 0

        while self._running:
            ok, frame = self._cap.read()
            if not ok:
                consecutive_failures += 1
                self._last_error = "read() devolvió False"
                # Si la cámara se desconecta no hay que morir: se reintenta
                # despacio. El resto del robot sigue operando a ciegas, que es
                # mejor que un proceso caído a mitad de ronda.
                time.sleep(0.05 if consecutive_failures < 20 else 0.5)
                continue

            consecutive_failures = 0
            with self._lock:
                self._frame = frame
                self._frame_id += 1

    # -- lectura ------------------------------------------------------------

    def read(self) -> tuple[int, np.ndarray] | None:
        """Devuelve (frame_id, frame BGR a resolución de proceso), o None."""
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame
            frame_id = self._frame_id

        target = (self._cfg.process_width, self._cfg.process_height)
        if (frame.shape[1], frame.shape[0]) != target:
            # INTER_AREA es el correcto al reducir: promedia los píxeles en vez
            # de muestrear, así que no se pierden objetos pequeños como la llave.
            frame = cv2.resize(frame, target, interpolation=cv2.INTER_AREA)
        return frame_id, frame

    def read_full(self) -> np.ndarray | None:
        """Frame a resolución de captura completa (para tomar fotos del dataset)."""
        with self._lock:
            return None if self._frame is None else self._frame.copy()
