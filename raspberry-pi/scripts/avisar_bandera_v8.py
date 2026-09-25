#!/usr/bin/env python3
"""Manda al ESP32 lo que ve la cámara -- para standalones/v8-logica-completa/.

Este NO es un script de misión: no manda comandos de motor, no habla el
protocolo de ``docs/protocolo-serial.md`` ni usa la máquina de estados de
``decision.py``. La misión completa la lleva el ESP32; la Pi solo aporta los
ojos (la razón: ver el aviso de ``avisar_bandera_v7.py``, la Pi decidiendo
resultó lenta e irregular para el corte de 500 ms del ESP32).

PROTOCOLO (texto, una línea por cuadro, sin framing ni checksum -- un byte
perdido solo estropea UNA línea y el salto de línea resincroniza):

    B <error> <area>\\n    la bandera del equipo contrario está a la vista
                          error: -100 (borde izquierdo) .. 0 (centrada) .. 100 (borde derecho)
                          area:  % del cuadro que ocupa la bandera (informativo; no decide nada)
    N\\n                   no la ve

El ESP32 responde con un byte por el mismo cable: ``R`` (equipo ROJO) o ``A``
(equipo AZUL), repetido cada 0.5 s, decidido por el switch físico del chasis.
Este script espera ese byte para saber QUÉ bandera buscar; ``--equipo`` solo
sirve para banco sin ESP32.

DETECTORES: primero el modelo de Edge Impulse; si no ve nada, el detector de
color y forma (respaldo, sin red neuronal). OJO: las cajas del modelo salen en
el tamaño REDUCIDO/recortado del Impulse y las del respaldo en el de la cámara,
así que cada error se mide contra el ancho de SU propio cuadro.

CONEXIÓN: al puerto USB NATIVO del ESP32-S3 (no el de programación/UART).

USO::

    python3 scripts/avisar_bandera_v8.py                    # normal: espera el switch del ESP32
    python3 scripts/avisar_bandera_v8.py --equipo rojo      # banco, sin switch/ESP32
    python3 scripts/avisar_bandera_v8.py --verbose          # detalle de cada cuadro
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

import serial  # noqa: E402

from athena.centering import error_horizontal  # noqa: E402
from athena.color_shape_detector import ColorShapeDetector  # noqa: E402
from athena.config import Config  # noqa: E402
from athena.ei_flag_detector import EiFlagDetector  # noqa: E402
from athena.link import PUERTO_AUTO, PUERTOS_CANDIDATOS  # noqa: E402

log = logging.getLogger("avisar_bandera_v8")
_parar = False

ETIQUETA_ROJO = "CILINDRO_ROJO"
ETIQUETA_AZUL = "CILINDRO_AZUL"

# Cada cuánto reintentar la conexión si el puerto no abrió o se cayó.
_RECONNECT_INTERVAL_S = 1.0

# Por debajo de esto el ESP32 (que da por vieja una línea B a los 300 ms) va a
# ver la bandera "parpadear": se avisa en el resumen.
_FPS_MINIMO_RECOMENDADO = 5.0


def _signal_handler(signum, frame) -> None:
    global _parar
    _parar = True
    log.info("Señal recibida, deteniendo...")


class EnlaceEsp32:
    """Conexión con el ESP32: manda las líneas B/N y lee el byte de equipo R/A.

    Reconecta sola si el puerto se cae (cable flojo, ESP32 reiniciado), y solo
    avisa UNA vez por caída para no inundar el log con reintentos.
    """

    def __init__(self, port: str, baud: int) -> None:
        self._configured_port = port
        self._baud = baud
        self._serial: serial.Serial | None = None
        self._last_attempt = 0.0
        self._aviso_dado = False
        self.lineas_enviadas = 0

    @property
    def conectado(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def _candidatos(self) -> tuple[str, ...]:
        if self._configured_port == PUERTO_AUTO:
            return PUERTOS_CANDIDATOS
        return (self._configured_port,)

    def _asegurar_conexion(self) -> None:
        if self.conectado:
            return
        now = time.monotonic()
        if now - self._last_attempt < _RECONNECT_INTERVAL_S:
            return
        self._last_attempt = now
        for candidato in self._candidatos():
            try:
                self._serial = serial.Serial(candidato, self._baud, timeout=0, write_timeout=0.1)
            except (serial.SerialException, OSError):
                continue
            log.info("Conectado al ESP32 en %s", candidato)
            self._aviso_dado = False
            return
        if not self._aviso_dado:
            self._aviso_dado = True
            log.warning("No se pudo abrir ningún puerto (%s) -- reintentando en silencio.",
                        ", ".join(self._candidatos()))

    def _caer(self, motivo: str) -> None:
        log.warning("Se cayó el enlace con el ESP32 (%s) -- reconectando.", motivo)
        try:
            if self._serial is not None:
                self._serial.close()
        except Exception:
            pass
        self._serial = None

    def enviar(self, linea: str) -> None:
        self._asegurar_conexion()
        if not self.conectado:
            return
        try:
            self._serial.write(linea.encode("ascii"))
            self.lineas_enviadas += 1
        except (serial.SerialException, OSError) as exc:
            self._caer(str(exc))

    def leer_equipo(self) -> str | None:
        """'rojo'/'azul' si llegó el byte de equipo del ESP32 desde la última
        llamada (se queda con el último si llegan varios), o None."""
        self._asegurar_conexion()
        if not self.conectado:
            return None
        equipo = None
        try:
            pendientes = self._serial.in_waiting
            if pendientes:
                for b in self._serial.read(pendientes):
                    if b == ord("R"):
                        equipo = "rojo"
                    elif b == ord("A"):
                        equipo = "azul"
        except (serial.SerialException, OSError) as exc:
            self._caer(str(exc))
        return equipo

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None


def _medir(box, frame_w: int, frame_h: int) -> tuple[int, int]:
    """(error -100..100, área % del cuadro) de una caja, contra SU propio cuadro."""
    error = int(round(100 * error_horizontal(box, frame_w)))
    area = int(round(100 * box.area / max(1, frame_w * frame_h)))
    return max(-100, min(100, error)), max(0, min(100, area))


def _medir_ei(box, ei_detector, frame, recorte_centrado: bool) -> tuple[int, int]:
    """Como ``_medir``, pero llevando la caja del modelo al cuadro COMPLETO de la cámara.

    Con el Impulse en "Fit shortest axis" el modelo NO ve el cuadro entero: recorta un
    cuadrado centrado del lado del eje corto (480x480 de un 640x480) y lo reduce. Medir el
    error contra ese cuadrado lo agranda por ancho_camara/lado_recorte (~1.33 en 4:3), y por
    eso el modelo y el respaldo de color (que sí mide contra el cuadro completo) daban
    errores distintos para la MISMA bandera (2026-09-24, visto en el journal: 25 vs 19,
    42 vs 32). Con ``recorte_centrado=False`` (Impulse en "Squash", sin recorte) no se corrige.
    """
    alto, ancho = frame.shape[:2]
    error_f = error_horizontal(box, ei_detector.frame_width)
    area_f = box.area / max(1, ei_detector.frame_width * ei_detector.frame_height)
    if recorte_centrado and ancho != alto:
        lado = min(ancho, alto)
        error_f *= lado / ancho
        area_f *= (lado * lado) / (ancho * alto)
    return (max(-100, min(100, int(round(100 * error_f)))),
            max(0, min(100, int(round(100 * area_f)))))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--equipo", default=None, choices=["rojo", "azul"],
                        help="fuerza el equipo SIN esperar el switch del ESP32 (banco sin switch o sin ESP32). "
                             "Normal: se omite y se espera el byte R/A que manda el ESP32.")
    parser.add_argument("--puerto", default=PUERTO_AUTO,
                        help="puerto serial del ESP32 (USB nativo, no UART). 'auto' prueba ttyACM0/1, ttyUSB0/1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--modelo", default="models/athena_ei_banderas.eim")
    parser.add_argument("--config", default=None)
    parser.add_argument("--min-confianza", type=float, default=0.6)
    parser.add_argument("--ei-sin-recorte", action="store_true",
                        help="el Impulse NO recorta el cuadro (modo Squash): no corrige el error del modelo. "
                             "Por defecto se asume 'Fit shortest axis' (recorte cuadrado centrado).")
    parser.add_argument("--area-cerca", type=float, default=0.10,
                        help="fracción del cuadro (0-1) que debe ocupar la mancha del color de la bandera para "
                             "tratarla como 'MUY cerca' y usar su centro de color en vez del modelo. Sube el "
                             "valor si confunde la franja de la pista con la bandera.")
    parser.add_argument("--resumen-s", type=float, default=5.0,
                        help="cada cuántos segundos imprime el resumen (FPS, líneas enviadas, última detección)")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    cfg = Config.load(args.config)
    enlace = EnlaceEsp32(args.puerto, args.baud)

    if args.equipo is not None:
        equipo = args.equipo
        log.info("Equipo forzado por --equipo: %s (sin esperar el switch del ESP32)", equipo.upper())
    else:
        log.info("Esperando el switch de equipo del ESP32 (byte 'R'/'A' por el USB nativo)...")
        equipo = None
        while equipo is None and not _parar:
            equipo = enlace.leer_equipo()
            if equipo is None:
                time.sleep(0.05)
        if _parar:
            enlace.close()
            return 0
        log.info("Equipo recibido del ESP32: %s", equipo.upper())

    etiqueta_objetivo = ETIQUETA_AZUL if equipo == "rojo" else ETIQUETA_ROJO
    log.info("Buscando: %s", etiqueta_objetivo)

    modelo_path = Path(args.modelo)
    if not modelo_path.is_absolute():
        modelo_path = REPO / modelo_path

    from athena.camera import Camera

    color_detector = ColorShapeDetector(geometry=cfg.geometry)

    frames = 0
    vistos = 0
    frames_resumen = 0
    t_inicio = time.monotonic()
    t_resumen = t_inicio
    ultima_fuente: str | None = None
    ultimo_error = ultima_area = 0
    lineas_al_ultimo_resumen = 0

    try:
        with Camera(cfg.camera) as cam, \
             EiFlagDetector(modelo_path, min_confidence=args.min_confianza) as ei_detector:

            log.info("Cámara y modelo listos. Mandando una línea por cuadro al ESP32.")

            while not _parar:
                nuevo_equipo = enlace.leer_equipo()
                if nuevo_equipo is not None and nuevo_equipo != equipo:
                    equipo = nuevo_equipo
                    etiqueta_objetivo = ETIQUETA_AZUL if equipo == "rojo" else ETIQUETA_ROJO
                    log.warning("El equipo cambió a %s (nuevo switch en el ESP32) -- buscando ahora: %s",
                                equipo.upper(), etiqueta_objetivo)

                frame = cam.read_full()
                if frame is None:
                    time.sleep(0.01)
                    continue

                # 0) bandera MUY cerca (mancha grande de su color): el modelo marca cajas mal
                #    centradas y el detector por forma la rechaza porque el borde la corta;
                # 1) modelo de Edge Impulse; 2) respaldo por color y forma
                fuente = None
                error = area = 0
                alto, ancho = frame.shape[:2]
                cerca = color_detector.detect_cerca(frame, etiqueta_objetivo, args.area_cerca)
                if cerca is not None:
                    fuente = "color-cerca"
                    caja_cerca, fraccion_cerca = cerca
                    error = max(-100, min(100, int(round(100 * error_horizontal(caja_cerca, ancho)))))
                    area = max(0, min(100, int(round(100 * fraccion_cerca))))
                    deteccion_ei = None
                else:
                    deteccion_ei = EiFlagDetector.best(ei_detector.detect(frame), etiqueta_objetivo)
                if fuente == "color-cerca":
                    pass
                elif deteccion_ei is not None:
                    fuente = "modelo"
                    error, area = _medir_ei(deteccion_ei.box, ei_detector, frame, not args.ei_sin_recorte)
                else:
                    respaldo = ColorShapeDetector.best(color_detector.detect(frame), etiqueta_objetivo)
                    if respaldo is not None:
                        fuente = "color+forma"
                        alto, ancho = frame.shape[:2]
                        error, area = _medir(respaldo.box, ancho, alto)

                if fuente is not None:
                    enlace.enviar("B %d %d\n" % (error, area))
                    vistos += 1
                    ultimo_error, ultima_area = error, area
                    log.debug("B %d %d (%s)", error, area, fuente)
                    if fuente != ultima_fuente:
                        log.info("Bandera contraria VISTA (%s): error=%d area=%d%%", fuente, error, area)
                else:
                    enlace.enviar("N\n")
                    if ultima_fuente is not None:
                        log.info("Bandera contraria FUERA de vista")
                ultima_fuente = fuente

                frames += 1
                frames_resumen += 1

                ahora = time.monotonic()
                if ahora - t_resumen >= args.resumen_s:
                    fps = frames_resumen / (ahora - t_resumen)
                    lineas = enlace.lineas_enviadas - lineas_al_ultimo_resumen
                    log.info("resumen: %.1f FPS | %d lineas enviadas | enlace=%s | equipo=%s | ultima B: error=%d area=%d%% | %s",
                             fps, lineas, "OK" if enlace.conectado else "CAIDO", equipo.upper(),
                             ultimo_error, ultima_area,
                             "viendo la bandera" if ultima_fuente else "sin ver la bandera")
                    if fps < _FPS_MINIMO_RECOMENDADO:
                        log.warning("FPS bajo (%.1f < %.0f): el ESP32 da por vieja una línea B a los 300 ms y verá la bandera parpadear.",
                                    fps, _FPS_MINIMO_RECOMENDADO)
                    t_resumen = ahora
                    frames_resumen = 0
                    lineas_al_ultimo_resumen = enlace.lineas_enviadas

    except Exception:
        log.exception("Fallo en el bucle de aviso")
        return 1
    finally:
        enlace.close()
        transcurrido = time.monotonic() - t_inicio
        if frames:
            log.info("%d frames en %.1f s (%.1f FPS), bandera vista en %d", frames, transcurrido,
                     frames / transcurrido, vistos)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
