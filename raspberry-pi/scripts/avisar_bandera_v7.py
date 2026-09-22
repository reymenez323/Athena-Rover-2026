#!/usr/bin/env python3
"""Avisa por serial cuando ve la bandera contraria -- para standalones/v7-mision-completa-camara/.

Este NO es un script de misión: no manda comandos de motor, no habla el
protocolo de ``docs/protocolo-serial.md`` ni el de ``src/athena/link.py``, no
usa la máquina de estados de ``decision.py``. Es deliberadamente lo más
simple posible -- cámara + modelo de Edge Impulse (+ respaldo color+forma) +
un byte por serial cada vez que ve la bandera del equipo contrario. Nada más.

Por qué existe aparte de ``run_rover.py``: la primera prueba completa en
pista mostró al robot completamente errático corriendo la máquina de estados
entera en la Raspberry Pi -- probablemente porque el ESP32-S3 corta los
motores solo si pasan 500ms sin un comando nuevo, y el bucle de
``run_rover.py`` corre DOS detectores por cuadro además de la cámara y el
resto de la lógica, lo bastante lento como para disparar ese failsafe por su
cuenta. La solución fue devolverle el control a
``standalones/v7-mision-completa-camara/`` (copia de v6, que sí funcionaba
bien en banco) y reducir el trabajo de la Pi a avisar "la veo" -- si este
script se cuelga, se atrasa, o ni siquiera está conectado, el ESP32 sigue la
misión igual, a ciegas, con el mismo comportamiento de v6.

EQUIPO: normalmente NO se pasa por línea de comandos. El switch físico de 3
posiciones del chasis ya decide ROJO/AZUL dentro del propio
v7-mision-completa-camara (ver setup() en su main.cpp), que le manda ese
mismo byte ('R'/'A') a este script por el mismo CAM_LINK -- así el equipo
nunca se puede desincronizar entre los dos lados. ``--equipo`` sigue
existiendo solo para banco sin switch instalado o sin ESP32 conectado.

CONEXIÓN: al puerto USB NATIVO del ESP32-S3 (no el de programación/UART) --
ver "SEÑAL DE CÁMARA" en standalones/v7-mision-completa-camara/src/main.cpp.

USO::

    python3 scripts/avisar_bandera_v7.py                    # normal: espera el switch del ESP32
    python3 scripts/avisar_bandera_v7.py --equipo rojo       # banco, sin switch/ESP32
    python3 scripts/avisar_bandera_v7.py --puerto /dev/ttyACM0
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

from athena.color_shape_detector import ColorShapeDetector  # noqa: E402
from athena.config import Config  # noqa: E402
from athena.ei_flag_detector import EiFlagDetector  # noqa: E402
from athena.link import PUERTO_AUTO, PUERTOS_CANDIDATOS  # noqa: E402

log = logging.getLogger("avisar_bandera_v7")
_parar = False

ETIQUETA_ROJO = "CILINDRO_ROJO"
ETIQUETA_AZUL = "CILINDRO_AZUL"

# Cada cuánto reintentar la conexión si el puerto no abrió o se cayó --
# mismo valor que athena.link.EspLink, para no martillar el puerto.
_RECONNECT_INTERVAL_S = 1.0


def _signal_handler(signum, frame) -> None:
    global _parar
    _parar = True
    log.info("Señal recibida, deteniendo...")


class SenalSerial:
    """Conexión con el ESP32: manda 'V' por detección, recibe 'R'/'A' de equipo.

    Deliberadamente sin el framing/checksum de ``athena.link.EspLink`` --
    ver el aviso grande en el módulo. Reconecta sola si el puerto se cae
    (cable flojo, ESP32 reiniciado), igual de silenciosa que EspLink para
    no inundar el log con reintentos.
    """

    def __init__(self, port: str, baud: int) -> None:
        self._configured_port = port
        self._baud = baud
        self._serial: serial.Serial | None = None
        self._last_attempt = 0.0
        self._aviso_dado = False

    def _candidatos(self) -> tuple[str, ...]:
        if self._configured_port == PUERTO_AUTO:
            return PUERTOS_CANDIDATOS
        return (self._configured_port,)

    def _asegurar_conexion(self) -> None:
        if self._serial is not None and self._serial.is_open:
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

    def avisar_bandera_vista(self) -> None:
        self._asegurar_conexion()
        if self._serial is None or not self._serial.is_open:
            return
        try:
            self._serial.write(b"V")
        except (serial.SerialException, OSError):
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def leer_equipo(self) -> str | None:
        """Drena lo que haya llegado del ESP32 y devuelve 'rojo'/'azul' si
        trae el byte de equipo ('R'/'A', ver CamaraBandera::EnviarEquipo()
        en el firmware), o None si no llegó nada nuevo. Si llegan varios
        bytes de golpe se queda con el último -- mismo criterio que usa el
        propio ESP32 para 'V' en CamaraBandera::Actualizar()."""
        self._asegurar_conexion()
        if self._serial is None or not self._serial.is_open:
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
        except (serial.SerialException, OSError):
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        return equipo

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--equipo", default=None, choices=["rojo", "azul"],
                        help="fuerza el equipo SIN esperar el switch del ESP32 (banco de pruebas, "
                             "sin switch instalado o sin ESP32 conectado). Normal: se omite, y se "
                             "espera el byte 'R'/'A' que manda v7-mision-completa-camara tras leer "
                             "su switch físico -- así nunca se puede desincronizar del switch real.")
    parser.add_argument("--puerto", default=PUERTO_AUTO,
                        help="puerto serial del ESP32 (USB nativo, no UART). 'auto' prueba ttyACM0/1, ttyUSB0/1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--modelo", default="models/athena_ei_banderas.eim")
    parser.add_argument("--config", default=None)
    parser.add_argument("--min-confianza", type=float, default=0.6)
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
    senal = SenalSerial(args.puerto, args.baud)

    if args.equipo is not None:
        equipo = args.equipo
        log.info("Equipo forzado por --equipo: %s (sin esperar el switch del ESP32)", equipo.upper())
    else:
        log.info("Esperando el switch de equipo del ESP32 (byte 'R'/'A' por CAM_LINK)...")
        equipo = None
        while equipo is None and not _parar:
            equipo = senal.leer_equipo()
            if equipo is None:
                time.sleep(0.05)
        if _parar:
            senal.close()
            return 0
        log.info("Equipo recibido del ESP32: %s", equipo.upper())

    etiqueta_objetivo = ETIQUETA_AZUL if equipo == "rojo" else ETIQUETA_ROJO
    log.info("Avisando cuando vea: %s", etiqueta_objetivo)

    modelo_path = Path(args.modelo)
    if not modelo_path.is_absolute():
        modelo_path = REPO / modelo_path

    from athena.camera import Camera

    color_detector = ColorShapeDetector(geometry=cfg.geometry)

    frames = 0
    ultima_fuente: str | None = None
    t_inicio = time.monotonic()

    try:
        with Camera(cfg.camera) as cam, \
             EiFlagDetector(modelo_path, min_confidence=args.min_confianza) as ei_detector:

            while not _parar:
                nuevo_equipo = senal.leer_equipo()
                if nuevo_equipo is not None and nuevo_equipo != equipo:
                    equipo = nuevo_equipo
                    etiqueta_objetivo = ETIQUETA_AZUL if equipo == "rojo" else ETIQUETA_ROJO
                    log.warning("Equipo cambio a %s (nuevo switch en el ESP32) -- avisando ahora: %s",
                                equipo.upper(), etiqueta_objetivo)

                frame = cam.read_full()
                if frame is None:
                    time.sleep(0.01)
                    continue

                detecciones = ei_detector.detect(frame)
                objetivo_ei = EiFlagDetector.best(detecciones, etiqueta_objetivo)

                if objetivo_ei is not None:
                    fuente = "modelo"
                else:
                    color_detecciones = color_detector.detect(frame)
                    objetivo_color = ColorShapeDetector.best(color_detecciones, etiqueta_objetivo)
                    fuente = "color+forma" if objetivo_color is not None else None

                if fuente is not None:
                    senal.avisar_bandera_vista()
                    if fuente != ultima_fuente:
                        log.info("Bandera contraria vista -> aviso mandado (%s)", fuente)
                elif ultima_fuente is not None:
                    log.info("Bandera contraria fuera de vista")
                ultima_fuente = fuente

                frames += 1

    except Exception:
        log.exception("Fallo en el bucle de aviso")
        return 1
    finally:
        senal.close()
        transcurrido = time.monotonic() - t_inicio
        if frames:
            log.info("%d frames en %.1f s (%.1f FPS)", frames, transcurrido, frames / transcurrido)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
