"""Enlace serial con el ESP32-S3, con reconexión automática.

El ESP32-S3 se conecta por su puerto USB nativo, que la Raspberry Pi ve como
``/dev/ttyACM0``. Detalle importante de ese modo: cuando el ESP32 se reinicia,
el dispositivo USB desaparece y vuelve a aparecer. Un programa que abra el
puerto una sola vez al arrancar se queda mudo para siempre después del primer
reset del microcontrolador — y eso pasa, por ejemplo, cada vez que se les
reprograma el firmware entre rondas.

Por eso esta clase nunca da el puerto por perdido: si se cae, sigue
reintentando en segundo plano mientras el resto del programa continúa. Las
escrituras fallidas se descartan en silencio; la lógica de decisión reenvía
comandos constantemente, así que no vale la pena encolar los viejos.
"""

from __future__ import annotations

import logging
import time
from collections import deque
from typing import Iterator

import serial

from .protocol import (
    GripperAction,
    MotorMode,
    PacketDecoder,
    TeamColor,
    Telemetry,
    encode_gripper,
    encode_led,
    encode_motor,
    encode_stop,
)

log = logging.getLogger(__name__)

_RECONNECT_INTERVAL_S = 1.0


class EspLink:
    def __init__(self, port: str, baud: int = 115200) -> None:
        self._port_name = port
        self._baud = baud
        self._serial: serial.Serial | None = None
        self._decoder = PacketDecoder()
        self._last_attempt = 0.0
        self.telemetry_log: deque[Telemetry] = deque(maxlen=200)

    # -- conexión -----------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def _ensure_connection(self) -> None:
        if self.connected:
            return
        now = time.monotonic()
        if now - self._last_attempt < _RECONNECT_INTERVAL_S:
            return
        self._last_attempt = now

        try:
            # timeout=0 -> lecturas no bloqueantes. El bucle de control no
            # puede permitirse esperar al ESP32: si no hay datos, sigue.
            self._serial = serial.Serial(self._port_name, self._baud, timeout=0, write_timeout=0.1)
            self._decoder = PacketDecoder()   # descartar cualquier trama a medias
            log.info("Conectado al ESP32 en %s", self._port_name)
        except (serial.SerialException, OSError) as exc:
            self._serial = None
            log.debug("Sin conexión con %s: %s", self._port_name, exc)

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.write(encode_stop())   # último gesto: frenar
                self._serial.flush()
            except (serial.SerialException, OSError):
                pass
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "EspLink":
        self._ensure_connection()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- envío --------------------------------------------------------------

    def _write(self, data: bytes) -> bool:
        self._ensure_connection()
        if self._serial is None:
            return False
        try:
            self._serial.write(data)
            return True
        except (serial.SerialException, OSError) as exc:
            log.warning("Se perdió el enlace con el ESP32: %s", exc)
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
            self._serial = None
            return False

    def send_motor(self, left: int, right: int) -> bool:
        return self._write(encode_motor(MotorMode.DRIVE, left, right))

    def send_stop(self) -> bool:
        return self._write(encode_stop())

    def send_gripper(self, action: GripperAction) -> bool:
        return self._write(encode_gripper(action))

    def send_led(self, team: TeamColor) -> bool:
        return self._write(encode_led(team))

    # -- recepción ----------------------------------------------------------

    def poll(self) -> Iterator[Telemetry]:
        """Lee lo que haya llegado y devuelve los paquetes completos."""
        self._ensure_connection()
        if self._serial is None:
            return

        try:
            waiting = self._serial.in_waiting
            if waiting <= 0:
                return
            data = self._serial.read(waiting)
        except (serial.SerialException, OSError) as exc:
            log.warning("Fallo al leer del ESP32: %s", exc)
            self._serial = None
            return

        for packet in self._decoder.feed(data):
            self.telemetry_log.append(packet)
            yield packet

    @property
    def dropped_frames(self) -> int:
        """Tramas descartadas por checksum malo. Si sube, revisa el cable."""
        return self._decoder.dropped_frames
