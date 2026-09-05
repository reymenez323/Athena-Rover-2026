"""Enlace serial con el ESP32-S3, con reconexión y descubrimiento de puerto.

Cuando el ESP32 se reinicia, su dispositivo USB desaparece y vuelve a
aparecer. Un programa que abra el puerto una sola vez al arrancar se queda
mudo para siempre después del primer reset del microcontrolador — y eso pasa,
por ejemplo, cada vez que se reprograma el firmware entre rondas.

Por eso esta clase nunca da el puerto por perdido: si se cae, sigue
reintentando en segundo plano mientras el resto del programa continúa. Las
escrituras fallidas se descartan en silencio; la lógica de decisión reenvía
comandos constantemente, así que no vale la pena encolar los viejos.

QUÉ PUERTO ES
-------------
Depende de por dónde esté enchufado el cable, y en la práctica cambió más de
una vez entre sesiones de trabajo:

* **/dev/ttyACM0** — puerto USB **nativo** del ESP32-S3 (GPIO 19/20), que es
  el que usa el firmware de vuelo (``ARDUINO_USB_CDC_ON_BOOT=1``).
* **/dev/ttyUSB0** — puerto de **programación** del DevKitC-1, que pasa por un
  conversor USB-serie (CP2102/CH340) y por eso aparece con otro nombre.

Para que nadie pierda una ronda depurando un nombre de dispositivo,
``serial_port`` acepta el valor especial ``"auto"``: se prueban los candidatos
de ``PUERTOS_CANDIDATOS`` en orden y se usa el primero que abra. Un puerto
explícito en la configuración se respeta tal cual y no se sustituye.

PERMISOS EN RASPBERRY PI OS
---------------------------
Abrir ``/dev/ttyACM*`` o ``/dev/ttyUSB*`` exige pertenecer al grupo
``dialout``. En Raspberry Pi OS Bookworm ya no existe el usuario ``pi`` por
defecto — cada quien crea el suyo al instalar — y un usuario nuevo puede
quedar fuera de ese grupo.

Ese caso se trata aparte a propósito: sin distinguirlo, un "permiso denegado"
es un ``OSError`` como cualquier otro y se confundiría con "el cable no está
puesto", dejando al robot mudo y reintentando en silencio para siempre. Se
arregla una sola vez, y hay que cerrar sesión para que tome efecto::

    sudo usermod -aG dialout $USER
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
    encode_flag_signal,
    encode_gripper,
    encode_led,
    encode_motor,
    encode_stop,
)

log = logging.getLogger(__name__)

_RECONNECT_INTERVAL_S = 1.0

#: Candidatos que se prueban, en orden, cuando ``serial_port`` es ``"auto"``.
#: Primero el USB nativo (el del firmware de vuelo), después el puerto de
#: programación del DevKit.
PUERTOS_CANDIDATOS: tuple[str, ...] = (
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
)

PUERTO_AUTO = "auto"


def _es_permiso_denegado(exc: Exception) -> bool:
    """¿El puerto existe pero el sistema no nos deja abrirlo?

    pyserial a veces envuelve el error del sistema en un ``SerialException``,
    así que no alcanza con mirar el tipo: se revisa también el texto.
    """
    if isinstance(exc, PermissionError):
        return True
    return "permission denied" in str(exc).lower()


class EspLink:
    def __init__(self, port: str = PUERTO_AUTO, baud: int = 115200) -> None:
        self._configured_port = port
        self._port_name = port          # el que se está usando de verdad
        self._baud = baud
        self._serial: serial.Serial | None = None
        self._decoder = PacketDecoder()
        self._last_attempt = 0.0
        self._aviso_permisos = False    # el aviso de 'dialout' se da una vez
        self.telemetry_log: deque[Telemetry] = deque(maxlen=200)

    @property
    def port(self) -> str:
        """El puerto que se está usando ahora (ya resuelto, si era ``auto``)."""
        return self._port_name

    def _candidatos(self) -> tuple[str, ...]:
        if self._configured_port == PUERTO_AUTO:
            return PUERTOS_CANDIDATOS
        return (self._configured_port,)

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

        ultimo_error: Exception | None = None
        for candidato in self._candidatos():
            try:
                # timeout=0 -> lecturas no bloqueantes. El bucle de control no
                # puede permitirse esperar al ESP32: si no hay datos, sigue.
                self._serial = serial.Serial(candidato, self._baud, timeout=0, write_timeout=0.1)
            except (serial.SerialException, OSError) as exc:
                # "Permiso denegado" no es "no está conectado": el puerto SÍ
                # existe y el problema es del sistema, no del cable. Se avisa
                # fuerte y una sola vez, porque si no queda enterrado en un
                # log de depuración mientras el robot parece simplemente mudo.
                if _es_permiso_denegado(exc):
                    if not self._aviso_permisos:
                        self._aviso_permisos = True
                        log.error(
                            "Permiso denegado al abrir %s. Falta estar en el grupo "
                            "'dialout'. Corré:  sudo usermod -aG dialout $USER  "
                            "y volvé a iniciar sesión.", candidato,
                        )
                ultimo_error = exc
                continue
            self._port_name = candidato
            self._decoder = PacketDecoder()   # descartar cualquier trama a medias
            log.info("Conectado al ESP32 en %s", candidato)
            return

        self._serial = None
        log.debug("Sin conexión con el ESP32 (probados: %s): %s",
                  ", ".join(self._candidatos()), ultimo_error)

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

    def send_flag_signal(self, detected: bool) -> bool:
        """Le dice al ESP32 si la cámara ve AHORA la bandera contraria.

        El ESP32 no tiene cámara: sin esto no puede cumplir el reto de
        demostración "detectar la bandera del oponente y señalizar su
        detección". Quien llame a esto solo debería hacerlo cuando el estado
        CAMBIA (ver ``scripts/run_rover.py``): mandarlo en cada cuadro llena
        el cable de tramas idénticas sin aportar nada.
        """
        return self._write(encode_flag_signal(detected))

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
