"""Protocolo serial con el ESP32-S3.

Este archivo es la cara Python de un contrato binario. La otra cara está en
``firmware-esp32/src/main.cpp``, sección [3]. **Si cambias algo aquí, cámbialo
allá.** El test ``tests/test_protocol.py`` lee el .cpp y compara los números,
así que una desincronización rompe el test en vez de romper el robot en plena
competencia.

Trama::

    [0xAA][TYPE][LEN][PAYLOAD ... LEN bytes][CHECKSUM]

    CHECKSUM = XOR de TYPE, LEN y todos los bytes del payload.

Todo entero multi-byte va en little-endian. Los booleanos viajan empacados en
bits de un solo byte: mandar structs crudos entre C++ y Python es pedir que el
padding invisible del compilador te arruine el día.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterator

START_BYTE = 0xAA
MAX_PAYLOAD = 32


class PacketType(IntEnum):
    # RPi -> ESP32
    CMD_MOTOR = 0x01
    CMD_GRIPPER = 0x02
    CMD_LED = 0x03
    # ESP32 -> RPi
    TLM_COLOR = 0x10
    TLM_REFLECT = 0x11
    TLM_HEALTH = 0x12
    TLM_TOF = 0x13


LEN_CMD_MOTOR = 3
LEN_CMD_GRIPPER = 1
LEN_CMD_LED = 1
LEN_TLM_COLOR = 7
LEN_TLM_REFLECT = 9
LEN_TLM_HEALTH = 5
LEN_TLM_TOF = 7


class MotorMode(IntEnum):
    STOP = 0
    DRIVE = 1


class GripperAction(IntEnum):
    OPEN = 0
    CLOSE = 1
    RAISE = 2
    LOWER = 3


class TeamColor(IntEnum):
    NONE = 0
    RED = 1
    BLUE = 2


class ColorLabel(IntEnum):
    """Colores que el TCS34725 puede reconocer en el piso de la pista."""

    UNKNOWN = 0
    BLACK = 1     # cinta del borde: salirse es perder la ronda
    YELLOW = 2    # zona neutra, donde va la llave
    RED = 3       # línea de la zona roja
    BLUE = 4      # línea de la zona azul
    FLOOR = 5     # tapete gris


# Índices de tarea del ESP32 dentro del bitmask de HealthReport.
# Mismo orden que el enum TaskId en main.cpp.
# Tipos que la Raspberry Pi puede RECIBIR. Solo telemetría: los CMD_* van en
# la otra dirección. Validar el byte de tipo contra este conjunto es lo que
# permite al parser resincronizar cuando llega basura por el cable (ver
# PacketDecoder._step).
RECEIVABLE_TYPES = frozenset(
    {PacketType.TLM_COLOR, PacketType.TLM_REFLECT, PacketType.TLM_HEALTH, PacketType.TLM_TOF}
)


# Mismo orden que el enum TaskId en main.cpp. TOF_SENSOR se agregó al final
# allá a propósito (no mueve el bit de nadie más), así que va al final aquí.
TASK_NAMES = (
    "serial_comm",
    "motor_control",
    "gripper_control",
    "color_sensor",
    "reflectance",
    "led_status",
    "tof_sensor",
)


# ---------------------------------------------------------------------------
# Telemetría recibida
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ColorTelemetry:
    timestamp_ms: int
    front: ColorLabel
    back: ColorLabel
    front_valid: bool
    back_valid: bool


@dataclass(frozen=True)
class ReflectTelemetry:
    timestamp_ms: int
    left_raw: int
    right_raw: int
    left_on_line: bool
    right_on_line: bool


@dataclass(frozen=True)
class ToFTelemetry:
    """Distancia del VL53L1X montado delante del gripper.

    ``valid`` es False cuando el sensor no responde o cuando la última
    medición no fue confiable (fuera de rango, señal débil, etc. — lo que el
    firmware traduce de ``VL53L1X::RangeStatus``). No confundir con "no hay
    nada delante": eso da una distancia válida, solo que grande.
    """

    timestamp_ms: int
    distance_mm: int
    valid: bool


@dataclass(frozen=True)
class HealthTelemetry:
    timestamp_ms: int
    faulted_bitmask: int

    @property
    def faulted_tasks(self) -> tuple[str, ...]:
        """Nombres de las tareas que el supervisor del ESP32 vio colgadas."""
        return tuple(
            name
            for i, name in enumerate(TASK_NAMES)
            if self.faulted_bitmask & (1 << i)
        )


Telemetry = ColorTelemetry | ReflectTelemetry | HealthTelemetry | ToFTelemetry


# ---------------------------------------------------------------------------
# Construcción de tramas (RPi -> ESP32)
# ---------------------------------------------------------------------------


def checksum(packet_type: int, payload: bytes) -> int:
    value = packet_type ^ len(payload)
    for byte in payload:
        value ^= byte
    return value & 0xFF


def _frame(packet_type: PacketType, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload de {len(payload)} bytes excede el máximo {MAX_PAYLOAD}")
    return bytes([START_BYTE, int(packet_type), len(payload)]) + payload + bytes(
        [checksum(int(packet_type), payload)]
    )


def encode_motor(mode: MotorMode, left: int, right: int) -> bytes:
    """Comando de motores. `left`/`right` en porcentaje, -100..100."""
    left = max(-100, min(100, int(left)))
    right = max(-100, min(100, int(right)))
    return _frame(PacketType.CMD_MOTOR, struct.pack("<Bbb", int(mode), left, right))


def encode_stop() -> bytes:
    return encode_motor(MotorMode.STOP, 0, 0)


def encode_gripper(action: GripperAction) -> bytes:
    return _frame(PacketType.CMD_GRIPPER, struct.pack("<B", int(action)))


def encode_led(team: TeamColor) -> bytes:
    return _frame(PacketType.CMD_LED, struct.pack("<B", int(team)))


# ---------------------------------------------------------------------------
# Parser incremental (ESP32 -> RPi)
# ---------------------------------------------------------------------------


class PacketDecoder:
    """Máquina de estados que reconstruye tramas a partir de bytes sueltos.

    Un UART entrega los datos partidos donde le da la gana, así que el parser
    no puede asumir que un ``read()`` trae una trama completa. Se le echan los
    bytes que lleguen y devuelve los paquetes que hayan quedado completos.

    Cualquier byte suelto o checksum incorrecto solo cuesta esa trama: el
    decodificador vuelve a buscar el byte de inicio y sigue. Nunca se queda
    trabado, que es justo lo que uno necesita a mitad de una competencia.
    """

    _WAIT_START, _READ_TYPE, _READ_LEN, _READ_PAYLOAD, _READ_CHECKSUM = range(5)

    def __init__(self) -> None:
        self._state = self._WAIT_START
        self._type = 0
        self._len = 0
        self._payload = bytearray()
        self.dropped_frames = 0   # útil para diagnosticar un cable con ruido

    def feed(self, data: bytes) -> Iterator[Telemetry]:
        for byte in data:
            packet = self._step(byte)
            if packet is not None:
                yield packet

    def _step(self, byte: int) -> Telemetry | None:
        if self._state == self._WAIT_START:
            if byte == START_BYTE:
                self._state = self._READ_TYPE

        elif self._state == self._READ_TYPE:
            if byte in RECEIVABLE_TYPES:
                self._type = byte
                self._state = self._READ_LEN
            elif byte == START_BYTE:
                # Basura que termina en 0xAA seguida del 0xAA real de la trama:
                # nos quedamos aquí esperando el tipo verdadero en vez de
                # tragarnos la trama buena.
                pass
            else:
                self._state = self._WAIT_START
                self.dropped_frames += 1

        elif self._state == self._READ_LEN:
            self._len = byte
            if byte > MAX_PAYLOAD:
                self._state = self._WAIT_START      # largo imposible: resincronizar
                self.dropped_frames += 1
            else:
                self._payload.clear()
                self._state = self._READ_CHECKSUM if byte == 0 else self._READ_PAYLOAD

        elif self._state == self._READ_PAYLOAD:
            self._payload.append(byte)
            if len(self._payload) >= self._len:
                self._state = self._READ_CHECKSUM

        elif self._state == self._READ_CHECKSUM:
            payload = bytes(self._payload)
            self._state = self._WAIT_START
            if byte == checksum(self._type, payload):
                return _decode_payload(self._type, payload)
            self.dropped_frames += 1

        return None


def _decode_payload(packet_type: int, payload: bytes) -> Telemetry | None:
    if packet_type == PacketType.TLM_COLOR and len(payload) == LEN_TLM_COLOR:
        timestamp, front, back, flags = struct.unpack("<IBBB", payload)
        return ColorTelemetry(
            timestamp_ms=timestamp,
            front=_safe_color(front),
            back=_safe_color(back),
            front_valid=bool(flags & 0x01),
            back_valid=bool(flags & 0x02),
        )

    if packet_type == PacketType.TLM_REFLECT and len(payload) == LEN_TLM_REFLECT:
        timestamp, left, right, flags = struct.unpack("<IHHB", payload)
        return ReflectTelemetry(
            timestamp_ms=timestamp,
            left_raw=left,
            right_raw=right,
            left_on_line=bool(flags & 0x01),
            right_on_line=bool(flags & 0x02),
        )

    if packet_type == PacketType.TLM_HEALTH and len(payload) == LEN_TLM_HEALTH:
        timestamp, bitmask = struct.unpack("<IB", payload)
        return HealthTelemetry(timestamp_ms=timestamp, faulted_bitmask=bitmask)

    if packet_type == PacketType.TLM_TOF and len(payload) == LEN_TLM_TOF:
        timestamp, distance, flags = struct.unpack("<IHB", payload)
        return ToFTelemetry(
            timestamp_ms=timestamp,
            distance_mm=distance,
            valid=bool(flags & 0x01),
        )

    return None   # tipo desconocido o largo raro: se ignora sin romper nada


def _safe_color(value: int) -> ColorLabel:
    try:
        return ColorLabel(value)
    except ValueError:
        return ColorLabel.UNKNOWN
