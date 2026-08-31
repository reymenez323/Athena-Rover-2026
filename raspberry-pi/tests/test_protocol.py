"""Tests del protocolo serial.

El test importante es ``test_constantes_coinciden_con_el_firmware``: abre el
``main.cpp`` real, extrae los números del protocolo y los compara con los de
Python. Si alguien cambia un código de paquete en un lado y olvida el otro,
falla aquí y no en plena competencia con el robot haciendo cosas raras.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

import pytest  # noqa: E402

from athena import protocol as p  # noqa: E402

FIRMWARE = REPO.parent / "firmware-esp32" / "src" / "main.cpp"


# ---------------------------------------------------------------------------
# Contrato con el firmware
# ---------------------------------------------------------------------------


def _constantes_del_firmware() -> dict[str, int]:
    """Extrae las constexpr del namespace Proto de main.cpp."""
    texto = FIRMWARE.read_text(encoding="utf-8")

    inicio = texto.index("namespace Proto")
    fin = texto.index("\n}", inicio)
    bloque = texto[inicio:fin]

    valores: dict[str, int] = {}
    # constexpr uint8_t NOMBRE = 0x12;   /  NOMBRE = 3;
    for nombre, valor in re.findall(r"constexpr\s+uint8_t\s+(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)", bloque):
        valores[nombre] = int(valor, 0)
    # Miembros del enum PacketType:  CMD_MOTOR   = 0x01,
    for nombre, valor in re.findall(r"^\s*(\w+)\s*=\s*(0x[0-9A-Fa-f]+)\s*,", bloque, re.MULTILINE):
        valores[nombre] = int(valor, 0)
    return valores


@pytest.mark.skipif(not FIRMWARE.exists(), reason="no se encontró el firmware")
def test_constantes_coinciden_con_el_firmware():
    fw = _constantes_del_firmware()

    assert fw["START_BYTE"] == p.START_BYTE
    assert fw["MAX_PAYLOAD"] == p.MAX_PAYLOAD

    assert fw["CMD_MOTOR"] == p.PacketType.CMD_MOTOR
    assert fw["CMD_GRIPPER"] == p.PacketType.CMD_GRIPPER
    assert fw["CMD_LED"] == p.PacketType.CMD_LED
    assert fw["TLM_COLOR"] == p.PacketType.TLM_COLOR
    assert fw["TLM_REFLECT"] == p.PacketType.TLM_REFLECT
    assert fw["TLM_HEALTH"] == p.PacketType.TLM_HEALTH
    assert fw["TLM_TOF"] == p.PacketType.TLM_TOF

    assert fw["LEN_CMD_MOTOR"] == p.LEN_CMD_MOTOR
    assert fw["LEN_CMD_GRIPPER"] == p.LEN_CMD_GRIPPER
    assert fw["LEN_CMD_LED"] == p.LEN_CMD_LED
    assert fw["LEN_TLM_COLOR"] == p.LEN_TLM_COLOR
    assert fw["LEN_TLM_REFLECT"] == p.LEN_TLM_REFLECT
    assert fw["LEN_TLM_HEALTH"] == p.LEN_TLM_HEALTH
    assert fw["LEN_TLM_TOF"] == p.LEN_TLM_TOF


@pytest.mark.skipif(not FIRMWARE.exists(), reason="no se encontró el firmware")
def test_los_largos_declarados_son_los_que_se_generan():
    """Los LEN_* del contrato deben coincidir con lo que produce el encoder."""
    assert len(p.encode_motor(p.MotorMode.DRIVE, 10, -10)) == 4 + p.LEN_CMD_MOTOR
    assert len(p.encode_gripper(p.GripperAction.OPEN)) == 4 + p.LEN_CMD_GRIPPER
    assert len(p.encode_led(p.TeamColor.RED)) == 4 + p.LEN_CMD_LED


# ---------------------------------------------------------------------------
# Codificación
# ---------------------------------------------------------------------------


def test_trama_de_motor_byte_a_byte():
    trama = p.encode_motor(p.MotorMode.DRIVE, 50, -50)
    assert trama[0] == 0xAA
    assert trama[1] == 0x01
    assert trama[2] == 3
    assert trama[3] == 1              # DRIVE
    assert trama[4] == 50
    assert trama[5] == 0xCE           # -50 en complemento a dos
    assert trama[6] == p.checksum(0x01, trama[3:6])


def test_las_velocidades_se_recortan_al_rango_valido():
    trama = p.encode_motor(p.MotorMode.DRIVE, 500, -500)
    assert trama[4] == 100
    assert trama[5] == 0x9C           # -100


# ---------------------------------------------------------------------------
# Decodificación
# ---------------------------------------------------------------------------


def _trama(tipo: int, payload: bytes) -> bytes:
    return bytes([0xAA, tipo, len(payload)]) + payload + bytes([p.checksum(tipo, payload)])


def test_decodifica_telemetria_de_color():
    import struct
    payload = struct.pack("<IBBB", 123456, int(p.ColorLabel.YELLOW), int(p.ColorLabel.BLACK), 0x03)
    decoder = p.PacketDecoder()
    paquetes = list(decoder.feed(_trama(0x10, payload)))

    assert len(paquetes) == 1
    tlm = paquetes[0]
    assert isinstance(tlm, p.ColorTelemetry)
    assert tlm.timestamp_ms == 123456
    assert tlm.front is p.ColorLabel.YELLOW
    assert tlm.back is p.ColorLabel.BLACK
    assert tlm.front_valid and tlm.back_valid


def test_decodifica_reflectancia_con_banderas_parciales():
    import struct
    payload = struct.pack("<IHHB", 900, 3000, 1200, 0x01)   # solo izquierda sobre línea
    paquetes = list(p.PacketDecoder().feed(_trama(0x11, payload)))

    tlm = paquetes[0]
    assert isinstance(tlm, p.ReflectTelemetry)
    assert tlm.left_raw == 3000
    assert tlm.right_raw == 1200
    assert tlm.left_on_line is True
    assert tlm.right_on_line is False


def test_el_bitmask_de_salud_se_traduce_a_nombres():
    import struct
    # bit 1 = motor_control, bit 3 = color_sensor
    payload = struct.pack("<IB", 5000, 0b00001010)
    tlm = list(p.PacketDecoder().feed(_trama(0x12, payload)))[0]

    assert isinstance(tlm, p.HealthTelemetry)
    assert tlm.faulted_tasks == ("motor_control", "color_sensor")


def test_el_bitmask_de_salud_reconoce_el_tof():
    import struct
    payload = struct.pack("<IB", 6000, 0b01000000)   # bit 6 = tof_sensor
    tlm = list(p.PacketDecoder().feed(_trama(0x12, payload)))[0]

    assert tlm.faulted_tasks == ("tof_sensor",)


def test_decodifica_telemetria_de_tof():
    import struct
    payload = struct.pack("<IHB", 321, 87, 0x01)
    tlm = list(p.PacketDecoder().feed(_trama(0x13, payload)))[0]

    assert isinstance(tlm, p.ToFTelemetry)
    assert tlm.timestamp_ms == 321
    assert tlm.distance_mm == 87
    assert tlm.valid is True


def test_tof_invalido_se_distingue_de_sin_lectura():
    import struct
    payload = struct.pack("<IHB", 5, 0, 0x00)
    tlm = list(p.PacketDecoder().feed(_trama(0x13, payload)))[0]

    assert tlm.valid is False


def test_una_trama_partida_en_pedazos_se_reconstruye():
    """Un UART parte los datos donde quiere; el parser debe aguantarlo."""
    import struct
    payload = struct.pack("<IB", 42, 0x01)
    trama = _trama(0x12, payload)

    decoder = p.PacketDecoder()
    recibidos = []
    for byte in trama:                       # de a un byte, el peor caso
        recibidos.extend(decoder.feed(bytes([byte])))

    assert len(recibidos) == 1
    assert recibidos[0].timestamp_ms == 42


def test_un_checksum_malo_descarta_solo_esa_trama():
    import struct
    payload = struct.pack("<IB", 42, 0x01)
    mala = bytearray(_trama(0x12, payload))
    mala[-1] ^= 0xFF                          # corromper el checksum
    buena = _trama(0x12, struct.pack("<IB", 99, 0x02))

    decoder = p.PacketDecoder()
    recibidos = list(decoder.feed(bytes(mala) + buena))

    # La mala se pierde, la siguiente llega bien: el enlace no se traba.
    assert len(recibidos) == 1
    assert recibidos[0].timestamp_ms == 99
    assert decoder.dropped_frames == 1


def test_basura_antes_de_una_trama_valida_no_estorba():
    """Basura terminada en 0xAA no debe tragarse la trama siguiente.

    Es el caso que rompe a un parser ingenuo: el 0xAA final de la basura lo
    mete en "esperando tipo", y entonces el 0xAA que de verdad abre la trama
    buena se interpreta como si fuera el tipo. La trama válida se pierde.

    Se resuelve validando el byte de tipo: si no es un tipo conocido pero sí
    es el byte de inicio, el parser se queda esperando el tipo verdadero.
    """
    import struct
    basura = bytes([0x00, 0xFF, 0x13, 0xAA, 0xAA])   # incluye falsos bytes de inicio
    buena = _trama(0x12, struct.pack("<IB", 7, 0))

    recibidos = list(p.PacketDecoder().feed(basura + buena))
    assert len(recibidos) == 1
    assert recibidos[0].timestamp_ms == 7


def test_un_tipo_desconocido_se_descarta_y_el_parser_sigue():
    import struct
    decoder = p.PacketDecoder()
    list(decoder.feed(bytes([0xAA, 0x77, 3, 1, 2, 3])))   # 0x77 no existe
    assert decoder.dropped_frames >= 1

    recibidos = list(decoder.feed(_trama(0x12, struct.pack("<IB", 55, 0))))
    assert len(recibidos) == 1
    assert recibidos[0].timestamp_ms == 55


def test_los_comandos_no_se_decodifican_del_lado_de_la_pi():
    """La Pi manda los CMD_*, no los recibe: verlos entrar es señal de ruido."""
    decoder = p.PacketDecoder()
    recibidos = list(decoder.feed(p.encode_motor(p.MotorMode.DRIVE, 10, 10)))
    assert recibidos == []


def test_un_largo_imposible_no_cuelga_el_parser():
    decoder = p.PacketDecoder()
    list(decoder.feed(bytes([0xAA, 0x12, 200])))     # 200 > MAX_PAYLOAD
    assert decoder.dropped_frames == 1

    import struct
    recibidos = list(decoder.feed(_trama(0x12, struct.pack("<IB", 1, 0))))
    assert len(recibidos) == 1                        # se recuperó


def test_ida_y_vuelta_de_todos_los_tipos_de_telemetria():
    import struct
    casos = [
        (0x10, struct.pack("<IBBB", 1, 3, 4, 0x03), p.ColorTelemetry),
        (0x11, struct.pack("<IHHB", 2, 100, 200, 0x02), p.ReflectTelemetry),
        (0x12, struct.pack("<IB", 3, 0xFF), p.HealthTelemetry),
        (0x13, struct.pack("<IHB", 4, 250, 0x01), p.ToFTelemetry),
    ]
    flujo = b"".join(_trama(t, pl) for t, pl, _ in casos)
    recibidos = list(p.PacketDecoder().feed(flujo))

    assert len(recibidos) == 4
    for recibido, (_, _, tipo) in zip(recibidos, casos):
        assert isinstance(recibido, tipo)
