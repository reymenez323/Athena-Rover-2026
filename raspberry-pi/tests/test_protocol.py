"""Tests del protocolo serial.

El test importante es el bloque "Contrato con el firmware": abre los
``main.cpp`` reales, extrae los números del protocolo y los compara con los de
Python. Si alguien cambia un código de paquete en un lado y olvida el otro,
falla aquí y no en plena competencia con el robot haciendo cosas raras.

La comparación es EXHAUSTIVA a propósito, y en las dos direcciones: no hay una
lista de constantes escrita a mano que alguien pueda olvidar ampliar. Eso ya
pasó de verdad — ``CMD_FLAG_SIGNAL`` vivió varios commits existiendo solo en
el firmware, porque el test de entonces enumeraba los paquetes uno por uno y
nadie agregó el nuevo a esa lista.
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
FIRMWARE_STANDALONE = REPO.parent / "firmware-esp32-standalone" / "src" / "main.cpp"


# ---------------------------------------------------------------------------
# Contrato con el firmware
# ---------------------------------------------------------------------------


def _constantes_del_firmware() -> dict[str, int]:
    """Extrae las constexpr y los miembros del enum del namespace Proto."""
    texto = FIRMWARE.read_text(encoding="utf-8")
    inicio = texto.index("namespace Proto")
    bloque = texto[inicio:texto.index("\n}", inicio)]

    valores: dict[str, int] = {}
    # constexpr uint8_t NOMBRE = 0x12;   /  NOMBRE = 3;
    for nombre, valor in re.findall(
        r"constexpr\s+uint8_t\s+(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)", bloque
    ):
        valores[nombre] = int(valor, 0)
    # Miembros del enum PacketType:  CMD_MOTOR   = 0x01,
    for nombre, valor in re.findall(
        r"^\s*(\w+)\s*=\s*(0x[0-9A-Fa-f]+)\s*,", bloque, re.MULTILINE
    ):
        valores[nombre] = int(valor, 0)
    return valores


def _enum_del_firmware(ruta: Path, nombre: str) -> dict[str, int]:
    """Lee un ``enum class X : uint8_t { A = 0, B = 1 };`` de un main.cpp.

    Los miembros sin valor explícito toman el siguiente correlativo, igual que
    en C++, para no obligar a que el firmware los escriba todos.
    """
    texto = ruta.read_text(encoding="utf-8")
    cuerpo = re.search(
        r"enum\s+class\s+" + nombre + r"\s*:\s*uint8_t\s*\{(.*?)\}", texto, re.DOTALL
    )
    assert cuerpo is not None, "no se encontró 'enum class " + nombre + "' en " + ruta.name

    valores: dict[str, int] = {}
    siguiente = 0
    for miembro in cuerpo.group(1).split(","):
        miembro = re.sub(r"//.*", "", miembro).strip()
        if not miembro:
            continue
        if "=" in miembro:
            clave, crudo = (parte.strip() for parte in miembro.split("=", 1))
            siguiente = int(crudo, 0)
        else:
            clave = miembro
        valores[clave] = siguiente
        siguiente += 1
    return valores


@pytest.mark.skipif(not FIRMWARE.exists(), reason="no se encontró el firmware")
def test_todos_los_tipos_de_paquete_coinciden():
    """Ni un paquete de más ni uno de menos, en ninguno de los dos lados."""
    fw = _constantes_del_firmware()

    assert fw["START_BYTE"] == p.START_BYTE
    assert fw["MAX_PAYLOAD"] == p.MAX_PAYLOAD

    # El firmware es la referencia: todo lo que declare como CMD_/TLM_ tiene
    # que existir en Python con el mismo valor...
    tipos_firmware = {
        nombre: valor
        for nombre, valor in fw.items()
        if nombre.startswith(("CMD_", "TLM_"))
    }
    assert tipos_firmware, "no se extrajo ningún tipo de paquete del firmware"

    for nombre, valor in tipos_firmware.items():
        assert hasattr(p.PacketType, nombre), (
            f"{nombre} existe en el firmware pero no en protocol.py"
        )
        actual = int(getattr(p.PacketType, nombre))
        assert actual == valor, (
            f"{nombre}: firmware=0x{valor:02X} != python=0x{actual:02X}"
        )

    # ...y al revés: Python no puede inventarse paquetes que el ESP32 ignora.
    for miembro in p.PacketType:
        assert miembro.name in tipos_firmware, (
            f"{miembro.name} existe en protocol.py pero no en el firmware"
        )


@pytest.mark.skipif(not FIRMWARE.exists(), reason="no se encontró el firmware")
def test_todos_los_largos_de_payload_coinciden():
    fw = _constantes_del_firmware()
    largos_firmware = {n: v for n, v in fw.items() if n.startswith("LEN_")}
    assert largos_firmware, "no se extrajo ningún LEN_ del firmware"

    for nombre, valor in largos_firmware.items():
        assert hasattr(p, nombre), f"{nombre} existe en el firmware pero no en protocol.py"
        assert getattr(p, nombre) == valor, (
            f"{nombre}: firmware={valor} != python={getattr(p, nombre)}"
        )

    # Y al revés, para que no queden LEN_ huérfanos en Python.
    for nombre in dir(p):
        if nombre.startswith("LEN_"):
            assert nombre in largos_firmware, (
                f"{nombre} existe en protocol.py pero no en el firmware"
            )


@pytest.mark.skipif(not FIRMWARE.exists(), reason="no se encontró el firmware")
@pytest.mark.parametrize(
    "nombre_cpp, enum_python",
    [
        ("MotorMode", p.MotorMode),
        ("GripperAction", p.GripperAction),
        ("TeamColor", p.TeamColor),
        ("ColorLabel", p.ColorLabel),
    ],
)
def test_los_enums_compartidos_coinciden(nombre_cpp, enum_python):
    """Los bytes que viajan por el cable significan lo mismo en los dos lados.

    Cubre, por ejemplo, el caso real del gripper de un solo servo: si alguien
    vuelve a agregar RAISE/LOWER en un lado y no en el otro, la Raspberry Pi
    mandaría un número que el ESP32 rechaza en silencio.
    """
    fw = _enum_del_firmware(FIRMWARE, nombre_cpp)
    py = {miembro.name: int(miembro) for miembro in enum_python}
    assert fw == py, f"{nombre_cpp}: firmware={fw} != python={py}"


@pytest.mark.skipif(
    not FIRMWARE_STANDALONE.exists(), reason="no se encontró el firmware autónomo"
)
@pytest.mark.parametrize("nombre_cpp", ["GripperAction", "ColorLabel", "TeamColor"])
def test_el_firmware_autonomo_usa_los_mismos_enums(nombre_cpp):
    """El firmware sin Raspberry Pi comparte hardware, y por tanto semántica.

    No habla el protocolo serial (no hay con quién), pero maneja los mismos
    servos y sensores. Si sus enums se separan de los del firmware de vuelo,
    los ángulos del gripper y los colores de zona dejan de significar lo mismo
    entre las dos variantes, que es justo lo que vuelve imposible depurar una
    comparándola con la otra.
    """
    assert _enum_del_firmware(FIRMWARE_STANDALONE, nombre_cpp) == _enum_del_firmware(
        FIRMWARE, nombre_cpp
    )


@pytest.mark.skipif(not FIRMWARE.exists(), reason="no se encontró el firmware")
def test_los_largos_declarados_son_los_que_se_generan():
    """Los LEN_* del contrato deben coincidir con lo que produce el encoder."""
    assert len(p.encode_motor(p.MotorMode.DRIVE, 10, -10)) == 4 + p.LEN_CMD_MOTOR
    assert len(p.encode_gripper(p.GripperAction.OPEN)) == 4 + p.LEN_CMD_GRIPPER
    assert len(p.encode_led(p.TeamColor.RED)) == 4 + p.LEN_CMD_LED
    assert len(p.encode_flag_signal(True)) == 4 + p.LEN_CMD_FLAG_SIGNAL


def test_la_senal_de_bandera_va_byte_a_byte():
    """El reto de demostración 'señalizar la detección' pasa por esta trama."""
    trama = p.encode_flag_signal(True)
    assert trama[0] == 0xAA
    assert trama[1] == int(p.PacketType.CMD_FLAG_SIGNAL) == 0x04
    assert trama[2] == p.LEN_CMD_FLAG_SIGNAL
    assert trama[3] == 1
    assert trama[4] == p.checksum(0x04, trama[3:4])

    assert p.encode_flag_signal(False)[3] == 0


def test_la_senal_de_bandera_no_es_telemetria():
    """El ESP32 la recibe; la Raspberry Pi nunca debe aceptarla de vuelta."""
    assert p.PacketType.CMD_FLAG_SIGNAL not in p.RECEIVABLE_TYPES


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
