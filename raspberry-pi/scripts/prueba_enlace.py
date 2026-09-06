#!/usr/bin/env python3
"""Prueba de banco del enlace serial con el ESP32-S3, sin cámara ni misión.

Es el equivalente binario de la vieja prueba de texto ``RED / BLUE / OFF``:
manda los comandos uno por uno, con pausas para mirar el LED, y al mismo
tiempo escucha y decodifica toda la telemetría que llega de vuelta. Al final
imprime un resumen con cuántas tramas llegó de cada tipo, a qué ritmo, y
cuántas se descartaron por checksum malo.

Uso::

    python3 scripts/prueba_enlace.py                  # LED + telemetría
    python3 scripts/prueba_enlace.py --equipo azul
    python3 scripts/prueba_enlace.py --solo-escuchar  # no manda nada, solo mira
    python3 scripts/prueba_enlace.py --gripper        # + el servo del gripper
    python3 scripts/prueba_enlace.py --motores        # + los motores (¡ruedas al aire!)

QUÉ HACE FALTA TENER CONECTADO: nada más que el cable USB. El firmware
arranca sus 8 tareas aunque no haya un solo sensor puesto — cada tarea
reintenta su hardware en segundo plano en vez de bloquear — así que esta
prueba se puede correr con el chasis a medio cablear. Los sensores que falten
se van a ver como telemetría inválida, no como silencio.

``--motores`` es la excepción: si el switch físico de equipo (ver
``hardware/conexiones-esp32-s3.md``) está en la posición central, MotorTask
ignora el comando por diseño — es el segundo failsafe del robot, no un
fallo de este script. Mové el switch a AZUL o ROJO antes de usar ``--motores``.

POR QUÉ ES DE UN SOLO HILO: el enlace se drena continuamente en el bucle
principal, sin hilos ni ``input()`` a mitad de la prueba. Si el programa se
quedara esperando una tecla mientras el ESP32 sigue mandando ~1 KB/s, el
búfer del sistema se desbordaría y las tramas perdidas parecerían un problema
del cable cuando en realidad las perdió esta herramienta. Por eso las fases
son por tiempo y las preguntas van todas al final.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from athena.config import Config  # noqa: E402
from athena.link import EspLink  # noqa: E402
from athena.protocol import (  # noqa: E402
    ColorTelemetry,
    GripperAction,
    HealthTelemetry,
    ReflectTelemetry,
    TeamColor,
    TeamSwitchTelemetry,
    Telemetry,
    ToFTelemetry,
)

EQUIPOS = {"rojo": TeamColor.RED, "azul": TeamColor.BLUE}

NOMBRE_TELEMETRIA = {
    ColorTelemetry: "TLM_COLOR   (sensores de color)",
    ReflectTelemetry: "TLM_REFLECT (reflectancia QTR)",
    ToFTelemetry: "TLM_TOF     (distancia VL53L1X)",
    HealthTelemetry: "TLM_HEALTH  (salud de tareas)",
    TeamSwitchTelemetry: "TLM_TEAM_SWITCH (switch fisico de equipo)",
}

# Ritmo al que cada tarea del firmware produce su telemetría (ver
# TaskPeriodMs en firmware-esp32/src/main.cpp). Sirve para avisar si algo
# llega mucho más lento de lo que debería.
HZ_ESPERADO = {
    ColorTelemetry: 10.0,
    ReflectTelemetry: 50.0,
    ToFTelemetry: 20.0,
    HealthTelemetry: 5.0,
    TeamSwitchTelemetry: 4.0,   # LedTask, TaskPeriodMs::LED_STATUS = 250 ms
}


class Estadisticas:
    """Cuenta lo que llega y se queda con la última muestra de cada tipo."""

    def __init__(self) -> None:
        self.conteo: Counter = Counter()
        self.ultima: dict[type, Telemetry] = {}
        self.segundos_escuchando = 0.0

    def registrar(self, paquete: Telemetry) -> None:
        self.conteo[type(paquete)] += 1
        self.ultima[type(paquete)] = paquete

    @property
    def total(self) -> int:
        return sum(self.conteo.values())


def _drenar(link: EspLink, stats: Estadisticas, segundos: float, etiqueta: str) -> None:
    """Escucha el enlace durante N segundos mostrando un contador en vivo.

    Nunca se bloquea: ``poll()`` devuelve lo que haya y sigue. Así el búfer
    del puerto no se acumula mientras quien prueba mira el LED.
    """
    fin = time.monotonic() + segundos
    inicio = time.monotonic()
    proximo_refresco = 0.0
    while time.monotonic() < fin:
        for paquete in link.poll():
            stats.registrar(paquete)
        ahora = time.monotonic()
        # La pantalla se refresca 10 veces por segundo, no cada vuelta: el
        # bucle gira a ~200 Hz y, si se redirige la salida a un archivo, cada
        # refresco quedaría como una línea más. Con esto el log guardado sigue
        # siendo legible.
        if ahora >= proximo_refresco:
            proximo_refresco = ahora + 0.1
            print(
                f"\r  {etiqueta:<38} {max(fin - ahora, 0):4.1f} s  "
                f"| tramas: {stats.total:5d}  descartadas: {link.dropped_frames}",
                end="", flush=True,
            )
        time.sleep(0.005)
    stats.segundos_escuchando += time.monotonic() - inicio
    print()


def _fase(link: EspLink, stats: Estadisticas, etiqueta: str, segundos: float,
          accion=None) -> None:
    """Ejecuta una acción (si la hay) y escucha durante el tiempo indicado."""
    if accion is not None and not accion():
        print(f"\n  ⚠️  Se perdió el enlace al mandar: {etiqueta}")
    _drenar(link, stats, segundos, etiqueta)


def _describir(paquete: Telemetry) -> str:
    if isinstance(paquete, ColorTelemetry):
        return (f"delantero={paquete.front.name}({'ok' if paquete.front_valid else 'sin lectura'}) "
                f"trasero={paquete.back.name}({'ok' if paquete.back_valid else 'sin lectura'})")
    if isinstance(paquete, ReflectTelemetry):
        return (f"izq={paquete.left_raw} der={paquete.right_raw} "
                f"(sobre línea: izq={paquete.left_on_line} der={paquete.right_on_line})")
    if isinstance(paquete, ToFTelemetry):
        return (f"{paquete.distance_mm} mm "
                f"({'medición válida' if paquete.valid else 'sin medición válida'})")
    if isinstance(paquete, HealthTelemetry):
        caidas = paquete.faulted_tasks
        return f"tareas colgadas: {', '.join(caidas) if caidas else 'ninguna'}"
    if isinstance(paquete, TeamSwitchTelemetry):
        return f"{paquete.team.name} ({'posicion central, robot bloqueado' if paquete.team is TeamColor.NONE else 'equipo elegido'})"
    return str(paquete)


def _resumen(link: EspLink, stats: Estadisticas) -> None:
    print("\n" + "=" * 72)
    print("RESUMEN")
    print("=" * 72)
    print(f"Puerto usado:        {link.port}")
    print(f"Tramas recibidas:    {stats.total}")
    print(f"Tramas descartadas:  {link.dropped_frames}   "
          "(checksum malo o basura; si sube, revisá el cable)")
    print(f"Tiempo escuchando:   {stats.segundos_escuchando:.1f} s")

    if stats.total == 0:
        print("\n❌ NO LLEGÓ NADA DEL ESP32. Las causas, en orden de probabilidad:")
        print("   1. El cable está en el puerto USB-C 'UART' del DevKit y no en el 'USB'.")
        print("      Con el firmware de vuelo, el protocolo sale por el USB NATIVO")
        print("      (/dev/ttyACM0). Por el 'UART' solo salen mensajes de depuración.")
        print("   2. El ESP32 no tiene cargado firmware-esp32/ (o tiene otro sketch).")
        print("   3. Falta estar en el grupo 'dialout':  sudo usermod -aG dialout $USER")
        print("      (y volver a iniciar sesión).")
        return

    print("\nPor tipo de telemetría:")
    for tipo, nombre in NOMBRE_TELEMETRIA.items():
        n = stats.conteo.get(tipo, 0)
        hz = n / stats.segundos_escuchando if stats.segundos_escuchando > 0 else 0.0
        esperado = HZ_ESPERADO[tipo]
        marca = "✅" if n > 0 and hz >= esperado * 0.5 else ("⚠️ " if n > 0 else "❌")
        print(f"  {marca} {nombre}: {n:5d} tramas  ({hz:5.1f} Hz, se esperan ~{esperado:.0f} Hz)")
        ultima = stats.ultima.get(tipo)
        if ultima is not None:
            print(f"       última: {_describir(ultima)}")

    salud = stats.ultima.get(HealthTelemetry)
    if isinstance(salud, HealthTelemetry) and salud.faulted_bitmask:
        print("\n⚠️  El supervisor del ESP32 reporta tareas colgadas: "
              f"{', '.join(salud.faulted_tasks)}")
        print("    Ojo: 'colgada' es que dejó de latir, NO que le falte el sensor.")
        print("    Un sensor ausente da telemetría inválida, no una tarea caída.")

    print("""
NOTA sobre sensores no conectados: es NORMAL y esperado ver
  · TLM_COLOR  con 'sin lectura' en los dos sensores,
  · TLM_TOF    con 'sin medición válida',
  · TLM_REFLECT con valores de ruido (los pines del ADC quedan al aire),
si esos sensores todavía no están cableados. Lo que importa en esta prueba es
que las tramas LLEGUEN, con el largo correcto y sin descartes.""")


def _preguntar(pregunta: str) -> bool | None:
    try:
        resp = input(f"  {pregunta} [s/n/?] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return None
    if resp.startswith("s"):
        return True
    if resp.startswith("n"):
        return False
    return None


def _checklist(args: argparse.Namespace) -> None:
    print("\n" + "=" * 72)
    print("LO QUE TENÍAS QUE VER (contestá de memoria, ya terminó la parte cronometrada)")
    print("=" * 72)

    preguntas = [
        f"¿El LED se encendió en {args.equipo.upper()} fijo?",
        "¿Destelló alternando BLANCO y el color de equipo, ~2 veces por segundo?",
        "¿Volvió al color de equipo fijo al terminar el destello?",
    ]
    if args.gripper:
        preguntas.append("¿El servo del gripper se movió (abrir → cerrar → abrir)?")
    if args.motores:
        preguntas.append("¿Los motores giraron adelante y luego frenaron?")

    resultados = [(p, _preguntar(p)) for p in preguntas]

    print("\nResultado de la revisión visual:")
    for pregunta, ok in resultados:
        marca = {True: "✅", False: "❌", None: "❓"}[ok]
        print(f"  {marca} {pregunta}")

    if any(ok is False for _, ok in resultados):
        print("""
Si las tramas llegaron pero el LED no respondió, el enlace está BIEN y el
problema es de cableado o de polaridad. Revisá:
  · R=GPIO39, G=GPIO38, B=GPIO41, común a GND (cátodo común: duty alto enciende).
  · Resistencia en serie por canal (220–330 Ω).
  · Si tu LED es de ÁNODO común, enciende invertido: hay que cambiar
    RgbLed en firmware-esp32/src/main.cpp (ver hardware/conexiones-esp32-s3.md).""")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--puerto", default=None,
                    help="Puerto serial. Por defecto usa el de la config ('auto').")
    ap.add_argument("--equipo", choices=sorted(EQUIPOS), default="rojo",
                    help="Color de equipo que se le manda al LED (por defecto: rojo).")
    ap.add_argument("--config", default=str(REPO / "config" / "rover.json"))
    ap.add_argument("--solo-escuchar", action="store_true",
                    help="No manda ningún comando: solo escucha telemetría.")
    ap.add_argument("--escucha", type=float, default=10.0,
                    help="Segundos de escucha final (por defecto: 10).")
    ap.add_argument("--gripper", action="store_true",
                    help="Incluye el servo del gripper. Requiere el PCA9685 cableado.")
    ap.add_argument("--motores", action="store_true",
                    help="Incluye los motores al 30%%. ¡SOLO con las ruedas al aire!")
    args = ap.parse_args()

    cfg = Config.load(args.config)
    puerto = args.puerto or cfg.serial_port
    equipo = EQUIPOS[args.equipo]

    print("=" * 72)
    print("PRUEBA DE ENLACE SERIAL  ·  Raspberry Pi ↔ ESP32-S3")
    print("=" * 72)
    print(f"Puerto configurado: {puerto}   ({cfg.serial_baud} baudios)")
    print("Mirá el LED del robot mientras corre la prueba.\n")

    if args.motores:
        print("⚠️  MOTORES ACTIVADOS. Con las ruedas apoyadas, el robot se va a mover.")
        try:
            if not input("   Escribí 'si' para continuar: ").strip().lower().startswith("si"):
                print("   Cancelado.")
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\n   Cancelado.")
            return 1

    stats = Estadisticas()

    with EspLink(port=puerto, baud=cfg.serial_baud) as link:
        if not link.connected:
            print(f"❌ No se pudo abrir el puerto ({puerto}).")
            print("   · ¿Aparece en `ls /dev/ttyACM* /dev/ttyUSB*`?")
            print("   · ¿Estás en el grupo 'dialout'? sudo usermod -aG dialout $USER")
            print("   · ¿El cable está en el puerto USB-C 'USB' del DevKit, no en el 'UART'?")
            return 1

        print(f"✅ Conectado en {link.port}\n")

        if args.solo_escuchar:
            _drenar(link, stats, args.escucha, "Solo escuchando (no se manda nada)")
            _resumen(link, stats)
            return 0

        print("Secuencia de comandos:")
        _fase(link, stats, "CMD_LED · apagado (NONE)", 2.0,
              lambda: link.send_led(TeamColor.NONE))
        _fase(link, stats, f"CMD_LED · equipo {args.equipo.upper()}", 3.0,
              lambda: link.send_led(equipo))
        _fase(link, stats, "CMD_FLAG_SIGNAL · bandera A LA VISTA", 5.0,
              lambda: link.send_flag_signal(True))
        _fase(link, stats, "CMD_FLAG_SIGNAL · ya no se ve", 3.0,
              lambda: link.send_flag_signal(False))

        if args.gripper:
            _fase(link, stats, "CMD_GRIPPER · abrir", 2.0,
                  lambda: link.send_gripper(GripperAction.OPEN))
            _fase(link, stats, "CMD_GRIPPER · cerrar sobre la llave", 2.0,
                  lambda: link.send_gripper(GripperAction.CLOSE_LLAVE))
            _fase(link, stats, "CMD_GRIPPER · cerrar sobre la bandera", 2.0,
                  lambda: link.send_gripper(GripperAction.CLOSE_BANDERA))
            _fase(link, stats, "CMD_GRIPPER · abrir", 2.0,
                  lambda: link.send_gripper(GripperAction.OPEN))

        if args.motores:
            switch = stats.ultima.get(TeamSwitchTelemetry)
            if isinstance(switch, TeamSwitchTelemetry) and switch.team is TeamColor.NONE:
                print("  ⚠️  El switch fisico de equipo esta en la posicion central: "
                      "MotorTask va a ignorar este comando y el robot NO se va a mover. "
                      "Mové el switch a AZUL o ROJO antes de repetir esta prueba.")
            # Se reenvía cada vuelta: MotorTask frena solo si pasa 500 ms sin
            # un comando válido (COMMS_FAILSAFE_TIMEOUT_MS), así que un único
            # envío daría un pulso de medio segundo y nada más.
            fin = time.monotonic() + 3.0
            print("  CMD_MOTOR · adelante al 30%")
            while time.monotonic() < fin:
                link.send_motor(30, 30)
                for paquete in link.poll():
                    stats.registrar(paquete)
                time.sleep(0.05)
            stats.segundos_escuchando += 3.0
            _fase(link, stats, "CMD_MOTOR · parada", 2.0, link.send_stop)

        _fase(link, stats, "Escuchando telemetría", args.escucha, None)

        # El LED queda en el color de equipo: es lo que exige el reglamento y
        # deja el robot en un estado reconocible al terminar la prueba.
        link.send_led(equipo)

        _resumen(link, stats)

    _checklist(args)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nInterrumpido.")
        sys.exit(130)
