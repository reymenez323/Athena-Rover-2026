"""Configuración de la Raspberry Pi.

Se usan dataclasses con valores por defecto sensatos y, opcionalmente, un
archivo JSON que los sobrescribe. JSON en vez de YAML a propósito: viene en la
librería estándar, y una dependencia menos en una Raspberry Pi es una cosa
menos que pueda fallar la mañana de la competencia.

El archivo real de cada Pi es ``config/rover.json`` (no se versiona: describe
ESTA Pi, no el código). ``config/rover.example.json`` es la plantilla.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path


@dataclass(frozen=True)
class CameraConfig:
    device: int = 0
    capture_width: int = 640
    capture_height: int = 480
    # Ancho de referencia del pipeline. Ya no se procesa a esta resolución
    # (el SDK de Edge Impulse recorta y reescala por su cuenta a 120x120),
    # pero sigue siendo el ancho contra el que está calibrada
    # ``GeometryConfig.focal_px``: ``run_rover.py`` reescala la focal desde
    # aquí al tamaño real que devuelve el modelo. No lo cambies sin recalibrar.
    process_width: int = 320
    process_height: int = 240
    fps: int = 30
    # MJPG en lugar de YUYV: una webcam USB 2.0 no tiene ancho de banda para
    # 640x480@30 sin comprimir, y con YUYV el driver baja solo a 10 fps.
    fourcc: str = "MJPG"


@dataclass(frozen=True)
class GeometryConfig:
    """Calibración de la cámara, para estimar distancia y ángulo a la bandera.

    Sirve para decidir cuándo frenar y acercarse. La distancia FINA, la que
    dispara el cierre de la pinza, no sale de aquí sino del ToF (VL53L1X) del
    ESP32, que mide de verdad en vez de estimar por tamaño aparente. O sea:
    que ``focal_px`` esté sin calibrar degrada el control, no lo rompe.

    LA CÁMARA
    ---------
    Webcam USB 2.0 que se vende con dos nombres —**Argom CAM20** y **Xtrike Me
    XPC-01**— pero es la misma: mismo fabricante, distinto revendedor. Si
    buscás su ficha vas a encontrar las dos, y no son cámaras distintas.

    Entrega 640x480 en MJPG o YUY2, que es exactamente lo que pide
    ``CameraConfig``, así que ahí no hay nada que ajustar.

    ⚠️ Su ficha técnica trae DOS campos mal etiquetados, y los dos son
    trampas para quien venga a calibrar esto:

    * **"Focal Length: 70-90 cm"** NO es la focal: es la distancia de uso
      recomendada (una webcam pensada para una cara a ~80 cm). Poner 70 en
      ``focal_px`` daría distancias absurdas.
    * **"Aperture: 2.8 mm"** casi seguro SÍ es la focal del lente. Una
      apertura se expresa como f/N y no lleva unidades de longitud; 2.8 mm en
      cambio es un valor típico de focal para una webcam.

    Tomando f = 2.8 mm, ``focal_px = f * ancho_px / ancho_sensor_mm``. El
    tamaño del sensor NO está en la ficha, y es lo que decide el resultado:

    ===============  ==================================
    Sensor           focal_px (a 320 px de ancho)
    ===============  ==================================
    1/4"  (3.6 mm)   ~249
    1/5"  (2.88 mm)  ~311
    1/6"  (2.4 mm)   ~373
    ===============  ==================================

    El valor por defecto (280) cae dentro de ese rango, así que no es un
    disparate — pero la horquilla es de ±25%, y la distancia estimada escala
    directo con esto.

    CÓMO MEDIRLA DE VERDAD (5 minutos, y vale más que toda la tabla de
    arriba): poné la bandera a una distancia D conocida y medida con cinta
    métrica (por ejemplo 500 mm), mirá la altura h en píxeles de su caja en
    ``run_rover.py --ver``, y despejá::

        focal_px = h * D / 150.0        # 150 mm = alto real del cilindro

    Repetilo a dos o tres distancias y promediá. Ese número, medido con ESTA
    cámara y a ESTE ancho de frame, reemplaza al de abajo.
    """

    focal_px: float = 280.0           # en píxeles, referida a CameraConfig.process_width
    bandera_altura_mm: float = 150.0  # cilindro de 15 cm, dato del reglamento
    bandera_diametro_mm: float = 50.0
    llave_lado_mm: float = 20.0


@dataclass(frozen=True)
class ControlConfig:
    """Ganancias del control visual. Conservadoras a propósito."""

    velocidad_crucero: int = 45      # % de PWM al avanzar en línea recta
    velocidad_busqueda: int = 35     # % al girar buscando
    velocidad_aproximacion: int = 30  # % al acercarse a un objetivo
    kp_angulo: float = 0.9           # cuánto corrige por grado de error
    correccion_max: int = 40         # tope de la corrección diferencial
    angulo_muerto_deg: float = 3.0   # por debajo de esto, se considera centrado
    distancia_agarre_mm: float = 120.0  # a esta distancia se cierra la pinza


@dataclass(frozen=True)
class Config:
    camera: CameraConfig = field(default_factory=CameraConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    control: ControlConfig = field(default_factory=ControlConfig)

    # "auto" prueba los candidatos de athena.link.PUERTOS_CANDIDATOS en orden
    # (ttyACM0, ttyACM1, ttyUSB0, ttyUSB1) y se queda con el primero que abra.
    # Es el valor recomendado: el ESP32 aparece como ttyACM* por su USB nativo
    # y como ttyUSB* por el puerto de programación, y en la práctica cambió
    # entre sesiones. Poné un puerto explícito solo si querés forzar uno.
    serial_port: str = "auto"
    serial_baud: int = 115200

    #: Secciones que ``load()`` acepta sobrescribir desde el JSON.
    _SECCIONES = ("camera", "geometry", "control")

    @staticmethod
    def load(path: str | Path | None = None) -> "Config":
        """Carga la configuración; si no hay archivo, usa los valores por defecto."""
        cfg = Config()
        if path is None:
            return cfg
        p = Path(path)
        if not p.exists():
            return cfg

        data = json.loads(p.read_text(encoding="utf-8"))
        # Solo se sobrescriben las secciones presentes en el JSON: un archivo
        # parcial es válido y el resto queda con los valores por defecto.
        for section in Config._SECCIONES:
            if section in data:
                current = getattr(cfg, section)
                cfg = replace(cfg, **{section: replace(current, **data[section])})
        for key in ("serial_port", "serial_baud"):
            if key in data:
                cfg = replace(cfg, **{key: data[key]})
        return cfg

    def dump(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")
