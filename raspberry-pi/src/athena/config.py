"""Configuración del sistema de visión.

Se usan dataclasses con valores por defecto sensatos y, opcionalmente, un
archivo JSON que los sobrescribe. JSON en vez de YAML a propósito: viene en la
librería estándar, y una dependencia menos en una Raspberry Pi es una cosa
menos que pueda fallar la mañana de la competencia.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path


@dataclass(frozen=True)
class CameraConfig:
    device: int = 0
    # Se captura a 640x480 y se procesa a 320x240. Procesar a media resolución
    # baja el costo a la cuarta parte y no perdemos nada útil: una bandera de
    # 5 cm a 1.5 m sigue midiendo ~20 px de ancho, de sobra para detectarla.
    capture_width: int = 640
    capture_height: int = 480
    process_width: int = 320
    process_height: int = 240
    fps: int = 30
    # MJPG en lugar de YUYV: una webcam USB 2.0 no tiene ancho de banda para
    # 640x480@30 sin comprimir, y con YUYV el driver baja solo a 10 fps.
    fourcc: str = "MJPG"


@dataclass(frozen=True)
class ColorRange:
    """Rango HSV. OpenCV usa H de 0..179, S y V de 0..255."""

    h_min: int
    h_max: int
    s_min: int = 90
    v_min: int = 60
    s_max: int = 255
    v_max: int = 255


@dataclass(frozen=True)
class ProposalConfig:
    """Parámetros de la etapa barata que propone regiones candidatas."""

    # El rojo cruza el 0 del círculo de tono, así que necesita dos rangos.
    rojo_bajo: ColorRange = field(default_factory=lambda: ColorRange(0, 10))
    rojo_alto: ColorRange = field(default_factory=lambda: ColorRange(170, 179))
    azul: ColorRange = field(default_factory=lambda: ColorRange(100, 130))

    min_area_px: int = 80        # descarta motas de ruido
    max_area_px: int = 40000     # descarta "todo el encuadre es rojo"
    # Una bandera es un cilindro parado: alta y estrecha. La llave es un cubo.
    # Estos rangos son laxos a propósito: aquí solo proponemos, la CNN decide.
    min_aspect: float = 0.2      # ancho/alto
    max_aspect: float = 3.0
    min_fill_ratio: float = 0.35  # área del blob / área de su caja
    max_proposals: int = 12      # tope duro: protege el presupuesto de CPU
    morph_kernel: int = 3


@dataclass(frozen=True)
class ClassifierConfig:
    model_path: str = "models/athena_cls.tflite"
    input_size: int = 64         # 64x64 basta y sobra para 4 clases tan distintas
    # 3 hilos, no 4: hay que dejarle un núcleo a la captura de cámara y al
    # bucle de control. Pedir los 4 hace que todo compita y sale más lento.
    num_threads: int = 3
    min_confidence: float = 0.60


@dataclass(frozen=True)
class TrackerConfig:
    iou_match_threshold: float = 0.3
    max_missed_frames: int = 5
    # Reclasificar cada N frames en vez de cada frame. Entre medias, el track
    # conserva su clase. Es lo que hace que la CNN casi no aparezca en el perfil.
    reclassify_every: int = 8


@dataclass(frozen=True)
class GeometryConfig:
    """Calibración de la cámara, para estimar distancia y ángulo.

    TODO: medir de verdad con scripts/benchmark.py --calibrar. Los valores por
    defecto son de una webcam típica de 640x480 con ~60 grados de campo de
    visión horizontal; sirven para arrancar, no para competir.
    """

    focal_px: float = 280.0          # distancia focal en píxeles, a 320x240
    bandera_altura_mm: float = 150.0  # dato del reglamento
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
    proposals: ProposalConfig = field(default_factory=ProposalConfig)
    classifier: ClassifierConfig = field(default_factory=ClassifierConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    serial_port: str = "/dev/ttyACM0"   # USB nativo del ESP32-S3
    serial_baud: int = 115200

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
        for section in ("camera", "classifier", "tracker", "geometry", "control"):
            if section in data:
                current = getattr(cfg, section)
                cfg = replace(cfg, **{section: replace(current, **data[section])})
        for key in ("serial_port", "serial_baud"):
            if key in data:
                cfg = replace(cfg, **{key: data[key]})
        return cfg

    def dump(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")
