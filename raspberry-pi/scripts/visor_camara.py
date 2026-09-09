#!/usr/bin/env python3
"""Visor web de la cámara: para calibrar y armar el dataset, por SSH.

Inspirado en un visor que otro equipo hizo para su XIAO ESP32-S3 Sense (AP
WiFi propio + página con la imagen en vivo y un botón para guardar capturas).
Nuestro robot no tiene esa cámara ni ESP-NOW: la cámara está en la Raspberry
Pi, así que este es el equivalente para NUESTRO hardware -- un servidor HTTP
mínimo (librería estándar, sin dependencias nuevas) que corre en la Pi y se
mira desde el navegador de otra máquina en la misma red, sin necesitar
monitor conectado a la Pi.

QUÉ MUESTRA, por cuadro:
  - El video con las cajas de detección de los DOS detectores superpuestas
    (modelo FOMO en rojo/azul, respaldo color+forma en amarillo -- mismo
    esquema de colores que ``run_rover.py --ver``), más la línea central y
    la zona muerta de centrado.
  - El giro que se SIMULARÍA con esa detección (izquierda/derecha en %,
    igual que ``athena.centering.calcular_giro``) -- se calcula pero NUNCA
    se manda a ningún motor: este script no abre el enlace serial ni
    necesita el ESP32 conectado en absoluto, es cámara sola.
  - Un botón por clase para guardar el cuadro CRUDO (sin las cajas
    dibujadas encima) en ``data/raw/<clase>/``, para ampliar el dataset de
    entrenamiento con los casos que salgan mal en banco.

CÓMO VERLO CON SOLO SSH: si tu laptop y la Pi están en la misma red, abrí
``http://<ip-de-la-pi>:8080`` directo en el navegador -- no hace falta nada
más. Si NO están en la misma red (perfil de red distinto, hotspot, etc.),
reenviá el puerto por el propio túnel de SSH::

    ssh -L 8080:localhost:8080 tu_usuario@ip-de-la-pi

y abrí ``http://localhost:8080`` en tu laptop mientras esa sesión SSH siga
abierta.

USO::

    python3 scripts/visor_camara.py --equipo rojo
    python3 scripts/visor_camara.py --equipo azul --puerto 8080
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from athena.centering import calcular_giro, error_horizontal  # noqa: E402
from athena.color_shape_detector import (  # noqa: E402
    ETIQUETA_AZUL,
    ETIQUETA_ROJO,
    ColorShapeDetector,
)
from athena.config import Config  # noqa: E402
from athena.ei_flag_detector import EiFlagDetector  # noqa: E402

log = logging.getLogger("visor_camara")
_parar = False

# Mismas 4 carpetas que ya existen en data/raw/ -- no se inventa taxonomía
# nueva, el botón de guardar solo puede escribir en una de estas.
CLASES_DATASET = ("bandera_roja", "bandera_azul", "llave", "fondo")

COLOR_ROJO_BGR = (0, 0, 255)
COLOR_AZUL_BGR = (255, 128, 0)
COLOR_AMARILLO_BGR = (0, 255, 255)
COLOR_VERDE_BGR = (0, 220, 0)
COLOR_BLANCO_BGR = (255, 255, 255)


def _signal_handler(signum, frame) -> None:
    global _parar
    _parar = True


# ---------------------------------------------------------------------------
# Estado compartido entre el bucle de cámara y los hilos del servidor HTTP
# ---------------------------------------------------------------------------

class EstadoCompartido:
    """Lo último que vio el bucle principal, protegido por un lock.

    El servidor HTTP corre en su propio hilo (varios, con
    ThreadingHTTPServer) y atiende pedidos en cualquier momento; el bucle de
    cámara escribe acá cada cuadro. Un lock simple alcanza: son lecturas y
    escrituras cortas, no vale la pena algo más elaborado para un visor de
    banco.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame_crudo: np.ndarray | None = None
        self._jpeg_anotado: bytes | None = None
        self._status: dict = {}

    def actualizar(self, frame_crudo: np.ndarray, jpeg_anotado: bytes, status: dict) -> None:
        with self._lock:
            self._frame_crudo = frame_crudo
            self._jpeg_anotado = jpeg_anotado
            self._status = status

    def jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg_anotado

    def status(self) -> dict:
        with self._lock:
            return dict(self._status)

    def frame_crudo(self) -> np.ndarray | None:
        with self._lock:
            return None if self._frame_crudo is None else self._frame_crudo.copy()


# ---------------------------------------------------------------------------
# Página web
# ---------------------------------------------------------------------------

PAGINA_HTML = """<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Athena Rover - Visor de camara</title>
<style>
  :root {{ color-scheme: dark; font-family: Arial, sans-serif; }}
  body {{ margin: 0; background: #101317; color: #f3f5f7; }}
  main {{ width: min(94vw, 820px); margin: 20px auto; }}
  h1 {{ margin: 0 0 4px; font-size: 1.3rem; }}
  .sub {{ color: #aeb8c2; margin-bottom: 14px; font-size: .85rem; }}
  .visor {{ width: 100%; border-radius: 12px; overflow: hidden;
            background: #000; border: 1px solid #343b44; }}
  .visor img {{ width: 100%; display: block; }}
  .botones {{ display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; }}
  button {{ appearance: none; border: 0; border-radius: 9px; padding: 10px 14px;
            font-size: .85rem; font-weight: 700; cursor: pointer; color: #fff; }}
  .b-rojo {{ background: #b23a3a; }} .b-rojo:hover {{ background: #952f2f; }}
  .b-azul {{ background: #2f6db2; }} .b-azul:hover {{ background: #285d97; }}
  .b-llave {{ background: #8a7a2a; }} .b-llave:hover {{ background: #726423; }}
  .b-fondo {{ background: #4a4f57; }} .b-fondo:hover {{ background: #3c4149; }}
  button:disabled {{ opacity: .6; cursor: wait; }}
  .estado-guardado {{ color: #aeb8c2; font-size: .8rem; margin-top: 6px; min-height: 1.1em; }}
  .datos {{ display: grid; grid-template-columns: repeat(2,minmax(0,1fr));
            gap: 8px; margin-top: 14px; }}
  .dato {{ background: #1a2027; border: 1px solid #303843; border-radius: 9px; padding: 10px; }}
  .etiqueta {{ color: #9eabb7; font-size: .74rem; }}
  .valor {{ font-size: .98rem; font-weight: 700; margin-top: 3px; }}
  .pie {{ color: #8f9aa5; font-size: .76rem; margin-top: 14px; }}
</style>
</head>
<body>
<main>
  <h1>Athena Rover -- Visor de camara</h1>
  <div class="sub">Equipo: {equipo} | buscando: {etiqueta_objetivo} | sin motores, sin ESP32 -- solo camara.
    Rojo/azul = modelo FOMO. Amarillo = respaldo color+forma. Linea central + zona muerta en verde/blanco.</div>
  <div class="visor"><img id="camara" alt="Vista de la camara"></div>
  <div class="botones">
    <span style="align-self:center;font-size:.82rem;color:#aeb8c2;">Guardar cuadro crudo como:</span>
    <button class="b-rojo" data-clase="bandera_roja">bandera_roja</button>
    <button class="b-azul" data-clase="bandera_azul">bandera_azul</button>
    <button class="b-llave" data-clase="llave">llave</button>
    <button class="b-fondo" data-clase="fondo">fondo</button>
  </div>
  <div class="estado-guardado" id="estadoGuardado"></div>
  <section class="datos">
    <div class="dato"><div class="etiqueta">Modelo (FOMO)</div><div class="valor" id="claseModelo">--</div></div>
    <div class="dato"><div class="etiqueta">Color+forma (respaldo)</div><div class="valor" id="claseColor">--</div></div>
    <div class="dato"><div class="etiqueta">Error horizontal</div><div class="valor" id="error">--</div></div>
    <div class="dato"><div class="etiqueta">Giro simulado (no se envia)</div><div class="valor" id="giro">--</div></div>
    <div class="dato"><div class="etiqueta">Latencia del modelo</div><div class="valor" id="latencia">--</div></div>
    <div class="dato"><div class="etiqueta">Cuadros procesados</div><div class="valor" id="frames">--</div></div>
  </section>
  <div class="pie" id="diagnostico">Conectando...</div>
</main>
<script>
  const intervaloImagen = {intervalo_ms};
  const imagen = document.getElementById('camara');
  const estadoGuardado = document.getElementById('estadoGuardado');

  function recargarImagen() {{
    const siguiente = () => setTimeout(recargarImagen, intervaloImagen);
    imagen.onload = siguiente;
    imagen.onerror = siguiente;
    imagen.src = '/captura.jpg?t=' + Date.now();
  }}

  async function actualizarEstado() {{
    try {{
      const r = await fetch('/estado.json?t=' + Date.now(), {{cache: 'no-store'}});
      const d = await r.json();
      document.getElementById('claseModelo').textContent = d.clase_modelo;
      document.getElementById('claseColor').textContent = d.clase_color;
      document.getElementById('error').textContent = d.error_x;
      document.getElementById('giro').textContent = d.giro;
      document.getElementById('latencia').textContent = d.latencia_ms + ' ms';
      document.getElementById('frames').textContent = d.frames + ' (' + d.fps + ' FPS)';
      document.getElementById('diagnostico').textContent =
        'Camara ' + d.camara_estado + ' | puerto ' + d.puerto_web;
    }} catch (e) {{
      document.getElementById('diagnostico').textContent = 'Esperando respuesta de la Pi...';
    }}
    setTimeout(actualizarEstado, 500);
  }}

  document.querySelectorAll('button[data-clase]').forEach(boton => {{
    boton.addEventListener('click', async () => {{
      const clase = boton.dataset.clase;
      document.querySelectorAll('button[data-clase]').forEach(b => b.disabled = true);
      estadoGuardado.textContent = 'Guardando...';
      try {{
        const r = await fetch('/guardar?clase=' + clase, {{cache: 'no-store'}});
        const d = await r.json();
        estadoGuardado.textContent = d.ok
          ? ('Guardado: ' + d.archivo)
          : ('Error: ' + d.error);
      }} catch (e) {{
        estadoGuardado.textContent = 'No se pudo guardar (revisa la conexion).';
      }} finally {{
        document.querySelectorAll('button[data-clase]').forEach(b => b.disabled = false);
      }}
    }});
  }});

  recargarImagen();
  actualizarEstado();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# Servidor HTTP
# ---------------------------------------------------------------------------

def _crear_handler(estado: EstadoCompartido, data_dir: Path, args, etiqueta_objetivo: str):
    """Fábrica del handler: BaseHTTPRequestHandler no admite __init__ propio
    con argumentos extra sin pelear con el protocolo del socketserver, así
    que el estado compartido se pasa por clausura en vez de por atributos de
    instancia."""

    class VisorHandler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *fmt_args) -> None:
            # BaseHTTPRequestHandler escribe cada pedido a stderr por
            # defecto -- con la imagen recargándose cada pocos cientos de ms
            # eso inunda la consola. Se enruta al logger propio, en DEBUG.
            log.debug("%s - %s", self.address_string(), format % fmt_args)

        def do_GET(self) -> None:
            ruta = urlparse(self.path)
            if ruta.path == "/":
                self._responder_html()
            elif ruta.path == "/captura.jpg":
                self._responder_jpeg()
            elif ruta.path == "/estado.json":
                self._responder_json()
            elif ruta.path == "/guardar":
                self._guardar(parse_qs(ruta.query))
            elif ruta.path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(404, "Ruta no encontrada")

        def _responder_html(self) -> None:
            cuerpo = PAGINA_HTML.format(
                equipo=args.equipo.upper(),
                etiqueta_objetivo=etiqueta_objetivo,
                intervalo_ms=args.intervalo_imagen_ms,
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(cuerpo)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(cuerpo)

        def _responder_jpeg(self) -> None:
            jpeg = estado.jpeg()
            if jpeg is None:
                self.send_error(503, "Todavia no hay ningun cuadro capturado")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.end_headers()
            self.wfile.write(jpeg)

        def _responder_json(self) -> None:
            cuerpo = json.dumps(estado.status()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(cuerpo)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(cuerpo)

        def _guardar(self, query: dict) -> None:
            clase = query.get("clase", [""])[0]
            respuesta: dict = {}

            # Whitelist estricta: `clase` viene de la URL, y esto termina en
            # una escritura a disco -- no se arma la ruta con lo que venga.
            if clase not in CLASES_DATASET:
                respuesta = {"ok": False, "error": f"clase invalida: {clase!r}"}
            else:
                frame = estado.frame_crudo()
                if frame is None:
                    respuesta = {"ok": False, "error": "todavia no hay ningun cuadro capturado"}
                else:
                    carpeta = data_dir / clase
                    carpeta.mkdir(parents=True, exist_ok=True)
                    nombre = f"captura_{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time() * 1000) % 1000:03d}.jpg"
                    ruta = carpeta / nombre
                    cv2.imwrite(str(ruta), frame)
                    log.info("Captura guardada: %s", ruta)
                    respuesta = {"ok": True, "archivo": f"data/raw/{clase}/{nombre}"}

            cuerpo = json.dumps(respuesta).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(cuerpo)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(cuerpo)

    return VisorHandler


# ---------------------------------------------------------------------------
# Dibujo del overlay
# ---------------------------------------------------------------------------

def _dibujar_overlay(
    frame: np.ndarray,
    detecciones_ei,
    frame_width_ei: int,
    frame_height_ei: int,
    detecciones_color,
    zona_muerta: float,
    giro_texto: str,
    fps: float,
) -> np.ndarray:
    vista = frame.copy()
    fh, fw = vista.shape[:2]

    # Cajas del modelo: reescaladas desde el frame reducido de Edge Impulse
    # (120x120) al frame completo -- mismo cálculo que run_rover.py --ver.
    escala_x = fw / max(1, frame_width_ei)
    escala_y = fh / max(1, frame_height_ei)
    for d in detecciones_ei:
        b = d.box
        x, y = int(b.x * escala_x), int(b.y * escala_y)
        w, h = int(b.w * escala_x), int(b.h * escala_y)
        color = COLOR_ROJO_BGR if d.label == ETIQUETA_ROJO else COLOR_AZUL_BGR
        cv2.rectangle(vista, (x, y), (x + w, y + h), color, 2)
        cv2.putText(vista, f"{d.label} {d.confidence:.2f}", (x, max(12, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)

    # Cajas del respaldo color+forma: ya en el sistema de referencia del
    # frame completo, sin reescalar.
    for d in detecciones_color:
        b = d.box
        cv2.rectangle(vista, (b.x, b.y), (b.x + b.w, b.y + b.h), COLOR_AMARILLO_BGR, 1)
        cv2.putText(vista, f"{d.label} ({d.orientation}) {d.confidence:.2f}",
                    (b.x, min(fh - 4, b.y + b.h + 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, COLOR_AMARILLO_BGR, 1)

    # Linea central + zona muerta, mismo criterio que centering.py.
    centro_x = fw // 2
    zona_px = int(zona_muerta * (fw / 2))
    cv2.line(vista, (centro_x, 0), (centro_x, fh), COLOR_BLANCO_BGR, 1)
    cv2.line(vista, (centro_x - zona_px, 0), (centro_x - zona_px, fh), COLOR_VERDE_BGR, 1)
    cv2.line(vista, (centro_x + zona_px, 0), (centro_x + zona_px, fh), COLOR_VERDE_BGR, 1)

    cv2.putText(vista, giro_texto, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1)
    cv2.putText(vista, f"{fps:.1f} FPS", (8, fh - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    return vista


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--equipo", required=True, choices=["rojo", "azul"],
                        help="tu equipo -- se centra sobre el cilindro CONTRARIO")
    parser.add_argument("--puerto", type=int, default=8080)
    parser.add_argument("--modelo", default="models/athena_ei_banderas.eim")
    parser.add_argument("--config", default=None)
    parser.add_argument("--data-dir", default=None,
                        help="carpeta base del dataset (por defecto: raspberry-pi/data/raw)")
    parser.add_argument("--zona-muerta", type=float, default=0.15)
    parser.add_argument("--kp", type=float, default=60.0)
    parser.add_argument("--correccion-max", type=int, default=40)
    parser.add_argument("--velocidad-base", type=int, default=35)
    parser.add_argument("--min-confianza", type=float, default=0.6)
    parser.add_argument("--intervalo-imagen-ms", type=int, default=250,
                        help="cada cuanto recarga la imagen el navegador (no el ritmo de captura)")
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
    equipo = args.equipo
    etiqueta_objetivo = ETIQUETA_AZUL if equipo == "rojo" else ETIQUETA_ROJO

    modelo_path = Path(args.modelo)
    if not modelo_path.is_absolute():
        modelo_path = REPO / modelo_path

    data_dir = Path(args.data_dir) if args.data_dir else (REPO / "data" / "raw")

    from athena.camera import Camera

    log.info("Equipo: %s | buscando: %s | dataset en: %s", equipo.upper(), etiqueta_objetivo, data_dir)
    log.warning("Solo camara: no se abre el enlace serial, no se manda NADA a motores ni al ESP32.")

    estado = EstadoCompartido()
    color_detector = ColorShapeDetector(geometry=cfg.geometry)

    frames = 0
    frames_sin_objetivo = 0
    sentido_busqueda = 1
    t_inicio = time.monotonic()
    ultimo_fps_calculo = t_inicio
    fps_actual = 0.0

    try:
        with Camera(cfg.camera) as cam, \
             EiFlagDetector(modelo_path, min_confidence=args.min_confianza) as ei_detector:

            handler_cls = _crear_handler(estado, data_dir, args, etiqueta_objetivo)
            servidor = ThreadingHTTPServer(("0.0.0.0", args.puerto), handler_cls)
            hilo_servidor = threading.Thread(target=servidor.serve_forever, daemon=True)
            hilo_servidor.start()
            log.info("Visor arriba en http://0.0.0.0:%d -- abrilo desde otra maquina en la misma red.", args.puerto)

            while not _parar:
                frame = cam.read_full()
                if frame is None:
                    time.sleep(0.01)
                    continue

                detecciones_ei = ei_detector.detect(frame)
                objetivo_ei = EiFlagDetector.best(detecciones_ei, etiqueta_objetivo)
                detecciones_color = color_detector.detect(frame)
                objetivo_color = ColorShapeDetector.best(detecciones_color, etiqueta_objetivo)

                fh, fw = frame.shape[:2]
                if objetivo_ei is not None:
                    caja, ancho_ref = objetivo_ei.box, ei_detector.frame_width
                    fuente = "modelo"
                elif objetivo_color is not None:
                    caja, ancho_ref = objetivo_color.box, fw
                    fuente = "color+forma"
                else:
                    caja, ancho_ref, fuente = None, 0, None

                if caja is not None:
                    frames_sin_objetivo = 0
                    error = error_horizontal(caja, ancho_ref)
                    giro = calcular_giro(
                        error, zona_muerta=args.zona_muerta, velocidad_base=args.velocidad_base,
                        kp=args.kp, correccion_max=args.correccion_max,
                    )
                    giro_texto = f"objetivo por {fuente} -> L={giro.left} R={giro.right} ({'centrado' if giro.centrado else 'corrigiendo'})"
                    error_texto = f"{error:+.2f}"
                    giro_json = f"L={giro.left} R={giro.right}"
                else:
                    # Mismo barrido alternante que decision._buscar_bandera /
                    # run_flag_tracker_ei.py cuando no hay objetivo: se simula
                    # igual, aunque acá nunca se envie a ningun motor.
                    frames_sin_objetivo += 1
                    if frames_sin_objetivo % 90 == 0:
                        sentido_busqueda = -sentido_busqueda
                    v = args.velocidad_base
                    giro_texto = f"sin objetivo -> buscaria girando (L={v * sentido_busqueda} R={-v * sentido_busqueda})"
                    error_texto = "--"
                    giro_json = f"buscando (L={v * sentido_busqueda} R={-v * sentido_busqueda})"

                ahora = time.monotonic()
                if ahora - ultimo_fps_calculo >= 1.0:
                    fps_actual = frames / (ahora - t_inicio) if ahora > t_inicio else 0.0
                    ultimo_fps_calculo = ahora

                vista = _dibujar_overlay(
                    frame, detecciones_ei, ei_detector.frame_width, ei_detector.frame_height,
                    detecciones_color, args.zona_muerta, giro_texto, fps_actual,
                )
                ok, buf = cv2.imencode(".jpg", vista, [cv2.IMWRITE_JPEG_QUALITY, 85])

                status = {
                    "clase_modelo": objetivo_ei.label if objetivo_ei is not None else "--",
                    "clase_color": (
                        f"{objetivo_color.label} ({objetivo_color.orientation})"
                        if objetivo_color is not None else "--"
                    ),
                    "error_x": error_texto,
                    "giro": giro_json,
                    "latencia_ms": round(ei_detector.last_timing_ms, 1),
                    "frames": frames,
                    "fps": round(fps_actual, 1),
                    "camara_estado": "OK",
                    "puerto_web": args.puerto,
                }
                estado.actualizar(frame, buf.tobytes() if ok else b"", status)

                frames += 1

    except Exception:
        log.exception("Fallo en el bucle del visor")
        return 1
    finally:
        try:
            servidor.shutdown()
        except Exception:
            pass
        transcurrido = time.monotonic() - t_inicio
        if frames:
            log.info("%d cuadros en %.1f s (%.1f FPS)", frames, transcurrido, frames / transcurrido)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
