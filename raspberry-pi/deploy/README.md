# Arranque automático del rover al encender la Raspberry Pi

Esto instala `run_rover.py` (la misión **completa**: llave → zona neutra →
bandera → retorno) como servicio de `systemd`, para que arranque solo al
encender la Pi, sin necesidad de conectarse por SSH cada vez.

`run_rover.py` YA usa el modelo de Edge Impulse para encontrar la bandera
(integrado directamente, no hace falta correr nada aparte para eso).
`run_flag_tracker_ei.py` queda solo como herramienta de calibración aislada
para ajustar a ojo la zona muerta/ganancia con `--ver` — **no lo uses como
servicio de arranque**: se saltaría el depósito obligatorio de la llave, lo
que pierde la ronda de inmediato según el reglamento.

## 1. Instalar

Desde `raspberry-pi/` en la Pi (por SSH):

```bash
whoami   # anota tu usuario, lo vas a necesitar en el paso siguiente
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null   # solo para saber qué hay conectado
```

> El puerto **no hace falta configurarlo**: `config.py` trae `"auto"`, que
> prueba ttyACM0/1 y ttyUSB0/1 y se queda con el primero que abra. Solo poné
> uno explícito en `config/rover.json` si querés forzarlo.

Edita `deploy/athena-rover.service` y reemplaza **las 3 apariciones** de
`TU_USUARIO_AQUI` por tu usuario real (el de `whoami`).

```bash
cp deploy/equipo.env.example deploy/equipo.env
nano deploy/equipo.env          # confirma o cambia EQUIPO=rojo / EQUIPO=azul

cp config/rover.example.json config/rover.json
nano config/rover.json          # pon el puerto que confirmaste arriba con ls
```

**Obligatorio ahora:** el modelo de Edge Impulse ya viene en el repo
(`models/athena_ei_banderas.eim`), pero al clonar queda **sin permiso de
ejecución** y `run_rover.py` no arranca sin él (falla rápido y claro, a
propósito):

```bash
chmod +x models/athena_ei_banderas.eim
```

```bash
sudo cp deploy/athena-rover.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable athena-rover.service
sudo systemctl start athena-rover.service
```

## 2. Verificar que arrancó bien

```bash
sudo systemctl status athena-rover.service
journalctl -u athena-rover.service -f      # log en vivo, Ctrl+C para salir
```

## 3. Cambiar de equipo antes de una ronda

```bash
nano deploy/equipo.env          # EQUIPO=rojo  o  EQUIPO=azul
sudo systemctl restart athena-rover.service
```

## 4. Detenerlo (para probar otros scripts a mano)

Mientras el servicio esté activo, tiene la cámara y el puerto serial
ocupados — `run_flag_tracker_ei.py` va a fallar al abrirlos hasta que lo
pares:

```bash
sudo systemctl stop athena-rover.service
```

Para que no vuelva a arrancar solo al reiniciar (sin desinstalarlo):

```bash
sudo systemctl disable athena-rover.service
```

Y para volver a activarlo: `sudo systemctl enable --now athena-rover.service`.

## 5. Ver qué pasó después de una ronda

Por defecto el log de systemd no siempre sobrevive un reinicio. Para que sí:

```bash
sudo mkdir -p /var/log/journal
sudo systemctl restart systemd-journald
```

Después de eso, `journalctl -u athena-rover.service --since "10 min ago"`
funciona incluso tras apagar y encender la Pi.
