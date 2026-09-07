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
uname -m   # tiene que decir aarch64: el .eim no corre en un sistema de 32 bits
whoami     # anota tu usuario, lo vas a necesitar en el paso siguiente
groups     # tiene que aparecer 'dialout', si no: sudo usermod -aG dialout $USER
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null   # solo para saber qué hay conectado
```

> El servicio corre como tu usuario (`User=` en el .service), así que necesita
> el grupo **`dialout`** para abrir el puerto del ESP32. Si lo acabás de
> agregar, cerrá sesión y volvé a entrar antes de instalar el servicio: un
> `usermod` no afecta a las sesiones ya abiertas.

> El puerto **no hace falta configurarlo**: `config.py` trae `"auto"`, que
> prueba ttyACM0/1 y ttyUSB0/1 y se queda con el primero que abra. Solo poné
> uno explícito en `config/rover.json` si querés forzarlo.

Edita `deploy/athena-rover.service` y reemplaza **las 3 apariciones** de
`TU_USUARIO_AQUI` por tu usuario real (el de `whoami`).

```bash
cp deploy/equipo.env.example deploy/equipo.env
# Normalmente NO hace falta tocar este archivo: el robot elige equipo con el
# switch físico de 3 posiciones del chasis (ver hardware/conexiones-esp32-
# s3.md). Solo descomentá EQUIPO=rojo/azul ahí si querés forzarlo sin el
# switch instalado (banco de pruebas).

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

**Obligatorio también:** el apagado seguro (ver más abajo) necesita permiso
`sudo` SIN contraseña para el comando `shutdown` puntual -- el servicio
corre sin terminal interactiva, así que un `sudo` que pida contraseña se
quedaría colgado para siempre esperándola. Se agrega en un archivo aparte
bajo `/etc/sudoers.d/`, no editando `/etc/sudoers` directamente (mismo
efecto, menos riesgo de dejar el archivo principal mal formado):

```bash
echo "$(whoami) ALL=(ALL) NOPASSWD: /usr/sbin/shutdown, /sbin/shutdown" | sudo tee /etc/sudoers.d/athena-rover-shutdown
sudo visudo -c   # valida la sintaxis antes de confiar en el archivo
```

Esto le da permiso a tu usuario de correr *solamente* `shutdown` sin
contraseña -- nada más se amplía.

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

## 2b. Apagado seguro al terminar una ronda

Cuando el switch físico de equipo vuelve al **centro** (posición 0) DESPUÉS
de haber estado en AZUL o ROJO, `run_rover.py` interpreta eso como "ya
terminé" y pide un `sudo shutdown -h now` -- no se limita a terminar el
proceso. Es a propósito: cortar la energía de la Raspberry Pi con el sistema
de archivos todavía montado es la forma clásica de corromper la tarjeta SD,
y como el objetivo de este arranque automático es no depender de monitor ni
SSH, hacía falta una forma de apagar la Pi que tampoco dependiera de eso.

**Procedimiento al terminar una ronda:**

1. Mové el switch físico de 3 posiciones de vuelta al centro (posición 0).
2. Esperá a que la Raspberry Pi termine de apagarse sola -- unos 15-20
   segundos es normal. Si tiene un LED de actividad de la propia tarjeta SD,
   dejar de parpadear es la señal más confiable; si no, esperá ese margen de
   tiempo igual.
3. Recién ahí desenergizá el robot (el switch general de la batería).

Si el log (`journalctl -u athena-rover.service -f`) muestra el aviso de
"Apagando la Raspberry Pi de forma segura" pero el sistema NO se apaga, casi
seguro es que falta el permiso `sudo` sin contraseña de más arriba -- el
propio log lo dice explícitamente en vez de fallar en silencio.

## 3. Cambiar de equipo antes de una ronda

**Normal: mové el switch físico de 3 posiciones del chasis** a AZUL o ROJO.
Se lee una sola vez, al puro inicio de la secuencia (`run_rover.py` espera
en la posición central antes de arrancar la ronda) — así que hacé esto ANTES
de que arranque el servicio, no a mitad de una ronda ya en curso. No hace
falta editar nada ni reiniciar el servicio a mano — ver
`hardware/conexiones-esp32-s3.md`.

Solo si estás en banco sin el switch instalado:

```bash
nano deploy/equipo.env          # descomentá EQUIPO=rojo  o  EQUIPO=azul
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
