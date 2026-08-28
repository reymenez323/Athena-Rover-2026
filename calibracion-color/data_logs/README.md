# data_logs/

Acá caen los `.csv` que genera `../calibrar_color.py`, uno por corrida
(`COLOR_<SUPERFICIE>_<SENSOR>_<fecha>_<hora>.csv`).

Se versionan en git a propósito: son datos irremplazables, capturados con
**estos** sensores, a **esta** distancia/ángulo y con **esta** luz — no se
regeneran con un comando. Si vuelves a calibrar, el archivo nuevo
simplemente se suma; no hace falta borrar los anteriores, sirven para
comparar corridas.

Por ahora solo se calibra el sensor DELANTERO (ver ../README.md), así que
deberían caer aquí 5 archivos — uno por color del reto (AZUL, ROJO,
AMARILLO, NEGRO, GRIS). Cuando le toque el turno al TRASERO, se suman otros
5 sin tocar los del delantero.
