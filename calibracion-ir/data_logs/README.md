# data_logs/

Acá caen los `.csv` que genera `../calibrar_ir.py`, uno por corrida
(`IR_<SUPERFICIE>_<fecha>_<hora>.csv`).

Se versionan en git a propósito: son datos irremplazables, capturados con
**este** sensor, a **esta** altura y con **esta** luz — no se regeneran con
un comando. Si vuelves a calibrar, el archivo nuevo simplemente se suma; no
hace falta borrar los anteriores, sirven para comparar corridas.
