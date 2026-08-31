# Dataset para entrenar el clasificador

## Qué va en cada carpeta

```
data/
├── raw/                  <-- AQUÍ van las fotos que tomes. Se versionan en git.
│   ├── bandera_roja/
│   ├── bandera_azul/
│   ├── llave/
│   └── fondo/            <-- negativos: piso, cinta, paredes, el otro robot...
└── crops/                <-- recortes generados automáticamente. NO se versionan.
```

`raw/` es la carpeta importante: son fotos irremplazables, tomadas con **tu**
cámara, en **tu** pista, con **tu** luz. Por eso sí van a git. `crops/` se
regenera con un comando, así que se ignora.

## Cómo capturar

```bash
python3 scripts/capture_dataset.py --clase bandera_roja
```

Se abre la vista de la cámara. Teclas:

| Tecla | Acción |
|-------|--------|
| `ESPACIO` | guarda el frame actual en la clase elegida |
| `a` | activa/desactiva captura automática (1 foto cada 0.5 s) |
| `q` | salir |

## Cuántas fotos hacen falta

Apunta a **150–300 por clase** como mínimo. Más importante que la cantidad es
la variedad: si todas las fotos son del mismo ángulo y con la misma luz, el
modelo aprende ese ángulo y esa luz, no el objeto.

Varía a propósito:

- **Distancia**: desde 15 cm hasta el otro extremo de la pista.
- **Ángulo**: de frente, de lado, en diagonal, con el objeto medio salido del encuadre.
- **Luz**: luz del salón encendida y apagada, con y sin luz natural, con sombras encima.
- **Fondo**: sobre las distintas zonas de la pista, cerca de las cintas de color.

La clase `fondo` es la más importante y la que todo el mundo descuida. Debe
contener justo lo que el robot va a confundir con una bandera: la cinta roja
del piso, la cinta azul, reflejos, el LED del robot contrario, ropa de color.
Sin buenos negativos, el modelo dirá "bandera" cada vez que vea algo rojo.

## Después de capturar

```bash
python3 scripts/train_classifier.py
```

Genera los recortes en `crops/`, entrena y exporta `models/athena_cls.tflite`
ya cuantizado a INT8 para la Raspberry Pi. Paso a paso completo (con cómo
copiar el modelo a la Pi y verificar que se está usando) en el
["Cómo entrenar el clasificador" del README principal](../README.md#cómo-entrenar-el-clasificador-paso-a-paso).

> `scripts/auto_label_ei.py` NO es parte de este flujo: es para pre-etiquetar
> fotos de otro equipo y subirlas a Edge Impulse Studio, un proyecto externo
> aparte. No genera nada que use `train_classifier.py`.
