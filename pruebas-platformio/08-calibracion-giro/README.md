# 08 — Calibración interactiva del giro en su propio eje

Sketch para encontrar, con el chasis y los motores **actuales**, cuántos
milisegundos de giro a fondo equivalen a un grado — el número que necesita
la fase `ESQUIVAR_CAJA` de
[`standalones/v6-mision-completa/`](../../standalones/v6-mision-completa/README.md):
tras soltar la caja, el robot gira sobre su propio eje para no arrastrarla
al seguir de largo hacia la bandera.

## Por qué hace falta esta calibración, y no basta con la de v5

`v5-agarrar-bandera` ya había medido en banco `kMsPorGrado = 3000ms/180°`
(2026-09-07) para su giro post-agarre (hoy deshabilitado en v6 a pedido
explícito, ver el aviso "SIN GIRO" en `v6-mision-completa/src/main.cpp`).
Ese número es de **antes** de dos cambios de hardware:

- El fix de nombres FL/RL de `07-caracterizacion-motores` (cableado físico
  no cambió, pero confirma que la fricción/comportamiento de cada rueda es
  el que es, no el que se pensaba).
- El reemplazo del motor trasero derecho (engranaje roto).

Con la fricción y el torque del chasis ya distintos, `3000ms/180°` es solo
un punto de partida para no arrancar de cero — no algo confiable sin volver
a medirlo.

## Antes de encender

Pon el robot en el piso con espacio libre alrededor para girar sin chocar
con nada. A diferencia de las pruebas de motores rectos, este sketch **sí**
arranca girando en cuanto mandas `girar` o `girarms` — no hay una fase de
"quieto hasta que mandes algo" previa a eso, cada comando de giro se
ejecuta de inmediato y de forma bloqueante (no se puede interrumpir a
mitad de giro con otro comando; espera a que termine).

## Cómo se usa

1. `pio run -t upload -t monitor` (o el botón equivalente de la extensión),
   por el puerto UART (`COM11` en esta PC).
2. En el monitor serial (115200 baudios), escribe un comando y Enter:

| Comando | Efecto |
|---|---|
| `girarms <izq\|der> <ms>` | Gira por un tiempo crudo, SIN convertir a grados — úsalo primero, para medir. |
| `calibrar <ms> <grados>` | Fija `ms_por_grado = ms / grados`, a partir de una corrida de `girarms` ya medida con transportador o marcas en el piso. |
| `girar <izq\|der> <grados>` | Gira esa cantidad de grados usando la calibración actual (`ms_por_grado * grados`) — para confirmar que la calibración da el ángulo esperado. |
| `msgrado <valor>` | Fija `ms_por_grado` directo, si ya lo sabes de una sesión anterior. |
| `duty <n>` | Cambia el duty (0-255, arranca en 255 — a fondo, mismo criterio que v5: al girar la única palanca que queda es tiempo, no fuerza). |
| `alto` | Detiene los 4 motores. |
| `?` | Reimprime el menú de ayuda. |
| *(línea vacía)* | Reimprime `ms_por_grado` y `duty` actuales. |

`"izq"`/`"der"` es el sentido en que gira el CHASIS visto desde arriba, no
un motor en particular. Si el primer `girar der` gira para el otro lado, es
solo cuestión de nomenclatura — usa el comando contrario, el giro en sí ya
funciona.

## Flujo sugerido para calibrar

1. `girarms der 1000` — gira 1 segundo a fondo. Observa/mide cuántos grados
   giró de verdad (una marca en el piso antes y después, o un transportador
   apoyado en el centro del chasis).
2. `calibrar 1000 <lo que mediste>` — por ejemplo, si giró unos 60°:
   `calibrar 1000 60`. El sketch calcula e imprime el nuevo `ms_por_grado`.
3. `girar der 90` — debería girar (aproximadamente) 90° exactos con la
   calibración recién fijada. Ajusta con `calibrar` de nuevo si hace falta
   afinar.
4. Repite para `izq` — no asumas que da el mismo `ms_por_grado` que `der`;
   ya sabemos que las 4 ruedas no son idénticas entre sí (ver el aviso del
   motor trasero izq. en `hardware/conexiones-esp32-s3.md`), así que el
   chasis bien puede girar distinto para cada lado.
5. Una vez conforme, pasa el `ms_por_grado` final (o los dos, si `izq` y
   `der` dieron distinto) a las constantes de `Mission::` en
   `standalones/v6-mision-completa/src/main.cpp` (`kMsPorGradoEsquive` /
   `kGiroEsquiveCajaDeg`), junto con el `duty` que usaste.

`monitor_filters = send_on_enter, log2file` (igual que `07-caracterizacion-
motores`) deja la sesión completa guardada en un `.log` en esta misma
carpeta, ya ignorado por git.
