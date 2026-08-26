---
título: Reglas del reto — Retos del Rover H07 (2025)
---

# Retos del Rover – H07

## Descripción de la competencia
Dos robots autónomos (equipo rojo y equipo azul) se enfrentan en un campo, cada uno debe capturar la bandera del oponente y llevarla a su propia zona segura. Competencia a 3 rondas; gana quien gane 2 de 3. Cada rover debe identificarse con un LED (rojo o azul).

Antes de buscar la bandera enemiga, en cada ronda cada robot debe depositar una llave en la "zona neutra" (primer desafío). Solo después de depositar la llave puede buscar la bandera. Si un robot busca la bandera antes de depositar la llave en la zona neutra, pierde la ronda de inmediato.

## Especificaciones del robot
- Dimensiones máximas: 30 x 20 x 20 cm.
- Debe identificar la bandera del oponente mediante cámara.
- Debe poder cargar y depositar una caja/llave de 3 x 3 x 3 cm en la zona neutra.
- Debe indicar mediante LED el equipo al que pertenece (rojo/azul).

## Especificaciones de la pista (medidas reales — según plano acotado, NO el documento de reglas)
El documento de reglas indica 250x120 cm y zona neutra de 40x40 cm, pero el equipo confirmó que esas medidas son incorrectas y que las medidas reales de la pista son las del plano acotado ("pista_robotica_plano_acotado_A3_escala_1_5"):

- Pista: **170 cm de largo x 83.5 cm de ancho**.
- Zona neutra (CENTRO): cinta amarilla, área de **27 cm x 29 cm**, en el centro de la pista.
- Zona roja: franja de cinta roja de ~2.0 cm de ancho (a confirmar), ubicada a 28 cm del borde superior (incluye marco negro + franja).
- Zona azul: franja de cinta azul de ~2.0 cm de ancho (a confirmar), ubicada a 28 cm del borde inferior (incluye marco negro + franja).
- Borde de la pista: marco de cinta negra.
- Bandera: cilindro de 5 cm de diámetro x 15 cm de alto. Colores rojo/azul según equipo. No debe modificarse ni añadir elementos externos para su detección.
- Llave: cubo de aprox. 20x20x20 mm (según lo descrito por el equipo; el doc de reglas dice caja de 3x3x3 cm, consistente).

## Especificaciones de competencia (por ronda)
- Los jueces colocan las banderas en posiciones arbitrarias dentro de la zona del participante en cada ronda.
- Si un robot saca 2 ruedas fuera de la pista, pierde la ronda de inmediato.
- Tiempo límite por ronda: 10 minutos. Al concluir, gana quien tenga la bandera oponente más cerca de su zona segura (o quien esté más cerca de la bandera si nadie la tiene).
- Si un robot busca la bandera antes de depositar la llave en la zona neutra, pierde la ronda de inmediato.
- El rover que no esté en posición para iniciar pierde la ronda.

## Retos de funcionalidad (demostración)
- Identificar correctamente líneas de color negro, amarillo, azul y rojo.
- Cargar y depositar la llave especificada.
- Trasladar la bandera.
- Detectar la bandera del oponente y señalizar su detección.

## Retos de clasificatoria (requisitos mínimos)
- Depositar la llave en la zona neutra.
- Identificar la bandera del oponente y dirigirse hacia ella.
- Trasladar la bandera y depositarla en su zona segura.

## Arquitectura del robot
- 4 motores (tracción).
- Gripper delantero accionado por servo, con segundo servo para subir/bajar el gripper.
- Sensor de color delantero + sensor de color trasero (detección de líneas de zona en el piso).
- 2 sensores de reflectancia delanteros (izquierda y derecha).
- Raspberry Pi (inteligencia/visión) comunicada por puerto serial con un ESP32-S3 (control de actuadores y lectura de sensores).
- Cámara web USB estándar para detectar bandera y llave.
