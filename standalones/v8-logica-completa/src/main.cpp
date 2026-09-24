// ===========================================================================
//  Athena Rover 2026 — Firmware ESP32-S3 AUTÓNOMO (sin Raspberry Pi)
//  Retos del Rover H07 · INTEC · Reymildo & Montse
//
//  VARIANTE v8-logica-completa (EN CONSTRUCCIÓN, por hitos M0..M7): banco de
//  validación de los 12 pasos de docs/logica-athena.md. Arranca como copia
//  EXACTA de v7 (hito M0: solo cambian los nombres de los mensajes); cada
//  hito posterior agrega una parte de la lógica y queda anotado acá abajo.
//  v6 y v7 NO se tocan. Cuando v8 esté depurado, lo que funcione se promueve
//  a firmware-esp32/ y raspberry-pi/ (ojo: allá speed<0 = adelante; acá
//  speed>0 = adelante).
//
//  HITOS: [M0] copia de v7, compila.
//         [M1] pasos 1-6 de la lógica + protección de borde (QTR derecho). Tras el
//              giro 2 el robot SE DETIENE (Phase::FIN_M1); las fases de bandera que
//              siguen en este archivo son de v7 y NO se alcanzan hasta M2-M5.
//         [M2] paso 7: búsqueda de la bandera y centrado proporcional con la cámara.
//              La cámara manda LÍNEAS DE TEXTO: 'B <error -100..100> <área %>' (la ve;
//              error negativo = a la izquierda) o 'N' (no la ve). Se pueden teclear a
//              mano en el monitor del puerto UART para probar sin la Pi. El robot
//              termina en FIN_M2 al llegar a kDistanciaAproximacionMm con la bandera vista.
//              kBancoSoloPaso7=true salta los pasos 1-6 y empieza directo en la búsqueda.
//         [M3] pasos 8-9: ajuste fino con el ToF y cierre del gripper. Cámara Y ToF son
//              OBLIGATORIOS: sin ver la bandera no se mide ni se cierra (si la pierde, vuelve
//              a buscar). El ToF solo cuenta con la bandera centrada. Tras kMaxPasosSeguridad
//              correcciones sin confirmar rango, si la cámara AÚN la ve, cierra igual.
//              El robot termina en FIN_M3 (bandera agarrada, LED verde).
//         [M4] el código de borde (QTR) cubre también las fases de búsqueda y centrado.
//              Sigue DESACTIVADO (kProteccionBordeActiva) hasta recalibrar el umbral.
//         [M5] pasos 10-12: con la bandera agarrada da la vuelta (~180 grados), sale de la
//              zona rival (lo confirma leyendo la franja rival con el sensor delantero), vuelve
//              recto hasta leer la franja de SU equipo, avanza unos ms, para y suelta la bandera
//              despacio (rampa del servo). kBancoPararTrasAgarrar=true detiene el robot tras
//              agarrar (FIN_M3) para probar el M3 sin que siga de largo.
//
//  ---- Texto heredado de v7 (sigue siendo válido hasta que un hito lo cambie) ----
//  VARIANTE v7-mision-completa-camara: copia EXACTA de
//  standalones/v6-mision-completa/ (que sí funcionaba en banco) más UNA
//  sola cosa nueva -- durante el giro de recentrado tras la caja, la
//  cámara de la Raspberry Pi puede cortar el giro en cuanto ve la bandera
//  contraria, en vez de esperar siempre el tiempo fijo completo. Ver
//  "SEÑAL DE CÁMARA" más abajo. TODO lo demás (motores, color, ToF,
//  gripper, tiempos, el resto de las fases) es intencionalmente idéntico a
//  v6 -- pedido explícito: no tocar nada que ya funcionaba.
//
//  VARIANTE v6-mision-completa (base): integra en una sola secuencia lo
//  validado por separado en v3-color-delantero (caja + sensor de color
//  delantero) y v5-agarrar-bandera (ToF + gripper). Sin QTR (el robot
//  todavía no puede doblar sobre las ruedas, así que todo el recorrido es
//  en línea recta) -- pedido explícito para esta prueba.
//
//  SECUENCIA COMPLETA:
//    1. Espera el switch de equipo (ROJO/AZUL) -- define también cuál es
//       el color de la "zona enemiga" más adelante (el opuesto al propio).
//    2. Asegura la CAJA/LLAVE (gripper a kClawClosedLlaveDeg, 128°) --
//       se asume ya puesta bajo el gripper al arrancar, igual que v3.
//    3. Avanza recto hasta ver AMARILLO con el sensor de color delantero
//       (zona de depósito) -- full stop, y suelta la caja.
//    4. RETROCEDE, GIRA para esquivar la caja recién soltada, AVANZA un
//       poco para terminar de salir de la zona amarilla, y GIRA otra vez
//       para recentrarse hacia donde va a estar la bandera (ver "GIRO DE
//       ESQUIVE Y RECENTRADO TRAS LA CAJA" más abajo) -- no seguir de largo
//       por encima de la caja.
//    5. ESPERA MEDIA (no muy corta, no muy larga -- pensada para que una
//       persona ponga la bandera al frente del robot, a la vista del ToF,
//       ya con la caja fuera del camino gracias a los giros del paso
//       anterior).
//    6. Se acerca a la bandera guiado por el ToF -- MISMO algoritmo de
//       v5-agarrar-bandera (avance grueso, full stop, pasos chicos,
//       confirmar rango 3 veces seguidas, recién ahí cerrar el gripper a
//       kClawClosedBanderaDeg). SIN GIRO tras agarrarla -- pedido explícito:
//       sigue recto (distinto de los giros de esquive/recentrado del paso 4).
//    7. Avanza recto hasta ver el color de la ZONA ENEMIGA (opuesto al
//       equipo elegido en el switch) con el sensor delantero -- full
//       stop, espera, y suelta la bandera.
//    8. TERMINADO.
//
//  POR QUÉ EL SENSOR DE COLOR VA EN EL BUS 1 Y NO EN EL BUS 0
//  ----------------------------------------------------------------------
//  Esta variante necesita el ToF Y el sensor de color delantero A LA VEZ
//  (v3 solo tenía color, v5 solo tenía ToF -- nunca convivieron antes).
//  El 2026-09-08 se confirmó en banco que el TCS34725 y el VL53L1X NO
//  pueden compartir el bus 0x29 ni con el orden correcto de
//  init()/setAddress() -- con el TCS34725 realmente encendido, el init()
//  del ToF fallaba por completo (ver calibracion/tof/README.md, la
//  sección "RESUELTO 2026-09-08 (de verdad)"). El fix real fue separar
//  los buses FÍSICAMENTE: el TCS34725 delantero se recableó al bus I2C
//  nº1 (junto al PCA9685, dirección distinta -- 0x29 vs 0x40, sin choque),
//  el TCS34725 trasero se desconectó, y el ToF se quedó SOLO en el bus 0
//  sin necesitar reasignación. Esta variante asume ese cableado -- ver
//  Pins:: más abajo. Si el TCS34725 delantero sigue en el bus 0 (GPIO8/9)
//  en tu robot, hace falta recablearlo al bus 1 (GPIO47/48) antes de usar
//  este firmware.
//
//  ⚠️ VOLTAJE DE PRUEBA: 7.60 V EN LOS MOTORES (subido de 6.60 V el 2026-09-11)
//  ----------------------------------------------------------------------
//  Empezó en 6.60 V, deliberadamente bajo -- todavía no se ha medido cuánto
//  consume la Raspberry Pi del mismo riel, así que se prefería dejar margen
//  en vez de subir a ciegas. Se subió a 7.60 V el mismo día porque a
//  6.60 V los pasos chicos de ajuste del ToF (Phase::PASO_AJUSTE) no tenían
//  fuerza suficiente para moverse -- a 7.60 V sí funcionan. Los ángulos de
//  giro no cambiaron mucho con la subida (quedaron en los mismos 130°/190°,
//  ver abajo), pero eso fue lo que se observó esta vez, no una garantía:
//  TODOS los parámetros de esta variante que dependen de tiempo (giros,
//  paso de ajuste, kVelocidadCrucero) están calibrados a ESE voltaje --
//  más voltaje es más torque al mismo % de PWM, así que el mismo ms ya no
//  mueve/gira necesariamente lo mismo. Si el voltaje real cambia de nuevo,
//  hay que revisar todo lo de tiempo otra vez, no solo la primera vez --
//  con pruebas-platformio/08-calibracion-giro/ para los giros.
//
//  GIRO DE ESQUIVE Y RECENTRADO TRAS LA CAJA (2026-09-09, ajustado
//  2026-09-11)
//  ----------------------------------------------------------------------
//  Tras soltar la caja en la zona amarilla, el robot retrocede
//  (Phase::RETROCEDER_TRAS_CAJA, para que el pivote no arrastre la caja),
//  gira para esquivarla (Phase::ESQUIVAR_CAJA), avanza un poco para
//  terminar de salir de la zona amarilla (Phase::AVANZAR_TRAS_ESQUIVE), y
//  gira de nuevo para reapuntar hacia donde va a estar la bandera
//  (Phase::GIRO_RECENTRAR) -- mismo primitivo de giro que v5-agarrar-
//  bandera (un lado ADELANTE, el otro ATRAS), calibrado con
//  pruebas-platformio/08-calibracion-giro/.
//
//  Los ángulos (Mission::kGiroEsquiveCajaDeg = 130°,
//  Mission::kGiroRecentrarDeg = 190°) son de banco 2026-09-11 con el robot
//  completo sobre la pista real -- MÁS que los 90°/180° "de libro" que se
//  esperarían, porque la pista tiene mucha fricción y el terreno es
//  irregular: el mismo comando de giro da un ángulo real distinto según
//  en qué punto de la pista esté el robot. Por eso son rangos anchos
//  (120-140° y 170-210° medidos) reducidos a un punto medio, no un
//  ángulo exacto verificado -- no vale la pena perseguir precisión en esta
//  superficie sin un sensor de ángulo real (giroscopio), que por ahora se
//  decidió NO agregar. Ajustar estas dos constantes con
//  pruebas-platformio/08-calibracion-giro/ si 130°/190° no dan un buen
//  resultado en la pista real.
//
//  SEÑAL DE CÁMARA (2026-09-12)
//  ----------------------------------------------------------------------
//  Ese giroscopio que se decidió NO agregar arriba, en la práctica, sí se
//  agrega -- pero como cámara, que de todos modos hace falta para el resto
//  del reglamento. Mientras el robot esquiva la caja (retroceso, primer
//  giro, avance), CAM_LINK puede estar recibiendo bytes de la Raspberry
//  Pi, pero MissionTask no los usa para nada todavía -- a propósito, ni de
//  casualidad debe tocar los motores antes de tiempo. Recién en
//  Phase::GIRO_RECENTRAR se consulta CamaraBandera::Confirmada(): en
//  cuanto la Pi avisa que ve la bandera contraria de forma sostenida (no
//  un byte suelto -- ver kFrescoBanderaCamaraMs/kSostenBanderaCamaraMs), el
//  giro se corta ahí mismo en vez de completar los 190° "de libro" a
//  ciegas. Si la Pi nunca avisa (no está conectada, el modelo no la ve,
//  lo que sea), kDuracionGiroRecentrarMs sigue siendo el tope de siempre --
//  la misión funciona igual que v6 con la Pi completamente ausente.
//
//  El protocolo es deliberadamente el más simple posible: un byte 'V' por
//  cada detección, sin framing ni checksum. Es EXCLUSIVO de este
//  standalone -- no tiene relación con docs/protocolo-serial.md (el de
//  firmware-esp32/, con telemetría completa) ni con lo que ya existía en
//  raspberry-pi/src/athena/. El script que hay que correr en la Pi para
//  esto es aparte, ver el mensaje donde se entregó este archivo.
//
//  DE VUELTA HACIA LA PI (2026-09-22): el switch físico de equipo ya decide
//  ROJO/AZUL en setup() (ver más abajo) -- para que avisar_bandera_v7.py no
//  tenga que recibirlo por línea de comandos (y arriesgarse a desincronizar
//  del switch real), este firmware le manda un byte 'R' o 'A' por el mismo
//  CAM_LINK, repetido cada kEquipoBroadcastMs (ver CamaraBandera::
//  EnviarEquipo()) en vez de una sola vez, para que la Pi lo reciba sin
//  importar el orden de arranque de los dos lados.
//
//  SIN GIRO DESPUÉS DE AGARRAR LA BANDERA
//  ----------------------------------------------------------------------
//  v5-agarrar-bandera giraba ~90° tras cerrar el gripper, para demostrar
//  que podía transportar la bandera y no solo sujetarla en el sitio. Acá
//  NO: pedido explícito para esta prueba -- distinto del giro de esquive
//  de arriba, que sí está habilitado (motivo distinto: alejarse de la caja,
//  no demostrar transporte).
//
//  EL ToF NO SIRVE PARA LA CAJA
//  ----------------------------------------------------------------------
//  El ToF está montado a una altura que nunca va a poder sensar la caja
//  (confirmado por el equipo) -- por eso la caja se agarra al arrancar
//  (se asume ya puesta) y se suelta por COLOR (zona amarilla), nunca por
//  distancia. El ToF solo entra en juego para la bandera, más adelante en
//  la secuencia, cuando ya no hay caja de por medio.
//
//  HARDWARE:
//    · 2x L298N            -> 4 motores (cada driver mueve 2)
//    · 1x VL53L1X (I2C0)   -> distancia a la bandera, SOLO en su bus
//    · 1x PCA9685 (I2C1)   -> el servo del gripper
//    · 1x TCS34725 (I2C1)  -> sensor de color DELANTERO (recableado, ver arriba)
//    · LED RGB (1x)        -> color sensado durante la búsqueda de zona,
//                             indicador de fase durante el resto (ver LedTask)
//    · Switch 3 posiciones -> elige equipo Y arma el robot
//
//  NADA de QTR, NADA de cámara, NADA de TCS34725 trasero (desconectado).
//
//  ÍNDICE
//    [1] Configuración: pines, prioridades, stacks, periodos
//    [2] Tipos compartidos entre tareas
//    [3] Colas y mutex del bus I2C nº1
//    [4] Watchdog cooperativo (heartbeats)
//    [5] Driver PCA9685 (servo por I2C)
//    [6] Driver TCS34725 (color, bus I2C nº1)
//    [7] Clasificación de color -> etiqueta
//    [8] Driver VL53L1X (Pololu, bus I2C nº0)
//    [9] Tareas de hardware (motores, gripper, color, ToF, LED)
//    [10] MissionTask — el cerebro: caja, zona amarilla, bandera, zona enemiga
//    [11] SupervisorTask, setup() / loop()
//
//  TODOS los parámetros que se han estado ajustando a mano en banco (giros,
//  tiempos, velocidades, umbrales del ToF) están juntos en [0] PARÁMETROS
//  AJUSTABLES, justo abajo -- para poder tocarlos y reflashear sin tener
//  que buscar entre las tareas/drivers de más abajo.
//
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <freertos/semphr.h>
#include <esp_system.h>

// ===========================================================================
//  [0] PARÁMETROS AJUSTABLES — todo lo que se ha estado calibrando a mano
//  en banco vive acá, junto, para poder tocarlo sin buscar en el resto del
//  archivo. Las fases de Mission::Phase que usa cada constante están
//  documentadas junto a la constante misma.
// ===========================================================================

namespace Mission {

constexpr bool kMotionEnabled = true;

constexpr int kVelocidadCrucero = 60;   // % de PWM al avanzar recto

// -- Arranque / caja --------------------------------------------------------
constexpr uint32_t kStartupDelayMs = 3000;   // tiempo para ubicar el robot y la caja
constexpr uint32_t kGripperSettleCajaMs = 400;         // tiempo mecánico para que el servo llegue
constexpr uint32_t kDelayTrasAsegurarCajaMs = 600;     // margen extra para que el agarre quede firme

// -- Zona amarilla (depósito de la caja) ------------------------------------
constexpr uint32_t kFullStopAntesDeSoltarCajaMs = 400;   // full stop antes de soltar, no se desliza
constexpr uint32_t kGripperSettleAperturaCajaMs = 400;

// -- Giro de esquive tras soltar la caja -------------------------------------
// Ver "GIRO DE ESQUIVE Y RECENTRADO TRAS LA CAJA" al principio del archivo.
//
// ⚠️ TODO LO DE ACÁ SE MIDIÓ A 7.60 V EN LOS MOTORES (subido de 6.60 V el
// 2026-09-11 -- ver el aviso grande al principio del archivo). Si el
// voltaje real cambia de nuevo, TODOS los valores de aquí basados en
// tiempo (estos giros, el avance intermedio, el paso de ajuste del ToF,
// kVelocidadCrucero) quedan en duda -- más voltaje es más torque en el
// mismo % de PWM, así que el mismo ms ya no gira/avanza necesariamente lo
// mismo. Revisar con 08-calibracion-giro/ (y re-verificar el resto) cada
// vez que cambie el voltaje real de los motores, no solo la primera vez.
//
// ⚠️ PARAMETRIZADO EN MS, NO EN GRADOS -- a propósito. Hasta el
// 2026-09-11 estos dos giros se definían como "grados * ms_por_grado", que
// se ve más intuitivo pero es una ilusión: en esta pista (fricción alta,
// terreno irregular) el mismo comando da un ángulo real distinto según en
// qué punto esté el robot, confirmado en banco -- "grados" no describe
// nada que de verdad se esté logrando, solo maquilla un número de tiempo.
// Ajustar el ms directo (más abajo) es más honesto y más rápido para
// tunear por prueba y error. Los valores de referencia en grados de la
// medición del 2026-09-11 quedan en el comentario de cada uno, por si
// ayuda a razonar la magnitud del cambio, pero NO son la fuente de verdad
// -- esa es el ms. `08-calibracion-giro/` sigue usando grados porque ahí
// sí tiene sentido (se mide con transportador contra el chasis solo).
constexpr int kVelocidadGiroEsquive = 100;   // % de PWM -- a fondo, igual que v5 (la única palanca es tiempo)

// Retrocede antes de girar, para que el pivote no arrastre/empuje la caja
// recién soltada (las ruedas barren un arco al pivotear, no giran en el
// sitio exacto donde quedó la caja). Duración subida de 500 a 900 ms el
// 2026-09-11 (500 no bastaba, según lo visto en banco).
//
// ⚠️ DUTY subido de 40% a 80% el mismo día: a 40% el robot no se movía de
// verdad hacia atrás (se confirmó en banco que el código SÍ manda reversa
// a los 4 motores -- MotorApply/MotorTask, no es un bug de software). Es
// arrancar desde parado, sin inercia, justo tras soltar la caja: el peor
// caso para vencer fricción estática, y 40% quedaba por debajo de TODO lo
// demás que ya funciona en este proyecto (60% de crucero, 100% en los
// giros) -- no hay motivo físico para que retroceder necesite menos fuerza
// que avanzar. Seguir ajustando según lo que se vea.
constexpr uint32_t kRetrocesoTrasCajaMs = 900;
constexpr int kVelocidadRetrocesoTrasCaja = 70;   // % de PWM

// Primer giro: esquivar la zona amarilla. Referencia en grados del banco
// 2026-09-11: 120-140° de comando hacían falta para despejar la caja (no
// 90° como se había puesto de entrada) -- 130° ~ 2166 ms era el punto
// medio con el ms_por_grado de entonces. AJUSTAR ACÁ, en ms, directamente.
constexpr uint32_t kDuracionGiroEsquiveMs = 2000;
// true = gira hacia la derecha (visto desde arriba) al esquivar; false =
// hacia la izquierda. Cuál conviene depende de dónde queda la caja/pista
// respecto al robot -- ajustar según la pista real, no es simétrico.
constexpr bool kGiroEsquiveHaciaDerecha = true;

// Tras el giro de esquive, avanza un poco en línea recta para terminar de
// salir de la huella de la zona amarilla antes de girar otra vez -- sin
// esto, el segundo giro (más grande) podía volver a pasar sobre la caja.
// Sin medir en banco todavía, punto de partida conservador.
constexpr uint32_t kAvanceTrasEsquiveMs = 1300;   // subido de 900 (2026-09-24): con 900 el giro 2 arrancaba aún encima de la caja
constexpr int kVelocidadAvanceTrasEsquive = 70;   // % de PWM, moderado

// Segundo giro: volver a centrarse hacia donde va a estar la bandera, tras
// haberse desviado con el giro de esquive. Referencia en grados del banco
// 2026-09-11: 170-210° de comando -- 190° ~ 3167 ms era el punto medio con
// el ms_por_grado de entonces. kDuracionGiroRecentrarMs YA NO es la
// duración garantizada de este giro -- ver "SEÑAL DE CÁMARA" más abajo --
// es el TOPE de seguridad si la cámara nunca avisa.
constexpr uint32_t kDuracionGiroRecentrarMs = 2800;
// Gira para el lado CONTRARIO al de esquive por defecto (deshace parte del
// desvío y sigue de largo hacia el otro lado) -- confirmar con la pista
// real cuál sentido deja al robot mejor apuntado hacia la bandera.
constexpr bool kGiroRecentrarHaciaDerecha = !kGiroEsquiveHaciaDerecha;

// -- Señal de cámara (Raspberry Pi) durante el giro de recentrado ----------
// Ver "SEÑAL DE CÁMARA" al principio del archivo. Un 'V' recibido por
// CAM_LINK cuenta como "vista" por este tiempo (por si la Pi manda a un
// ritmo bajo o irregular, un solo aviso no debería caducar de inmediato);
// hace falta que la señal esté FRESCA de forma continua por
// kSostenBanderaCamaraMs antes de confiar y cortar el giro -- mismo
// espíritu que el "detección sostenida" que ya usa decision.py en la Pi,
// para no cortar el giro por un aviso suelto/ruido.
constexpr uint32_t kFrescoBanderaCamaraMs = 300;
constexpr uint32_t kSostenBanderaCamaraMs = 150;

// -- Paso 7: buscar la bandera y centrarse (hito M2) --------------------------
// Protocolo de la cámara (CAM_LINK o el puerto UART, para simular a mano):
//   'B <error> <área>'  la ve. error -100..100 (negativo = a la izquierda, 0 = centrada);
//                       área = % del cuadro que ocupa la bandera (solo informativo, no decide nada)
//   'N'                 no la ve
constexpr bool kBancoSoloPaso7 = true;   // SOLO BANCO: salta los pasos 1-6 y empieza directo buscando la bandera. Poner false para la corrida completa
// Búsqueda cuando la cámara no la ve: pausa -> pivote A -> pausa -> pivote B (doble) -> pausa -> pivote A -> pausa -> avance corto, y repite.
// Los pivotes alternados casi se anulan entre sí; las pausas dan cuadros estables a la cámara.
constexpr uint32_t kBusquedaPausaMs   = 400;   // quieto para que la cámara mire sin desenfoque
constexpr uint32_t kBusquedaPivoteMs  = 600;   // duración del pivote A (el B dura el doble). ~600 ms ~ 40 grados a 100 %, a 7.60 V
constexpr int      kBusquedaVelocidadPivote = 100;   // % de PWM del pivote (a menos no vence la fricción)
constexpr uint32_t kBusquedaAvanceMs  = 400;   // avance corto al final de cada ciclo, para explorar más adelante
constexpr int      kBusquedaVelocidadAvance = 50;    // % de PWM de ese avance
constexpr bool     kBusquedaPrimerPivoteHaciaDerecha = true;   // lado del primer pivote
// Centrado proporcional: si |error| <= zona muerta avanza recto; si no, pivota en pulsos
// (pulso + pausa para leer de nuevo, porque la cámara llega con retraso) con más velocidad cuanto mayor el error.
constexpr int      kZonaMuertaCentrado = 15;        // error ignorado (equivale a 0.15)
constexpr uint32_t kPulsoCentradoMs    = 120;       // duración de cada pulso de giro
constexpr uint32_t kAsentarCentradoMs  = 250;       // pausa entre pulsos antes de volver a leer el error
constexpr int      kVelocidadCentradoMin = 60;      // % de PWM del pulso con el error apenas fuera de la zona muerta
constexpr int      kVelocidadCentradoMax = 100;     // % de PWM con el error máximo
constexpr int      kVelocidadAcercamiento = 50;     // % de PWM al avanzar recto hacia la bandera ya centrada
constexpr uint32_t kPerdidaBanderaMs   = 500;       // si la cámara deja de verla, espera esto quieto antes de volver a buscar

// -- Aviso de equipo hacia la Raspberry Pi (para avisar_bandera_v7.py) -----
// El switch físico ya decide el equipo acá (ver setup()); en vez de que la
// Pi lo reciba por línea de comandos (que puede desincronizarse del switch
// real), el ESP32 lo manda solo, repetido a este ritmo -- así, sin importar
// si avisar_bandera_v7.py arranca antes, después, o se reinicia a mitad de
// ronda, lo recibe igual en menos de un ciclo.
constexpr uint32_t kEquipoBroadcastMs = 500;

// -- Protección de borde (QTR) -- hito M1 ------------------------------------
// Regla 1 de docs/logica-athena.md: nunca salirse de la pista (el marco es
// cinta negra). El QTR compara el emisor IR encendido contra apagado; una
// diferencia chica (menor que kBordeRestadoUmbral) = superficie negra = borde.
// Solo actúa en las fases que avanzan o giran (no al retroceder ni quieto).
constexpr bool     kProteccionBordeActiva = false;  // interruptor general. DESACTIVADO por ahora: el QTR derecho dio falsos bordes cerca del amarillo (2026-09-24); se recalibra en M4
constexpr bool     kQtrIzquierdoActivo    = false;  // el izquierdo está pegado al tope (sin señal útil, diagnosticado 2026-09-24): NO activar hasta repararlo
constexpr int16_t  kBordeRestadoUmbral    = 40;     // |dif| menor que esto = negro (negro ~1-8, gris ~76-87 medido en banco)
constexpr uint16_t kQtrTopeAdc            = 4085;   // off y on >= esto = sensor pegado al tope: NO cuenta como negro (sería un falso borde)
constexpr uint32_t kBordeDebounceMs       = 50;     // el borde debe verse sostenido este tiempo antes de reaccionar (filtra ruido)
constexpr uint32_t kBordeParadaMs         = 200;    // parada total antes de retroceder
constexpr uint32_t kBordeRetrocesoMs      = 300;    // retroceso corto (no hay sensor trasero: por eso corto y limitado)
constexpr int      kBordeVelocidadRetroceso = 60;   // % de PWM del retroceso
constexpr uint32_t kBordeGiroMs           = 500;    // pivote para alejarse del borde
constexpr int      kBordeVelocidadGiro    = 100;    // % de PWM del pivote
constexpr bool     kBordeGiroHaciaDerecha = false;  // con solo el QTR derecho activo el borde queda a la derecha: se gira a la IZQUIERDA

// -- Retorno con la bandera (hito M5, pasos 10-12) ----------------------------
constexpr bool     kBancoPararTrasAgarrar = true;   // SOLO BANCO: tras agarrar la bandera se detiene (FIN_M3) en vez de volver. Poner false para la corrida completa
constexpr uint32_t kGiroRetornoMs         = 2700;   // pivote para dar la vuelta con la bandera (~180 grados: 2800 ms dio ~190 en pista, ver kDuracionGiroRecentrarMs)
constexpr int      kVelocidadGiroRetorno  = 100;    // % de PWM del pivote
constexpr bool     kGiroRetornoHaciaDerecha = true; // sentido del pivote de vuelta
constexpr int      kVelocidadRetorno      = 60;     // % de PWM al avanzar recto de vuelta
constexpr uint32_t kSalirZonaRivalTopeMs  = 4000;   // si no lee la franja rival en este tiempo, asume que ya salió y sigue (la franja mide ~18.5 mm, puede saltársela)
constexpr uint32_t kVolverTopeMs          = 9000;   // si no lee su franja en este tiempo, PARA y suelta igual (evita seguir hasta salirse de la pista)
constexpr uint32_t kAvanceTrasLeerZonaPropiaMs = 300;   // ms que sigue avanzando tras leer su franja, antes de parar (por tiempo: no hay IMU)
constexpr int      kVelocidadEntradaZonaPropia = 50;    // % de PWM de ese último avance
constexpr uint32_t kFullStopZonaPropiaMs  = 700;    // parada total antes de soltar la bandera
constexpr bool     kRetornoEvitaAmarillo  = true;   // si al volver lee AMARILLO (zona neutra, con la caja encima) la rodea para no arrastrarla. SIN PROBAR en pista
constexpr bool     kEvitarAmarilloHaciaDerecha = true;   // lado hacia el que se aparta del amarillo
constexpr uint32_t kEvitarAmarilloRetrocesoMs = 300;     // retroceso corto antes de apartarse
constexpr uint32_t kEvitarAmarilloGiroMs  = 700;    // pivote para apartarse del amarillo
constexpr int      kSoltarLentoPasoDeg    = 5;      // la bandera se suelta despacio: el servo baja de 65 grados a 0 en pasos de esta cantidad...
constexpr uint32_t kSoltarLentoPasoMs     = 60;     // ...uno cada tanto (65/5 = 13 pasos x 60 ms ~ 0.8 s)
constexpr uint32_t kSoltarLentoEsperaMs   = 1200;   // espera tras pedir la apertura lenta (debe ser mayor que la rampa)

// -- Espera media entre caja y bandera ---------------------------------------
// Pedido explícito: "ni muy corta ni muy larga" -- tiempo para que una
// persona ponga la bandera al frente del robot (ya reorientado por el giro
// de esquive), a la vista del ToF. Ajustar acá si 6 s se queda corto/largo.
constexpr uint32_t kEsperaReacomodoMs = 6000;

// -- Acercamiento a la bandera (ToF) -- MISMOS valores que v5-agarrar-bandera,
// confirmados en banco 2026-09-07 -----------------------------------------
constexpr uint16_t kDistanciaAproximacionMm = 150;
constexpr uint16_t kRangoAgarreMinMm = 56;
constexpr uint16_t kRangoAgarreMaxMm = 62;   // subido de 60 (2026-09-24, pedido de Montse: rango 56-62)
constexpr int kVelocidadPaso = 60;
constexpr uint32_t kPasoDuracionMs = 150;
constexpr uint32_t kSettleTrasParoMs = 200;
constexpr int kLecturasConsecutivasRequeridas = 3;
constexpr int kMaxPasosSeguridad = 40;
constexpr uint32_t kGripperSettleBanderaMs = 500;

// -- Tras agarrar la bandera: SIN GIRO, pedido explícito (distinto de los
// giros de esquive/recentrado tras la caja, que sí están habilitados) --
// solo un respiro corto y sigue recto.
constexpr uint32_t kEsperaTrasAgarrarBanderaMs = 600;

// -- Zona enemiga (depósito de la bandera) -----------------------------------
constexpr uint32_t kFullStopZonaEnemigaMs = 700;   // "espere un poco" antes de soltar
constexpr uint32_t kGripperSettleSueltaBanderaMs = 400;

// -- Fases de la misión -------------------------------------------------------
// (no son "parámetros" en sí, pero viven acá junto con todo lo demás de
// Mission:: para no partir el namespace en dos mitades del archivo)
enum class Phase : uint8_t {
    ARRANQUE = 0,
    ASEGURAR_CAJA,
    ESPERAR_ANTES_DE_AVANZAR,
    BUSCAR_ZONA_AMARILLA,
    DETENER_ZONA_AMARILLA,
    DEPOSITAR_CAJA,
    RETROCEDER_TRAS_CAJA,
    ESQUIVAR_CAJA,
    AVANZAR_TRAS_ESQUIVE,
    GIRO_RECENTRAR,
    ESPERAR_REACOMODO,
    AVANCE_GRUESO_BANDERA,
    DETENER_PARA_MEDIR,
    PASO_AJUSTE,
    CERRAR_GRIPPER_BANDERA,
    ESPERAR_TRAS_AGARRE,
    AVANZAR_ZONA_ENEMIGA,
    DETENER_ZONA_ENEMIGA,
    SOLTAR_BANDERA,
    TERMINADO,
    FALLO_AJUSTE,
    BORDE_PARAR,        // M1: reacción al borde negro
    BORDE_RETROCEDER,
    BORDE_GIRAR,
    FIN_M1,             // M1: fin de los pasos 1-6, el robot se detiene
    BUSCAR_BANDERA,     // M2: paso 7, la cámara no la ve
    CENTRAR_Y_AVANZAR,  // M2: paso 7, la cámara la ve: se centra y avanza
    FIN_M2,             // M2: (ya no se alcanza: ahora pasa al ajuste fino)
    CENTRAR_FINO,       // M3: un pulso de giro para recentrar la bandera antes de medir con el ToF
    FIN_M3,             // M3: bandera agarrada, el robot se detiene
    GIRO_RETORNO,       // M5: paso 10, da la vuelta con la bandera
    SALIR_ZONA_RIVAL,   // M5: paso 10, avanza hasta leer la franja rival (confirma la salida)
    VOLVER_ZONA_PROPIA, // M5: paso 11, avanza hasta leer la franja de su equipo
    EVITAR_AMARILLO,    // M5: rodea la zona neutra si la pisa al volver
    AVANCE_EN_ZONA_PROPIA, // M5: paso 12, unos ms más dentro de su zona
    DETENER_ZONA_PROPIA,   // M5: paso 12, parada total
    SOLTAR_BANDERA_LENTO,  // M5: paso 12, apertura lenta del gripper
};

inline const char *PhaseName(Phase phase) {
    switch (phase) {
        case Phase::ARRANQUE:                  return "ARRANQUE";
        case Phase::ASEGURAR_CAJA:              return "ASEGURAR_CAJA";
        case Phase::ESPERAR_ANTES_DE_AVANZAR:   return "ESPERAR_ANTES_DE_AVANZAR";
        case Phase::BUSCAR_ZONA_AMARILLA:       return "BUSCAR_ZONA_AMARILLA";
        case Phase::DETENER_ZONA_AMARILLA:      return "DETENER_ZONA_AMARILLA";
        case Phase::DEPOSITAR_CAJA:             return "DEPOSITAR_CAJA";
        case Phase::RETROCEDER_TRAS_CAJA:       return "RETROCEDER_TRAS_CAJA";
        case Phase::ESQUIVAR_CAJA:              return "ESQUIVAR_CAJA";
        case Phase::AVANZAR_TRAS_ESQUIVE:       return "AVANZAR_TRAS_ESQUIVE";
        case Phase::GIRO_RECENTRAR:             return "GIRO_RECENTRAR";
        case Phase::ESPERAR_REACOMODO:          return "ESPERAR_REACOMODO";
        case Phase::AVANCE_GRUESO_BANDERA:      return "AVANCE_GRUESO_BANDERA";
        case Phase::DETENER_PARA_MEDIR:         return "DETENER_PARA_MEDIR";
        case Phase::PASO_AJUSTE:                return "PASO_AJUSTE";
        case Phase::CERRAR_GRIPPER_BANDERA:     return "CERRAR_GRIPPER_BANDERA";
        case Phase::ESPERAR_TRAS_AGARRE:        return "ESPERAR_TRAS_AGARRE";
        case Phase::AVANZAR_ZONA_ENEMIGA:       return "AVANZAR_ZONA_ENEMIGA";
        case Phase::DETENER_ZONA_ENEMIGA:       return "DETENER_ZONA_ENEMIGA";
        case Phase::SOLTAR_BANDERA:             return "SOLTAR_BANDERA";
        case Phase::TERMINADO:                  return "TERMINADO";
        case Phase::FALLO_AJUSTE:               return "FALLO_AJUSTE";
        case Phase::BORDE_PARAR:                return "BORDE_PARAR";
        case Phase::BORDE_RETROCEDER:           return "BORDE_RETROCEDER";
        case Phase::BORDE_GIRAR:                return "BORDE_GIRAR";
        case Phase::FIN_M1:                     return "FIN_M1";
        case Phase::BUSCAR_BANDERA:             return "BUSCAR_BANDERA";
        case Phase::CENTRAR_Y_AVANZAR:          return "CENTRAR_Y_AVANZAR";
        case Phase::FIN_M2:                     return "FIN_M2";
        case Phase::CENTRAR_FINO:               return "CENTRAR_FINO";
        case Phase::FIN_M3:                     return "FIN_M3";
        case Phase::GIRO_RETORNO:               return "GIRO_RETORNO";
        case Phase::SALIR_ZONA_RIVAL:           return "SALIR_ZONA_RIVAL";
        case Phase::VOLVER_ZONA_PROPIA:         return "VOLVER_ZONA_PROPIA";
        case Phase::EVITAR_AMARILLO:            return "EVITAR_AMARILLO";
        case Phase::AVANCE_EN_ZONA_PROPIA:      return "AVANCE_EN_ZONA_PROPIA";
        case Phase::DETENER_ZONA_PROPIA:        return "DETENER_ZONA_PROPIA";
        case Phase::SOLTAR_BANDERA_LENTO:       return "SOLTAR_BANDERA_LENTO";
        default:                                return "DESCONOCIDA";
    }
}

} // namespace Mission

// ===========================================================================
//  [1] CONFIGURACIÓN
// ===========================================================================

namespace Pins {
    // -------- Motores: 2x L298N, cada uno mueve 2 motores ------------------
    constexpr uint8_t L298N_L_IN1 = 4;
    constexpr uint8_t L298N_L_IN2 = 5;
    constexpr uint8_t L298N_L_ENA = 6;
    constexpr uint8_t L298N_L_IN3 = 7;
    constexpr uint8_t L298N_L_IN4 = 15;
    constexpr uint8_t L298N_L_ENB = 16;

    constexpr uint8_t L298N_R_IN1 = 10;
    constexpr uint8_t L298N_R_IN2 = 11;
    constexpr uint8_t L298N_R_ENA = 12;
    constexpr uint8_t L298N_R_IN3 = 13;
    constexpr uint8_t L298N_R_IN4 = 14;
    constexpr uint8_t L298N_R_ENB = 17;

    // -------- Bus I2C nº0: VL53L1X (ToF), SOLO -------------------------------
    // Sin TCS34725 acá -- ver el aviso grande al principio del archivo.
    constexpr uint8_t I2C0_SDA = 8;
    constexpr uint8_t I2C0_SCL = 9;
    constexpr uint8_t TOF_XSHUT = 3;

    // -------- Bus I2C nº1: PCA9685 (gripper) + TCS34725 delantero ----------
    // El TCS34725 delantero vive ACÁ, no en el bus 0 -- ver el aviso grande
    // al principio del archivo.
    constexpr uint8_t I2C1_SDA = 47;
    constexpr uint8_t I2C1_SCL = 48;
    constexpr uint8_t TCS_LED_FRONT = 18;   // LED de iluminación del TCS34725, GPIO aparte (no I2C)

    // -------- LED RGB: color sensado / indicador de fase --------------------
    constexpr uint8_t LED_RGB_R = 39;
    constexpr uint8_t LED_RGB_G = 38;
    constexpr uint8_t LED_RGB_B = 41;

    // -------- Switch de 3 posiciones: elige equipo Y arma el robot ---------
    constexpr uint8_t TEAM_SWITCH_BLUE = 40;
    constexpr uint8_t TEAM_SWITCH_RED  = 21;

    // -------- Reflectancia: 2x QTRX-HD-01A (a 3.3 V, NUNCA 5 V) -------------
    constexpr uint8_t QTR_LEFT_OUT     = 1;    // ADC1_CH0
    constexpr uint8_t QTR_RIGHT_OUT    = 2;    // ADC1_CH1
    constexpr uint8_t QTR_EMITTER_CTRL = 42;   // CTRL de los emisores IR, compartido
}

namespace I2CAddr {
    constexpr uint8_t PCA9685  = 0x40;
    constexpr uint8_t TCS34725 = 0x29;   // bus 1
    // VL53L1X: dirección de fábrica, sin reasignar -- el bus 0 es solo
    // suyo, no hace falta (ver el aviso grande al principio del archivo).
    constexpr uint8_t VL53L1X = 0x29;    // bus 0
}

namespace ServoChannel {
    constexpr uint8_t CLAW = 0;
}

namespace Pwm {
    constexpr uint32_t MOTOR_FREQ_HZ    = 1000;
    constexpr uint8_t  MOTOR_RESOLUTION = 8;

    constexpr uint32_t RGB_FREQ_HZ    = 5000;
    constexpr uint8_t  RGB_RESOLUTION = 8;

    constexpr uint32_t SERVO_FREQ_HZ  = 50;
    constexpr uint16_t SERVO_TICK_MIN = 205;
    constexpr uint16_t SERVO_TICK_MAX = 410;
}

// Ángulos ya calibrados -- ver pruebas-platformio/06-calibracion-gripper/.
constexpr int kClawOpenDeg          = 0;
constexpr int kClawClosedLlaveDeg   = 128;
constexpr int kClawClosedBanderaDeg = 65;

namespace TaskPriority {
    constexpr UBaseType_t SUPERVISOR      = 6;
    constexpr UBaseType_t MOTOR_CONTROL   = 5;
    constexpr UBaseType_t MISSION         = 4;
    constexpr UBaseType_t GRIPPER_CONTROL = 3;
    constexpr UBaseType_t COLOR_SENSOR    = 2;
    constexpr UBaseType_t TOF_SENSOR      = 2;
    constexpr UBaseType_t REFLECTANCE     = 2;
    constexpr UBaseType_t LED_STATUS      = 1;
}

namespace TaskStack {
    constexpr uint32_t SUPERVISOR      = 3072;
    constexpr uint32_t MISSION         = 4096;
    constexpr uint32_t MOTOR_CONTROL   = 3072;
    constexpr uint32_t GRIPPER_CONTROL = 3072;
    constexpr uint32_t COLOR_SENSOR    = 3584;
    constexpr uint32_t TOF_SENSOR      = 3072;
    constexpr uint32_t REFLECTANCE     = 3072;
    constexpr uint32_t LED_STATUS      = 2048;
}

namespace TaskPeriodMs {
    constexpr uint32_t SUPERVISOR    = 200;
    constexpr uint32_t MISSION       = 50;
    constexpr uint32_t MOTOR_CONTROL = 20;
    constexpr uint32_t GRIPPER       = 50;
    constexpr uint32_t COLOR_SENSOR  = 100;
    constexpr uint32_t TOF_SENSOR    = 50;   // igual al ranging period del VL53L1X
    constexpr uint32_t REFLECTANCE   = 20;   // 50 Hz
    constexpr uint32_t LED_STATUS    = 100;
}

constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;
constexpr uint32_t MISSION_FAILSAFE_TIMEOUT_MS = 500;

enum class TaskId : uint8_t {
    MOTOR_CONTROL = 0,
    GRIPPER_CONTROL,
    COLOR_SENSOR,
    TOF_SENSOR,
    LED_STATUS,
    REFLECTANCE,
    MISSION,
    COUNT
};

#define DEBUG_LINK Serial0   // consola por el puerto UART del DevKit -- así flashear y
                              // monitorear usan el mismo cable, sin cambiar de puerto

// SEÑAL DE CÁMARA (2026-09-12): puerto USB nativo, dedicado a recibir de la
// Raspberry Pi un aviso de "veo la bandera contraria" durante el giro de
// recentrado -- ver Phase::GIRO_RECENTRAR y CamaraBandera:: más abajo. Este
// standalone sigue siendo "sin Raspberry Pi" para TODO lo demás (motores,
// color, ToF, gripper, tiempos) -- la Pi acá solo manda un byte 'V' cada
// vez que ve la bandera del equipo contrario, nada más. Si la Pi nunca se
// conecta o se cae, CAM_LINK simplemente no recibe nada y el giro cae en
// su tope de tiempo de siempre -- no hay forma de que la ausencia de la Pi
// rompa la misión.
#define CAM_LINK Serial

// ===========================================================================
//  [2] TIPOS COMPARTIDOS ENTRE TAREAS
// ===========================================================================

enum class TeamColor     : uint8_t { NONE = 0, RED = 1, BLUE = 2 };
enum class MotorMode     : uint8_t { STOP = 0, DRIVE = 1 };
enum class GripperAction : uint8_t { OPEN = 0, CLOSE_LLAVE = 1, CLOSE_BANDERA = 2, OPEN_SLOW = 3 };
enum class ColorLabel    : uint8_t { UNKNOWN = 0, BLACK, YELLOW, RED, BLUE, FLOOR };

// Fase "visible" para el LED -- MissionTask decide, LedTask solo traduce.
// COLOR: mientras se busca una zona por color, el LED muestra el color
// sensado en vivo (igual que v3). Las demás fases muestran un indicador
// de progreso fijo/parpadeante (igual que v5).
enum class EstadoVisible : uint8_t {
    COLOR = 0,        // buscando zona -- LED = color sensado en vivo
    BUSCANDO_BANDERA, // acercamiento grueso a la bandera (ToF)
    AJUSTANDO,        // parado, midiendo/dando pasos chicos hacia la bandera
    AGARRADA,         // gripper cerrado sobre la bandera, avanzando
    TERMINADO,
    FALLO,            // se agotaron los pasos de ajuste sin confirmar rango
    BORDE,            // M1: reaccionando al borde negro de la pista (morado fijo)
};

struct MotorCommand {
    MotorMode mode = MotorMode::STOP;
    int8_t    left  = 0;
    int8_t    right = 0;
};

struct GripperCommand {
    GripperAction action = GripperAction::OPEN;
};

struct LedCommand {
    EstadoVisible estado = EstadoVisible::COLOR;
    ColorLabel    color  = ColorLabel::UNKNOWN;   // solo se usa si estado == COLOR
};

struct ColorReading {
    uint32_t   timestamp_ms = 0;
    ColorLabel color        = ColorLabel::UNKNOWN;
    bool       valid        = false;
};

struct TofReading {
    uint32_t timestamp_ms = 0;
    uint16_t distance_mm  = 0;
    bool     valid        = false;
};

// Lectura del QTR con rechazo de luz ambiente: dif = (emisor encendido) - (emisor apagado).
struct ReflectanceReading {
    uint32_t timestamp_ms  = 0;
    int16_t  left_restado  = 0;
    int16_t  right_restado = 0;
    bool     left_pegado   = false;   // pegado al tope del ADC: sin señal útil
    bool     right_pegado  = false;
    bool     left_on_line  = false;   // true = ve negro (borde)
    bool     right_on_line = false;
};

struct HealthReport {
    uint32_t timestamp_ms          = 0;
    uint8_t  faulted_tasks_bitmask = 0;
};

// ===========================================================================
//  [3] COLAS Y MUTEX DEL BUS I2C Nº1
// ===========================================================================
//
//   MissionTask     --> motorCmdQueue   (1, overwrite) --> MotorTask
//                   --> gripperCmdQueue (4, FIFO)       --> GripperTask
//                   --> ledCmdQueue     (1, overwrite)  --> LedTask
//   ColorSensorTask --> colorQueue      (4, FIFO)       --> MissionTask
//   TofSensorTask   --> tofQueue        (4, FIFO)       --> MissionTask
//   SupervisorTask  --> healthQueue     (1, overwrite)  --> MissionTask (solo log)

static QueueHandle_t g_motorCmdQueue   = nullptr;
static QueueHandle_t g_gripperCmdQueue = nullptr;
static QueueHandle_t g_ledCmdQueue     = nullptr;
static QueueHandle_t g_colorQueue      = nullptr;
static QueueHandle_t g_tofQueue        = nullptr;
static QueueHandle_t g_reflectQueue    = nullptr;
static QueueHandle_t g_healthQueue     = nullptr;

// Protege el bus Nº1 (Wire1): el TCS34725 delantero (ColorSensorTask,
// núcleo 0) y el PCA9685 (GripperTask, núcleo 1) lo comparten ahora que el
// color se movió a este bus -- ver el aviso grande al principio del
// archivo. El bus Nº0 (Wire, ToF) lo toca ÚNICAMENTE TofSensorTask, así
// que no necesita mutex propio.
static SemaphoreHandle_t g_i2c1Mutex = nullptr;
constexpr uint32_t I2C1_LOCK_TIMEOUT_MS = 50;

static inline bool I2c1Lock() {
    return xSemaphoreTake(g_i2c1Mutex, pdMS_TO_TICKS(I2C1_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline void I2c1Unlock() {
    xSemaphoreGive(g_i2c1Mutex);
}

// ===========================================================================
//  [4] WATCHDOG COOPERATIVO
// ===========================================================================

static volatile uint32_t g_lastHeartbeatMs[(size_t)TaskId::COUNT];

static void WatchdogInit() {
    const uint32_t now = millis();
    for (size_t i = 0; i < (size_t)TaskId::COUNT; ++i) g_lastHeartbeatMs[i] = now;
}

static inline void Heartbeat(TaskId id) {
    g_lastHeartbeatMs[(size_t)id] = millis();
}

static uint8_t WatchdogCheck() {
    const uint32_t now = millis();
    uint8_t faulted = 0;
    for (size_t i = 0; i < (size_t)TaskId::COUNT; ++i) {
        if ((uint32_t)(now - g_lastHeartbeatMs[i]) > WATCHDOG_TIMEOUT_MS) {
            faulted |= (uint8_t)(1u << i);
        }
    }
    return faulted;
}

// ===========================================================================
//  [5] DRIVER PCA9685 — servo del gripper, bus I2C nº1
// ===========================================================================

namespace Pca9685 {
    constexpr uint8_t REG_MODE1    = 0x00;
    constexpr uint8_t REG_MODE2    = 0x01;
    constexpr uint8_t REG_LED0_ON_L = 0x06;
    constexpr uint8_t REG_PRESCALE = 0xFE;

    constexpr uint8_t MODE1_RESTART = 0x80;
    constexpr uint8_t MODE1_AI      = 0x20;
    constexpr uint8_t MODE1_SLEEP   = 0x10;
    constexpr uint8_t MODE2_OUTDRV  = 0x04;

    bool WriteReg(uint8_t reg, uint8_t value) {
        Wire1.beginTransmission(I2CAddr::PCA9685);
        Wire1.write(reg);
        Wire1.write(value);
        return Wire1.endTransmission() == 0;
    }

    bool ReadReg(uint8_t reg, uint8_t &out) {
        Wire1.beginTransmission(I2CAddr::PCA9685);
        Wire1.write(reg);
        if (Wire1.endTransmission(false) != 0) return false;
        if (Wire1.requestFrom((int)I2CAddr::PCA9685, 1) != 1) return false;
        out = (uint8_t)Wire1.read();
        return true;
    }

    bool Init(uint32_t freq_hz) {
        if (!WriteReg(REG_MODE1, MODE1_SLEEP)) return false;
        const uint32_t prescale = (25000000UL / (4096UL * freq_hz)) - 1UL;
        if (!WriteReg(REG_PRESCALE, (uint8_t)prescale)) return false;
        if (!WriteReg(REG_MODE1, MODE1_AI)) return false;
        delayMicroseconds(500);
        if (!WriteReg(REG_MODE1, MODE1_AI | MODE1_RESTART)) return false;
        if (!WriteReg(REG_MODE2, MODE2_OUTDRV)) return false;
        uint8_t check = 0;
        return ReadReg(REG_MODE1, check);
    }

    bool SetChannel(uint8_t channel, uint16_t ticks) {
        if (channel > 15) return false;
        if (ticks > 4095) ticks = 4095;
        Wire1.beginTransmission(I2CAddr::PCA9685);
        Wire1.write(REG_LED0_ON_L + 4 * channel);
        Wire1.write(0x00);
        Wire1.write(0x00);
        Wire1.write((uint8_t)(ticks & 0xFF));
        Wire1.write((uint8_t)(ticks >> 8));
        return Wire1.endTransmission() == 0;
    }
}

static uint16_t ServoAngleToTicks(int angle_deg) {
    angle_deg = constrain(angle_deg, 0, 180);
    return (uint16_t)(Pwm::SERVO_TICK_MIN +
        ((uint32_t)(Pwm::SERVO_TICK_MAX - Pwm::SERVO_TICK_MIN) * (uint32_t)angle_deg) / 180UL);
}

// ===========================================================================
//  [6] DRIVER TCS34725 — color delantero, bus I2C nº1
// ===========================================================================

namespace Tcs34725 {
    constexpr uint8_t CMD_BIT      = 0x80;
    constexpr uint8_t CMD_AUTO_INC = 0x20;

    constexpr uint8_t REG_ENABLE  = 0x00;
    constexpr uint8_t REG_ATIME   = 0x01;
    constexpr uint8_t REG_CONTROL = 0x0F;
    constexpr uint8_t REG_ID      = 0x12;
    constexpr uint8_t REG_CDATAL  = 0x14;

    constexpr uint8_t ENABLE_PON = 0x01;
    constexpr uint8_t ENABLE_AEN = 0x02;

    constexpr uint8_t ATIME_24MS = 0xEB;
    constexpr uint8_t GAIN_4X    = 0x01;

    struct Rgbc { uint16_t c = 0, r = 0, g = 0, b = 0; };

    bool WriteReg(uint8_t reg, uint8_t value) {
        Wire1.beginTransmission(I2CAddr::TCS34725);
        Wire1.write(CMD_BIT | reg);
        Wire1.write(value);
        return Wire1.endTransmission() == 0;
    }

    bool ReadReg(uint8_t reg, uint8_t &out) {
        Wire1.beginTransmission(I2CAddr::TCS34725);
        Wire1.write(CMD_BIT | reg);
        if (Wire1.endTransmission() != 0) return false;
        if (Wire1.requestFrom((int)I2CAddr::TCS34725, 1) != 1) return false;
        out = (uint8_t)Wire1.read();
        return true;
    }

    bool Init() {
        uint8_t id = 0;
        if (!ReadReg(REG_ID, id)) return false;
        if (id != 0x44 && id != 0x4D) return false;

        if (!WriteReg(REG_ATIME, ATIME_24MS)) return false;
        if (!WriteReg(REG_CONTROL, GAIN_4X)) return false;
        if (!WriteReg(REG_ENABLE, ENABLE_PON)) return false;
        delay(3);
        return WriteReg(REG_ENABLE, ENABLE_PON | ENABLE_AEN);
    }

    bool Read(Rgbc &out) {
        Wire1.beginTransmission(I2CAddr::TCS34725);
        Wire1.write(CMD_BIT | CMD_AUTO_INC | REG_CDATAL);
        if (Wire1.endTransmission() != 0) return false;
        if (Wire1.requestFrom((int)I2CAddr::TCS34725, 8) != 8) return false;

        out.c = (uint16_t)(Wire1.read() | (Wire1.read() << 8));
        out.r = (uint16_t)(Wire1.read() | (Wire1.read() << 8));
        out.g = (uint16_t)(Wire1.read() | (Wire1.read() << 8));
        out.b = (uint16_t)(Wire1.read() | (Wire1.read() << 8));
        return true;
    }
}

// ===========================================================================
//  [7] CLASIFICACIÓN DE COLOR -> ETIQUETA
// ===========================================================================
//
//  Umbrales del DELANTERO, reajustados 2026-09-07 -- MISMOS que
//  calibracion/color/detector-tcs/src/main.cpp y v3-color-delantero. Se
//  calibraron con el TCS34725 en el bus 0; ahora vive en el bus 1 (ver el
//  aviso grande al principio del archivo) -- si el bus nuevo le cambia el
//  ruido eléctrico percibido, la primera señal de alerta sería confusiones
//  entre GRIS y AMARILLO, igual que las que ya pasaron el 2026-09-07.

static ColorLabel ClassifyColor(const Tcs34725::Rgbc &s) {
    if (s.c < 314) return ColorLabel::BLACK;

    const float total = (float)s.c;
    const float r = (float)s.r / total;
    const float g = (float)s.g / total;
    const float b = (float)s.b / total;

    if (r > 0.450f && g < 0.312f && b < 0.300f) return ColorLabel::RED;
    if (b > 0.206f && r < 0.390f)               return ColorLabel::BLUE;
    if (r > 0.420f && g > 0.200f && b < 0.140f) return ColorLabel::YELLOW;

    return ColorLabel::FLOOR;
}

inline const char *ColorLabelName(ColorLabel label) {
    switch (label) {
        case ColorLabel::BLACK:   return "NEGRO";
        case ColorLabel::YELLOW:  return "AMARILLO";
        case ColorLabel::RED:     return "ROJO";
        case ColorLabel::BLUE:    return "AZUL";
        case ColorLabel::FLOOR:   return "GRIS/PISO";
        case ColorLabel::UNKNOWN:
        default:                  return "DESCONOCIDO";
    }
}

// ===========================================================================
//  [8] DRIVER VL53L1X (Pololu) — bus I2C nº0, SOLO
// ===========================================================================

namespace Tof {
    constexpr uint16_t TIMING_BUDGET_US = 50000;   // 50 ms
    constexpr uint32_t RANGING_PERIOD_MS = 50;
}

VL53L1X g_tof;

bool TofBringUp() {
    g_tof.setBus(&Wire);
    g_tof.setTimeout(500);
    // Sin setAddress(): el bus 0 es solo suyo, se queda en su dirección de
    // fábrica -- ver el aviso grande al principio del archivo.
    if (!g_tof.init()) return false;
    g_tof.setDistanceMode(VL53L1X::Long);
    g_tof.setMeasurementTimingBudget(Tof::TIMING_BUDGET_US);
    g_tof.startContinuous(Tof::RANGING_PERIOD_MS);
    return true;
}

// ===========================================================================
//  [9] TAREAS DE HARDWARE
// ===========================================================================

namespace {

// ---------------------------------------------------------------------------
//  9.1  MotorTask — 4 motores a través de 2 drivers L298N
// ---------------------------------------------------------------------------

struct Motor {
    uint8_t in1, in2, en, ledc_channel;
};

constexpr Motor kMotorFL = {Pins::L298N_L_IN2, Pins::L298N_L_IN1, Pins::L298N_L_ENA, 0};
constexpr Motor kMotorRL = {Pins::L298N_L_IN3, Pins::L298N_L_IN4, Pins::L298N_L_ENB, 1};
constexpr Motor kMotorFR = {Pins::L298N_R_IN1, Pins::L298N_R_IN2, Pins::L298N_R_ENA, 2};
constexpr Motor kMotorRR = {Pins::L298N_R_IN3, Pins::L298N_R_IN4, Pins::L298N_R_ENB, 3};

void PwmAttach(uint8_t pin, uint8_t channel, uint32_t freqHz, uint8_t resolution) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;
    ledcAttach(pin, freqHz, resolution);
#else
    ledcSetup(channel, freqHz, resolution);
    ledcAttachPin(pin, channel);
#endif
}

void PwmWrite(uint8_t pin, uint8_t channel, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;
    ledcWrite(pin, duty);
#else
    (void)pin;
    ledcWrite(channel, duty);
#endif
}

void MotorSetup(const Motor &m) {
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
    PwmAttach(m.en, m.ledc_channel, Pwm::MOTOR_FREQ_HZ, Pwm::MOTOR_RESOLUTION);
    PwmWrite(m.en, m.ledc_channel, 0);
}

// speed > 0 -> "avanza" -- misma convención confirmada en banco que
// v3-color-delantero/v4-merodeo-color/v5-agarrar-bandera.
void MotorApply(const Motor &m, int speed) {
    speed = constrain(speed, -100, 100);
    const bool forward = (speed > 0);
    digitalWrite(m.in1, forward ? HIGH : LOW);
    digitalWrite(m.in2, forward ? LOW  : HIGH);
    PwmWrite(m.en, m.ledc_channel, (uint32_t)abs(speed) * 255u / 100u);
}

void MotorsStop() {
    MotorApply(kMotorFL, 0);
    MotorApply(kMotorRL, 0);
    MotorApply(kMotorFR, 0);
    MotorApply(kMotorRR, 0);
}

void MotorTask(void *) {
    MotorSetup(kMotorFL);
    MotorSetup(kMotorRL);
    MotorSetup(kMotorFR);
    MotorSetup(kMotorRR);
    MotorsStop();

    MotorCommand current{};
    uint32_t last_cmd_ms = millis();

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::MOTOR_CONTROL);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        MotorCommand incoming;
        if (xQueueReceive(g_motorCmdQueue, &incoming, 0) == pdTRUE) {
            current = incoming;
            last_cmd_ms = millis();
        }

        const bool mission_stale =
            (uint32_t)(millis() - last_cmd_ms) > MISSION_FAILSAFE_TIMEOUT_MS;

        if (mission_stale || current.mode == MotorMode::STOP) {
            MotorsStop();
        } else {
            MotorApply(kMotorFL, current.left);
            MotorApply(kMotorRL, current.left);
            MotorApply(kMotorFR, current.right);
            MotorApply(kMotorRR, current.right);
        }

        Heartbeat(TaskId::MOTOR_CONTROL);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  9.2  GripperTask — 1 servo vía PCA9685 (bus I2C nº1, con mutex)
// ---------------------------------------------------------------------------

void GripperTask(void *) {
    bool pca_ok = false;
    if (I2c1Lock()) {
        pca_ok = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
        if (pca_ok) {
            Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
        }
        I2c1Unlock();
    }
    if (!pca_ok) {
        DEBUG_LINK.println("[Gripper] PCA9685 no responde. Reintentando en segundo plano.");
    }

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::GRIPPER);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        if (!pca_ok && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            if (I2c1Lock()) {
                pca_ok = Pca9685::Init(Pwm::SERVO_FREQ_HZ);
                if (pca_ok) {
                    pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
                }
                I2c1Unlock();
            }
        }

        GripperCommand cmd;
        if (xQueueReceive(g_gripperCmdQueue, &cmd, 0) == pdTRUE && pca_ok) {
            if (cmd.action == GripperAction::OPEN_SLOW) {
                // Apertura lenta (paso 12: soltar la bandera suavemente): baja del ángulo
                // de agarre de la bandera al abierto en pasos, tomando el bus solo un
                // instante por paso para no dejar sin bus al sensor de color.
                for (int a = kClawClosedBanderaDeg; a > kClawOpenDeg; a -= Mission::kSoltarLentoPasoDeg) {
                    if (I2c1Lock()) {
                        pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(a));
                        I2c1Unlock();
                    }
                    vTaskDelay(pdMS_TO_TICKS(Mission::kSoltarLentoPasoMs));
                }
                if (I2c1Lock()) {
                    pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
                    I2c1Unlock();
                }
            } else if (I2c1Lock()) {
                switch (cmd.action) {
                    case GripperAction::OPEN:
                        pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
                        break;
                    case GripperAction::CLOSE_LLAVE:
                        pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawClosedLlaveDeg));
                        break;
                    case GripperAction::CLOSE_BANDERA:
                        pca_ok = Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawClosedBanderaDeg));
                        break;
                    default:
                        break;
                }
                I2c1Unlock();
            }
        }

        Heartbeat(TaskId::GRIPPER_CONTROL);
        vTaskDelayUntil(&last_wake, period);
    }
}

template <typename T>
void PushDropOldest(QueueHandle_t queue, const T &item) {
    if (xQueueSend(queue, &item, 0) != pdTRUE) {
        T discard;
        xQueueReceive(queue, &discard, 0);
        xQueueSend(queue, &item, 0);
    }
}

// ---------------------------------------------------------------------------
//  9.3  ColorSensorTask — TCS34725 delantero, bus I2C nº1 (con mutex)
// ---------------------------------------------------------------------------

void ColorSensorTask(void *) {
    pinMode(Pins::TCS_LED_FRONT, OUTPUT);
    digitalWrite(Pins::TCS_LED_FRONT, HIGH);   // iluminación fija: no depender de la luz del salón

    bool front_ok = false;
    if (I2c1Lock()) {
        front_ok = Tcs34725::Init();
        I2c1Unlock();
    }
    if (!front_ok) DEBUG_LINK.println("[Color] TCS34725 delantero no responde (bus I2C 1).");

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::COLOR_SENSOR);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        if (!front_ok && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            if (I2c1Lock()) {
                front_ok = Tcs34725::Init();
                I2c1Unlock();
            }
        }

        ColorReading reading;
        reading.timestamp_ms = millis();

        if (front_ok) {
            Tcs34725::Rgbc sample;
            bool read_ok = false;
            if (I2c1Lock()) {
                read_ok = Tcs34725::Read(sample);
                I2c1Unlock();
            }
            if (read_ok) {
                reading.color = ClassifyColor(sample);
                reading.valid = true;
            } else {
                front_ok = false;
            }
        }

        PushDropOldest(g_colorQueue, reading);

        Heartbeat(TaskId::COLOR_SENSOR);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  9.4  TofSensorTask — VL53L1X, bus I2C nº0 (sin mutex, único dueño)
// ---------------------------------------------------------------------------

void TofSensorTask(void *) {
    pinMode(Pins::TOF_XSHUT, OUTPUT);
    digitalWrite(Pins::TOF_XSHUT, HIGH);   // fuera de reset; sin reasignación (ver aviso arriba)

    Wire.begin(Pins::I2C0_SDA, Pins::I2C0_SCL);   // bus 0 -- SOLO el ToF

    bool tof_ok = TofBringUp();
    if (!tof_ok) DEBUG_LINK.println("[ToF] VL53L1X no responde. Reintentando en segundo plano.");

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::TOF_SENSOR);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_retry_ms = millis();

    for (;;) {
        if (!tof_ok && (uint32_t)(millis() - last_retry_ms) > 1000) {
            last_retry_ms = millis();
            tof_ok = TofBringUp();
        }

        TofReading reading;
        reading.timestamp_ms = millis();

        if (tof_ok) {
            if (g_tof.dataReady()) {
                reading.distance_mm = g_tof.read(false);
                reading.valid = !g_tof.timeoutOccurred() &&
                                 g_tof.ranging_data.range_status == VL53L1X::RangeValid;
                if (g_tof.timeoutOccurred()) tof_ok = false;
            }
        }

        PushDropOldest(g_tofQueue, reading);

        Heartbeat(TaskId::TOF_SENSOR);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ---------------------------------------------------------------------------
//  9.5  LedTask — color sensado (fases COLOR) o indicador de fase (resto)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
//  ReflectanceTask -- 2x QTRX-HD-01A con rechazo de luz ambiente (hito M1)
// ---------------------------------------------------------------------------
// Misma técnica que firmware-esp32/ y v4: emisor IR APAGADO (>= 1 ms, menos
// solo lo atenúa) -> lectura off; ENCENDIDO -> lectura on; dif = on - off.
// Un sensor con off y on pegados al tope del ADC no da información: se marca
// pegado y NO cuenta como negro (leería dif=0 = negro todo el tiempo).
void ReflectanceTask(void *) {
    analogReadResolution(12);
    analogSetPinAttenuation(Pins::QTR_LEFT_OUT, ADC_11db);
    analogSetPinAttenuation(Pins::QTR_RIGHT_OUT, ADC_11db);

    pinMode(Pins::QTR_EMITTER_CTRL, OUTPUT);
    digitalWrite(Pins::QTR_EMITTER_CTRL, HIGH);

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::REFLECTANCE);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        digitalWrite(Pins::QTR_EMITTER_CTRL, LOW);
        delay(2);   // >= 1 ms: apagado real
        const uint16_t left_off  = (uint16_t)analogRead(Pins::QTR_LEFT_OUT);
        const uint16_t right_off = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);

        digitalWrite(Pins::QTR_EMITTER_CTRL, HIGH);
        delayMicroseconds(200);   // asentar el fototransistor con luz IR estable
        const uint16_t left_on  = (uint16_t)analogRead(Pins::QTR_LEFT_OUT);
        const uint16_t right_on = (uint16_t)analogRead(Pins::QTR_RIGHT_OUT);

        ReflectanceReading r;
        r.timestamp_ms  = millis();
        r.left_restado  = (int16_t)((int32_t)left_on  - (int32_t)left_off);
        r.right_restado = (int16_t)((int32_t)right_on - (int32_t)right_off);
        r.left_pegado   = left_off  >= Mission::kQtrTopeAdc && left_on  >= Mission::kQtrTopeAdc;
        r.right_pegado  = right_off >= Mission::kQtrTopeAdc && right_on >= Mission::kQtrTopeAdc;
        r.right_on_line = !r.right_pegado && abs(r.right_restado) < Mission::kBordeRestadoUmbral;
        r.left_on_line  = Mission::kQtrIzquierdoActivo && !r.left_pegado &&
                          abs(r.left_restado) < Mission::kBordeRestadoUmbral;

        PushDropOldest(g_reflectQueue, r);

        Heartbeat(TaskId::REFLECTANCE);
        vTaskDelayUntil(&last_wake, period);
    }
}

namespace RgbLed {
    constexpr uint8_t CH_R = 4;
    constexpr uint8_t CH_G = 5;
    constexpr uint8_t CH_B = 6;

    constexpr bool kCommonAnode = false;   // cátodo común, duty alto = más brillante

    void Setup() {
        PwmAttach(Pins::LED_RGB_R, CH_R, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(Pins::LED_RGB_G, CH_G, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
        PwmAttach(Pins::LED_RGB_B, CH_B, Pwm::RGB_FREQ_HZ, Pwm::RGB_RESOLUTION);
    }

    void SetRaw(uint8_t r, uint8_t g, uint8_t b) {
        if (kCommonAnode) { r = 255 - r; g = 255 - g; b = 255 - b; }
        PwmWrite(Pins::LED_RGB_R, CH_R, r);
        PwmWrite(Pins::LED_RGB_G, CH_G, g);
        PwmWrite(Pins::LED_RGB_B, CH_B, b);
    }

    void ApplyColor(ColorLabel color) {
        switch (color) {
            case ColorLabel::FLOOR:   SetRaw(60, 60, 60);  break;   // blanco -- piso gris
            case ColorLabel::YELLOW:  SetRaw(255, 170, 0); break;
            case ColorLabel::RED:     SetRaw(255, 0, 0);   break;
            case ColorLabel::BLUE:    SetRaw(0, 0, 255);   break;
            case ColorLabel::BLACK:
            case ColorLabel::UNKNOWN:
            default:
                SetRaw(0, 0, 0);
                break;
        }
    }
}

void LedTask(void *) {
    RgbLed::Setup();
    RgbLed::SetRaw(0, 0, 0);

    EstadoVisible estado = EstadoVisible::COLOR;
    ColorLabel color = ColorLabel::UNKNOWN;
    uint32_t blink_ms = millis();
    bool blink_on = false;

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::LED_STATUS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        LedCommand cmd;
        if (xQueueReceive(g_ledCmdQueue, &cmd, 0) == pdTRUE) {
            estado = cmd.estado;
            color  = cmd.color;
        }

        if ((uint32_t)(millis() - blink_ms) > 300) {
            blink_ms = millis();
            blink_on = !blink_on;
        }

        switch (estado) {
            case EstadoVisible::COLOR:
                RgbLed::ApplyColor(color);
                break;
            case EstadoVisible::BUSCANDO_BANDERA:
                RgbLed::SetRaw(40, 40, 40);   // blanco tenue fijo
                break;
            case EstadoVisible::AJUSTANDO:
                RgbLed::SetRaw(blink_on ? 255 : 0, blink_on ? 190 : 0, 0);   // amarillo parpadeando
                break;
            case EstadoVisible::AGARRADA:
                RgbLed::SetRaw(0, 255, 0);    // verde fijo
                break;
            case EstadoVisible::TERMINADO:
                RgbLed::SetRaw(0, 255, 0);    // verde fijo
                break;
            case EstadoVisible::FALLO:
                RgbLed::SetRaw(blink_on ? 255 : 0, 0, 0);   // rojo parpadeando
                break;
            case EstadoVisible::BORDE:
                RgbLed::SetRaw(160, 0, 200);  // morado fijo
                break;
        }

        Heartbeat(TaskId::LED_STATUS);
        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace anónimo (tareas de hardware)

// ===========================================================================
//  [10] MISSIONTASK — caja, zona amarilla, bandera, zona enemiga
//  (los parámetros ajustables de Mission:: y las fases viven en la sección
//  [0] PARÁMETROS AJUSTABLES, al principio del archivo)
// ===========================================================================

inline void SetDrive(MotorCommand &m, int left, int right) {
    m.mode  = MotorMode::DRIVE;
    m.left  = (int8_t)constrain(left, -100, 100);
    m.right = (int8_t)constrain(right, -100, 100);
}

// ---------------------------------------------------------------------------
//  9.6  Señal de cámara -- líneas de texto por CAM_LINK (y por el UART, para simular)
// ---------------------------------------------------------------------------
// La Raspberry Pi manda una línea por cuadro: 'B <error> <área>' si ve la
// bandera contraria, 'N' si no. Sin framing ni checksum a propósito: un byte
// perdido solo estropea UNA línea y el salto de línea resincroniza. Los 'V'
// sueltos del protocolo viejo de v7 se descartan. También se lee el puerto UART
// (DEBUG_LINK) para poder teclear líneas a mano en el monitor y probar sin la Pi.
// Es seguro llamarlo en cualquier fase; lo que cambia es si MissionTask actúa.
namespace CamaraBandera {
    volatile uint32_t ultima_b_ms = 0;          // millis() de la última línea B; 0 = no la ve (N) o nunca llegó
    volatile uint32_t sostenida_desde_ms = 0;   // 0 = no está fresca ahora mismo
    volatile int      error = 0;                // -100..100
    volatile int      area  = 0;                // 0..100, informativo

    static char    linea[24];
    static uint8_t largo = 0;

    static void ProcesarLinea(const char *l) {
        if (l[0] == 'B') {
            int e = 0, a = 0;
            const int n = sscanf(l + 1, "%d %d", &e, &a);
            if (n >= 1) {
                error = constrain(e, -100, 100);
                area  = (n >= 2) ? constrain(a, 0, 100) : 0;
                ultima_b_ms = millis();
            }
        } else if (l[0] == 'N') {
            ultima_b_ms = 0;
        }
    }

    static void Alimentar(char c) {
        if (c == 'V') return;   // protocolo viejo de v7: se descarta
        if (c == '\n' || c == '\r') {
            if (largo > 0) { linea[largo] = 0; ProcesarLinea(linea); largo = 0; }
            return;
        }
        if (largo < sizeof(linea) - 1) linea[largo++] = c; else largo = 0;
    }

    // true = llegó una línea B hace menos de kFrescoBanderaCamaraMs.
    bool Fresca() {
        return ultima_b_ms != 0 &&
               (uint32_t)(millis() - ultima_b_ms) <= Mission::kFrescoBanderaCamaraMs;
    }

    void Actualizar() {
        while (CAM_LINK.available() > 0)   Alimentar((char)CAM_LINK.read());
        while (DEBUG_LINK.available() > 0) Alimentar((char)DEBUG_LINK.read());

        if (Fresca()) {
            if (sostenida_desde_ms == 0) sostenida_desde_ms = millis();
        } else {
            sostenida_desde_ms = 0;
        }
    }

    // true = la señal lleva fresca de forma continua al menos
    // kSostenBanderaCamaraMs -- no es "la vio una vez", es "la sigue viendo".
    bool Confirmada() {
        return sostenida_desde_ms != 0 &&
               (uint32_t)(millis() - sostenida_desde_ms) >= Mission::kSostenBanderaCamaraMs;
    }

    int Error() { return error; }
    int Area()  { return area; }

    // Difunde el equipo (ya decidido por el switch físico en setup()) a la
    // Raspberry Pi, repetido cada kEquipoBroadcastMs en vez de una sola vez
    // al arrancar -- así avisar_bandera_v7.py lo recibe también si se
    // conecta o se reinicia después de este firmware, sin tener que
    // reiniciar el ESP32. Un solo byte, mismo espíritu que 'V': 'R' = ROJO,
    // 'A' = AZUL.
    void EnviarEquipo(TeamColor team) {
        static uint32_t ultimo_envio_ms = 0;
        const uint32_t ahora = millis();
        if ((uint32_t)(ahora - ultimo_envio_ms) < Mission::kEquipoBroadcastMs) return;
        ultimo_envio_ms = ahora;
        CAM_LINK.write(team == TeamColor::RED ? 'R' : 'A');
    }
}

// pvTeam apunta a g_myTeam (global, ver setup()) -- de ahí se deriva
// también g_enemyColor, calculado una sola vez al arrancar la tarea.
void MissionTask(void *pvTeam) {
    const TeamColor team = *reinterpret_cast<TeamColor *>(pvTeam);
    const ColorLabel enemy_color = (team == TeamColor::RED) ? ColorLabel::BLUE : ColorLabel::RED;
    const ColorLabel own_color   = (team == TeamColor::RED) ? ColorLabel::RED  : ColorLabel::BLUE;

    Mission::Phase phase = Mission::Phase::ARRANQUE;
    Mission::Phase last_logged_phase = phase;
    uint32_t phase_started_ms = millis();

    ColorReading last_color{};
    TofReading last_tof{};
    ReflectanceReading last_reflect{};

    // Protección de borde (M1): cuánto lleva visto el negro, a qué fase volver
    // tras reaccionar, cuántas veces ha reaccionado y si ya se avisó de un QTR pegado.
    uint32_t borde_desde_ms = 0;   // 0 = no se está confirmando ahora
    Mission::Phase fase_interrumpida = Mission::Phase::ARRANQUE;
    uint32_t borde_eventos = 0;
    bool aviso_qtr_pegado_dado = false;

    // Paso 7 (M2): ciclo de búsqueda y centrado por pulsos.
    int      busq_paso = 0;             // 0..7, ver Mission::kBusqueda*
    uint32_t busq_paso_desde_ms = 0;
    int      centr_sub = 0;             // 0 = pausa/lectura, 1 = pulso de giro
    uint32_t centr_sub_desde_ms = 0;
    int      centr_sentido = 1;         // +1 = gira a la derecha, -1 = a la izquierda
    uint32_t perdida_desde_ms = 0;      // 0 = la cámara la ve ahora
    uint32_t last_cam_log_ms = 0;
    uint32_t last_ret_log_ms = 0;
    bool     soltar_lento_enviado = false;   // la apertura lenta se pide UNA sola vez, no en cada ciclo

    // Cerrojos: en cuanto se ve el color buscado UNA vez en la fase
    // correspondiente, esto pasa a true y ya no vuelve a false -- mismo
    // criterio que v1/v3 (no seguir de largo por una lectura suelta que
    // cambió un instante después).
    bool zona_amarilla_detectada = false;
    bool zona_enemiga_detectada  = false;

    // Log periódico de diagnóstico SOLO en AVANZAR_ZONA_ENEMIGA -- a
    // diferencia del log de cambio de fase de más abajo, este imprime en
    // vivo mientras se busca el color de la zona enemiga, para poder ver
    // qué está leyendo el sensor en el momento exacto en que debería
    // reconocer rojo/azul y no lo hace.
    uint32_t last_color_debug_ms = 0;

    int lecturas_en_rango_seguidas = 0;
    int pasos_dados = 0;
    int sentido_paso = 1;   // -1 = paso hacia atrás, +1 = hacia adelante

    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::MISSION);
    TickType_t last_wake = xTaskGetTickCount();

    DEBUG_LINK.printf("[Mission] v8-logica-completa (M5) -- equipo=%s, zona enemiga=%s\n",
                       team == TeamColor::RED ? "ROJO" : "AZUL",
                       ColorLabelName(enemy_color));

    for (;;) {
        CamaraBandera::Actualizar();
        CamaraBandera::EnviarEquipo(team);

        ColorReading c;
        while (xQueueReceive(g_colorQueue, &c, 0) == pdTRUE) last_color = c;
        TofReading t;
        while (xQueueReceive(g_tofQueue, &t, 0) == pdTRUE) last_tof = t;
        ReflectanceReading rf;
        while (xQueueReceive(g_reflectQueue, &rf, 0) == pdTRUE) last_reflect = rf;
        HealthReport h;
        while (xQueueReceive(g_healthQueue, &h, 0) == pdTRUE) {
            DEBUG_LINK.printf("[Mission] tareas colgadas, bitmask=0x%02X\n", h.faulted_tasks_bitmask);
        }

        const ColorLabel color_activo = last_color.valid ? last_color.color : ColorLabel::UNKNOWN;

        MotorCommand motor;   // por defecto: STOP
        GripperCommand gripper;
        bool send_gripper = false;
        EstadoVisible estado_led = EstadoVisible::COLOR;
        ColorLabel led_color = color_activo;

        if (Mission::kMotionEnabled) {

            if ((phase == Mission::Phase::BUSCAR_BANDERA || phase == Mission::Phase::CENTRAR_Y_AVANZAR ||
                 phase == Mission::Phase::DETENER_PARA_MEDIR || phase == Mission::Phase::PASO_AJUSTE ||
                 phase == Mission::Phase::CENTRAR_FINO) &&
                (uint32_t)(millis() - last_cam_log_ms) > 300) {
                last_cam_log_ms = millis();
                DEBUG_LINK.printf("[Camara] %s fresca=%d sostenida=%d err=%d area=%d%% | tof=%u mm valido=%d\n",
                                   Mission::PhaseName(phase), CamaraBandera::Fresca(), CamaraBandera::Confirmada(),
                                   CamaraBandera::Error(), CamaraBandera::Area(),
                                   last_tof.distance_mm, last_tof.valid);
            }

            if ((phase == Mission::Phase::SALIR_ZONA_RIVAL || phase == Mission::Phase::VOLVER_ZONA_PROPIA) &&
                (uint32_t)(millis() - last_ret_log_ms) > 300) {
                last_ret_log_ms = millis();
                DEBUG_LINK.printf("[Retorno] %s color=%s valido=%d (franja rival=%s, franja propia=%s)\n",
                                   Mission::PhaseName(phase), ColorLabelName(color_activo), last_color.valid,
                                   ColorLabelName(enemy_color), ColorLabelName(own_color));
            }

            // --- Protección de borde (QTR): prioridad sobre las fases que
            // avanzan o giran (no al retroceder ni quieto). Confirma el negro
            // kBordeDebounceMs, guarda la fase y reacciona (BORDE_*). ---------
            if (last_reflect.right_pegado && !aviso_qtr_pegado_dado) {
                aviso_qtr_pegado_dado = true;
                DEBUG_LINK.println("[Borde] AVISO: QTR derecho pegado al tope -- SIN proteccion de borde");
            }
            {
                const bool fase_con_borde =
                    phase == Mission::Phase::BUSCAR_ZONA_AMARILLA ||
                    phase == Mission::Phase::BUSCAR_BANDERA ||
                    phase == Mission::Phase::CENTRAR_Y_AVANZAR ||
                    phase == Mission::Phase::GIRO_RETORNO ||
                    phase == Mission::Phase::SALIR_ZONA_RIVAL ||
                    phase == Mission::Phase::VOLVER_ZONA_PROPIA ||
                    phase == Mission::Phase::ESQUIVAR_CAJA ||
                    phase == Mission::Phase::AVANZAR_TRAS_ESQUIVE ||
                    phase == Mission::Phase::GIRO_RECENTRAR;
                const bool en_borde = last_reflect.right_on_line || last_reflect.left_on_line;
                if (Mission::kProteccionBordeActiva && fase_con_borde && en_borde) {
                    if (borde_desde_ms == 0) {
                        borde_desde_ms = millis();
                    } else if ((uint32_t)(millis() - borde_desde_ms) >= Mission::kBordeDebounceMs) {
                        ++borde_eventos;
                        fase_interrumpida = phase;
                        DEBUG_LINK.printf("[Borde] #%u en fase %s -- QTR der dif=%d, izq dif=%d (%s). Parando.\n",
                                           (unsigned)borde_eventos, Mission::PhaseName(phase),
                                           (int)last_reflect.right_restado, (int)last_reflect.left_restado,
                                           Mission::kQtrIzquierdoActivo ? "izq activo" : "izq desactivado");
                        phase = Mission::Phase::BORDE_PARAR;
                        phase_started_ms = millis();
                        borde_desde_ms = 0;
                    }
                } else {
                    borde_desde_ms = 0;
                }
            }

            // --- Cerrojos: detenerse YA, para siempre, al ver el color -----
            if (!zona_amarilla_detectada && phase == Mission::Phase::BUSCAR_ZONA_AMARILLA &&
                color_activo == ColorLabel::YELLOW) {
                zona_amarilla_detectada = true;
                phase = Mission::Phase::DETENER_ZONA_AMARILLA;
                phase_started_ms = millis();
            }
            if (!zona_enemiga_detectada && phase == Mission::Phase::AVANZAR_ZONA_ENEMIGA &&
                color_activo == enemy_color) {
                zona_enemiga_detectada = true;
                phase = Mission::Phase::DETENER_ZONA_ENEMIGA;
                phase_started_ms = millis();
            }

            if (phase == Mission::Phase::AVANZAR_ZONA_ENEMIGA &&
                (uint32_t)(millis() - last_color_debug_ms) > 300) {
                last_color_debug_ms = millis();
                DEBUG_LINK.printf("[Mission] buscando=%s  color=%s  valido=%d\n",
                                   ColorLabelName(enemy_color), ColorLabelName(color_activo),
                                   last_color.valid);
            }

            if (phase != last_logged_phase) {
                DEBUG_LINK.printf("[Mission] fase -> %s (color=%s, tof=%u mm, valido=%d)\n",
                                   Mission::PhaseName(phase), ColorLabelName(color_activo),
                                   last_tof.distance_mm, last_tof.valid);
                last_logged_phase = phase;
            }

            switch (phase) {

                case Mission::Phase::ARRANQUE: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kStartupDelayMs) {
                        if (Mission::kBancoSoloPaso7) {
                            DEBUG_LINK.println("[Mission] BANCO SOLO PASO 7: se saltan los pasos 1-6.");
                            phase = Mission::Phase::BUSCAR_BANDERA;
                            busq_paso = 0;
                            busq_paso_desde_ms = millis();
                        } else {
                            phase = Mission::Phase::ASEGURAR_CAJA;
                        }
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::ASEGURAR_CAJA: {
                    gripper.action = GripperAction::CLOSE_LLAVE;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleCajaMs) {
                        phase = Mission::Phase::ESPERAR_ANTES_DE_AVANZAR;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::ESPERAR_ANTES_DE_AVANZAR: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kDelayTrasAsegurarCajaMs) {
                        phase = Mission::Phase::BUSCAR_ZONA_AMARILLA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Sin QTR -- avanza recto sin más hasta que el cerrojo de
                // arriba detecte amarillo.
                case Mission::Phase::BUSCAR_ZONA_AMARILLA: {
                    SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    break;
                }

                // Full stop antes de soltar -- no se abre la pinza mientras
                // el robot todavía se desliza por inercia.
                case Mission::Phase::DETENER_ZONA_AMARILLA: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kFullStopAntesDeSoltarCajaMs) {
                        phase = Mission::Phase::DEPOSITAR_CAJA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::DEPOSITAR_CAJA: {
                    gripper.action = GripperAction::OPEN;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleAperturaCajaMs) {
                        phase = Mission::Phase::RETROCEDER_TRAS_CAJA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Retrocede un poco antes de girar -- para que el arco que
                // barren las ruedas al pivotear no arrastre/empuje la caja
                // recién soltada. Ver el aviso grande al principio del
                // archivo (duración/velocidad sin medir en banco todavía).
                case Mission::Phase::RETROCEDER_TRAS_CAJA: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kRetrocesoTrasCajaMs) {
                        SetDrive(motor, -Mission::kVelocidadRetrocesoTrasCaja, -Mission::kVelocidadRetrocesoTrasCaja);
                    } else {
                        phase = Mission::Phase::ESQUIVAR_CAJA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Gira sobre su propio eje para alejarse de la caja recién
                // soltada, en vez de seguir de largo por encima de ella --
                // ver "GIRO DE ESQUIVE Y RECENTRADO TRAS LA CAJA" al
                // principio del archivo. Mismo primitivo de giro que
                // Phase::GIRAR en v5-agarrar-bandera (un lado ADELANTE, el
                // otro ATRAS).
                case Mission::Phase::ESQUIVAR_CAJA: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kDuracionGiroEsquiveMs) {
                        const int v = Mission::kVelocidadGiroEsquive;
                        if (Mission::kGiroEsquiveHaciaDerecha) {
                            SetDrive(motor, v, -v);
                        } else {
                            SetDrive(motor, -v, v);
                        }
                    } else {
                        phase = Mission::Phase::AVANZAR_TRAS_ESQUIVE;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Avanza un poco tras esquivar, para terminar de salir de
                // la huella de la zona amarilla antes de girar de nuevo --
                // sin esto, el giro de recentrado (más grande) podía volver
                // a pasar sobre la caja.
                case Mission::Phase::AVANZAR_TRAS_ESQUIVE: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kAvanceTrasEsquiveMs) {
                        SetDrive(motor, Mission::kVelocidadAvanceTrasEsquive, Mission::kVelocidadAvanceTrasEsquive);
                    } else {
                        phase = Mission::Phase::GIRO_RECENTRAR;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Segundo giro: reapuntar hacia donde va a estar la
                // bandera, tras haberse desviado con el giro de esquive --
                // mismo primitivo, sentido por defecto contrario al de
                // esquive (ver Mission::kGiroRecentrarHaciaDerecha).
                //
                // GUIADO POR CÁMARA (2026-09-12): a diferencia de
                // ESQUIVAR_CAJA/AVANZAR_TRAS_ESQUIVE (siempre ciegos, a
                // tiempo fijo, sin importar lo que diga CAM_LINK), acá SÍ
                // se corta el giro en cuanto CamaraBandera::Confirmada() es
                // true -- la Pi ya vio la bandera contraria de forma
                // sostenida, así que seguir girando a ciegas solo puede
                // pasarse de largo. Si nunca confirma, kDuracionGiroRecen-
                // trarMs sigue siendo el tope de seguridad de siempre.
                case Mission::Phase::GIRO_RECENTRAR: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    const bool tope_alcanzado =
                        (uint32_t)(millis() - phase_started_ms) >= Mission::kDuracionGiroRecentrarMs;
                    if (CamaraBandera::Confirmada() || tope_alcanzado) {
                        if (CamaraBandera::Confirmada()) {
                            DEBUG_LINK.println("[Mission] giro 2 terminado: la camara confirmo la bandera -- a centrar.");
                            phase = Mission::Phase::CENTRAR_Y_AVANZAR;
                            centr_sub = 0;
                            centr_sub_desde_ms = millis();
                            perdida_desde_ms = 0;
                        } else {
                            DEBUG_LINK.println("[Mission] giro 2 terminado por tope de tiempo, sin ver la bandera -- a buscar.");
                            phase = Mission::Phase::BUSCAR_BANDERA;
                            busq_paso = 0;
                            busq_paso_desde_ms = millis();
                        }
                        phase_started_ms = millis();
                    } else {
                        const int v = Mission::kVelocidadGiroEsquive;
                        if (Mission::kGiroRecentrarHaciaDerecha) {
                            SetDrive(motor, v, -v);
                        } else {
                            SetDrive(motor, -v, v);
                        }
                    }
                    break;
                }

                // Robot quieto (motor en STOP por defecto): tiempo para que
                // una persona ponga la bandera al frente, a la vista del
                // ToF -- la caja ya quedó fuera del camino por los giros de
                // las fases anteriores.
                case Mission::Phase::ESPERAR_REACOMODO: {
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kEsperaReacomodoMs) {
                        phase = Mission::Phase::AVANCE_GRUESO_BANDERA;
                        phase_started_ms = millis();
                        lecturas_en_rango_seguidas = 0;
                        pasos_dados = 0;
                    }
                    break;
                }

                // ---- A partir de acá: MISMO algoritmo que v5-agarrar-bandera ----

                case Mission::Phase::AVANCE_GRUESO_BANDERA: {
                    estado_led = EstadoVisible::BUSCANDO_BANDERA;
                    if (last_tof.valid && last_tof.distance_mm <= Mission::kDistanciaAproximacionMm) {
                        phase = Mission::Phase::DETENER_PARA_MEDIR;
                        phase_started_ms = millis();
                        lecturas_en_rango_seguidas = 0;
                        pasos_dados = 0;
                        break;
                    }
                    SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    break;
                }

                case Mission::Phase::DETENER_PARA_MEDIR: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kSettleTrasParoMs) {
                        break;   // sigue quieto, todavía asentando
                    }

                    // Cámara OBLIGATORIA: sin verla no se mide ni se cierra. Si la
                    // pierde más de kPerdidaBanderaMs, vuelve a buscar.
                    if (!CamaraBandera::Fresca()) {
                        lecturas_en_rango_seguidas = 0;
                        if (perdida_desde_ms == 0) perdida_desde_ms = millis();
                        if ((uint32_t)(millis() - perdida_desde_ms) >= Mission::kPerdidaBanderaMs) {
                            DEBUG_LINK.println("[Mission] ajuste: la camara perdio la bandera -- a buscar.");
                            phase = Mission::Phase::BUSCAR_BANDERA;
                            phase_started_ms = millis();
                            busq_paso = 0;
                            busq_paso_desde_ms = millis();
                        }
                        break;
                    }
                    perdida_desde_ms = 0;

                    // Tope de correcciones: la cámara AÚN la ve, así que (pedido de
                    // Montse) cierra igual aunque el ToF no haya confirmado el rango.
                    if (pasos_dados >= Mission::kMaxPasosSeguridad) {
                        DEBUG_LINK.printf("[Mission] ajuste: %d correcciones sin confirmar el rango, la camara aun la ve -- cierra igual.\n",
                                           pasos_dados);
                        phase = Mission::Phase::CERRAR_GRIPPER_BANDERA;
                        phase_started_ms = millis();
                        break;
                    }

                    // El ToF solo cuenta con la bandera CENTRADA (mide un punto).
                    const int e = CamaraBandera::Error();
                    if (abs(e) > Mission::kZonaMuertaCentrado) {
                        lecturas_en_rango_seguidas = 0;
                        ++pasos_dados;
                        centr_sentido = (e > 0) ? 1 : -1;
                        DEBUG_LINK.printf("[Mission] ajuste: bandera descentrada (err=%d) -- pulso de centrado (correccion %d/%d).\n",
                                           e, pasos_dados, Mission::kMaxPasosSeguridad);
                        phase = Mission::Phase::CENTRAR_FINO;
                        phase_started_ms = millis();
                        break;
                    }

                    const bool en_rango = last_tof.valid &&
                        last_tof.distance_mm >= Mission::kRangoAgarreMinMm &&
                        last_tof.distance_mm <= Mission::kRangoAgarreMaxMm;

                    if (en_rango) {
                        ++lecturas_en_rango_seguidas;
                        DEBUG_LINK.printf("[Mission] en rango (%u mm), %d/%d lecturas seguidas (camara err=%d area=%d%%)\n",
                                           last_tof.distance_mm, lecturas_en_rango_seguidas,
                                           Mission::kLecturasConsecutivasRequeridas, e, CamaraBandera::Area());
                        if (lecturas_en_rango_seguidas >= Mission::kLecturasConsecutivasRequeridas) {
                            phase = Mission::Phase::CERRAR_GRIPPER_BANDERA;
                            phase_started_ms = millis();
                        } else {
                            phase_started_ms = millis();
                        }
                        break;
                    }

                    lecturas_en_rango_seguidas = 0;
                    sentido_paso = (last_tof.valid && last_tof.distance_mm < Mission::kRangoAgarreMinMm)
                        ? -1
                        : 1;
                    phase = Mission::Phase::PASO_AJUSTE;
                    phase_started_ms = millis();
                    break;
                }

                // Un pulso corto de giro para recentrar la bandera antes de volver a medir.
                case Mission::Phase::CENTRAR_FINO: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kPulsoCentradoMs) {
                        const int v = Mission::kVelocidadCentradoMin;
                        if (centr_sentido > 0) SetDrive(motor, v, -v); else SetDrive(motor, -v, v);
                    } else {
                        phase = Mission::Phase::DETENER_PARA_MEDIR;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::PASO_AJUSTE: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    if ((uint32_t)(millis() - phase_started_ms) >= Mission::kPasoDuracionMs) {
                        ++pasos_dados;
                        phase = Mission::Phase::DETENER_PARA_MEDIR;
                        phase_started_ms = millis();
                        break;
                    }
                    const int v = Mission::kVelocidadPaso * sentido_paso;
                    SetDrive(motor, v, v);
                    break;
                }

                case Mission::Phase::CERRAR_GRIPPER_BANDERA: {
                    estado_led = EstadoVisible::AJUSTANDO;
                    gripper.action = GripperAction::CLOSE_BANDERA;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleBanderaMs) {
                        if (Mission::kBancoPararTrasAgarrar) {
                            DEBUG_LINK.println("[Mission] bandera agarrada -- BANCO: el robot se detiene (FIN_M3).");
                            phase = Mission::Phase::FIN_M3;
                        } else {
                            DEBUG_LINK.println("[Mission] bandera agarrada -- a volver a la zona propia.");
                            phase = Mission::Phase::ESPERAR_TRAS_AGARRE;
                        }
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Sin giro -- pedido explícito. Robot quieto (STOP por
                // defecto), respiro corto y sigue derecho.
                case Mission::Phase::ESPERAR_TRAS_AGARRE: {
                    estado_led = EstadoVisible::AGARRADA;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kEsperaTrasAgarrarBanderaMs) {
                        phase = Mission::Phase::GIRO_RETORNO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Sin QTR -- avanza recto hasta que el cerrojo de arriba
                // detecte el color de la zona enemiga.
                case Mission::Phase::AVANZAR_ZONA_ENEMIGA: {
                    estado_led = EstadoVisible::COLOR;
                    SetDrive(motor, Mission::kVelocidadCrucero, Mission::kVelocidadCrucero);
                    break;
                }

                case Mission::Phase::DETENER_ZONA_ENEMIGA: {
                    estado_led = EstadoVisible::AGARRADA;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kFullStopZonaEnemigaMs) {
                        phase = Mission::Phase::SOLTAR_BANDERA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::SOLTAR_BANDERA: {
                    estado_led = EstadoVisible::AGARRADA;
                    gripper.action = GripperAction::OPEN;
                    send_gripper = true;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kGripperSettleSueltaBanderaMs) {
                        phase = Mission::Phase::TERMINADO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // ---- M1: reacción al borde negro -----------------------------------
                case Mission::Phase::BORDE_PARAR: {
                    estado_led = EstadoVisible::BORDE;
                    if ((uint32_t)(millis() - phase_started_ms) >= Mission::kBordeParadaMs) {
                        phase = Mission::Phase::BORDE_RETROCEDER;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::BORDE_RETROCEDER: {
                    estado_led = EstadoVisible::BORDE;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kBordeRetrocesoMs) {
                        SetDrive(motor, -Mission::kBordeVelocidadRetroceso, -Mission::kBordeVelocidadRetroceso);
                    } else {
                        phase = Mission::Phase::BORDE_GIRAR;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::BORDE_GIRAR: {
                    estado_led = EstadoVisible::BORDE;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kBordeGiroMs) {
                        const int v = Mission::kBordeVelocidadGiro;
                        if (Mission::kBordeGiroHaciaDerecha) {
                            SetDrive(motor, v, -v);
                        } else {
                            SetDrive(motor, -v, v);
                        }
                    } else {
                        DEBUG_LINK.printf("[Borde] reaccion terminada -- reanuda %s (cronometro reiniciado)\n",
                                           Mission::PhaseName(fase_interrumpida));
                        phase = fase_interrumpida;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // ---- M2: paso 7 -- buscar la bandera y centrarse ------------------
                case Mission::Phase::BUSCAR_BANDERA: {
                    estado_led = EstadoVisible::BUSCANDO_BANDERA;
                    if (CamaraBandera::Confirmada()) {
                        DEBUG_LINK.printf("[Mission] bandera vista durante la busqueda (err=%d, area=%d%%) -- a centrar.\n",
                                           CamaraBandera::Error(), CamaraBandera::Area());
                        phase = Mission::Phase::CENTRAR_Y_AVANZAR;
                        phase_started_ms = millis();
                        centr_sub = 0;
                        centr_sub_desde_ms = millis();
                        perdida_desde_ms = 0;
                        break;
                    }
                    const uint32_t t = (uint32_t)(millis() - busq_paso_desde_ms);
                    const int vp = Mission::kBusquedaVelocidadPivote;
                    bool paso_termino = false;
                    switch (busq_paso) {
                        case 1:
                        case 5:   // pivote A
                            if (Mission::kBusquedaPrimerPivoteHaciaDerecha) SetDrive(motor, vp, -vp); else SetDrive(motor, -vp, vp);
                            paso_termino = t >= Mission::kBusquedaPivoteMs;
                            break;
                        case 3:   // pivote B (lado contrario, el doble)
                            if (Mission::kBusquedaPrimerPivoteHaciaDerecha) SetDrive(motor, -vp, vp); else SetDrive(motor, vp, -vp);
                            paso_termino = t >= 2 * Mission::kBusquedaPivoteMs;
                            break;
                        case 7:   // avance corto
                            SetDrive(motor, Mission::kBusquedaVelocidadAvance, Mission::kBusquedaVelocidadAvance);
                            paso_termino = t >= Mission::kBusquedaAvanceMs;
                            break;
                        default:  // 0, 2, 4, 6: pausa quieto
                            paso_termino = t >= Mission::kBusquedaPausaMs;
                            break;
                    }
                    if (paso_termino) {
                        busq_paso = (busq_paso + 1) % 8;
                        busq_paso_desde_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::CENTRAR_Y_AVANZAR: {
                    estado_led = EstadoVisible::BUSCANDO_BANDERA;
                    // 1) ¿llegó? La cámara la ve Y el ToF dice que está cerca.
                    if (CamaraBandera::Fresca() && last_tof.valid &&
                        last_tof.distance_mm <= Mission::kDistanciaAproximacionMm) {
                        DEBUG_LINK.printf("[Mission] cerca de la bandera: tof=%u mm, camara err=%d area=%d%% -- ajuste fino con el ToF.\n",
                                           last_tof.distance_mm, CamaraBandera::Error(), CamaraBandera::Area());
                        phase = Mission::Phase::DETENER_PARA_MEDIR;
                        phase_started_ms = millis();
                        lecturas_en_rango_seguidas = 0;
                        pasos_dados = 0;
                        perdida_desde_ms = 0;
                        break;
                    }
                    // 2) ¿perdió la señal? Quieto un momento y de vuelta a buscar.
                    if (!CamaraBandera::Fresca()) {
                        if (perdida_desde_ms == 0) perdida_desde_ms = millis();
                        if ((uint32_t)(millis() - perdida_desde_ms) >= Mission::kPerdidaBanderaMs) {
                            DEBUG_LINK.println("[Mission] la camara perdio la bandera -- a buscar.");
                            phase = Mission::Phase::BUSCAR_BANDERA;
                            phase_started_ms = millis();
                            busq_paso = 0;
                            busq_paso_desde_ms = millis();
                        }
                        break;   // motor en STOP mientras espera
                    }
                    perdida_desde_ms = 0;
                    // 3) Centrado proporcional por pulsos.
                    const int e = CamaraBandera::Error();
                    if (abs(e) <= Mission::kZonaMuertaCentrado) {
                        SetDrive(motor, Mission::kVelocidadAcercamiento, Mission::kVelocidadAcercamiento);
                        centr_sub = 0;
                        centr_sub_desde_ms = millis();   // si se descentra, espera un asentamiento antes del primer pulso
                    } else if (centr_sub == 0) {
                        if ((uint32_t)(millis() - centr_sub_desde_ms) >= Mission::kAsentarCentradoMs) {
                            centr_sub = 1;
                            centr_sub_desde_ms = millis();
                            centr_sentido = (e > 0) ? 1 : -1;
                        }
                    } else {
                        if ((uint32_t)(millis() - centr_sub_desde_ms) < Mission::kPulsoCentradoMs) {
                            const float frac = constrain((float)(abs(e) - Mission::kZonaMuertaCentrado) /
                                                          (float)(100 - Mission::kZonaMuertaCentrado), 0.0f, 1.0f);
                            const int v = Mission::kVelocidadCentradoMin +
                                          (int)((Mission::kVelocidadCentradoMax - Mission::kVelocidadCentradoMin) * frac);
                            if (centr_sentido > 0) SetDrive(motor, v, -v); else SetDrive(motor, -v, v);
                        } else {
                            centr_sub = 0;
                            centr_sub_desde_ms = millis();
                        }
                    }
                    break;
                }

                // ---- M5: pasos 10-12, retorno con la bandera -------------------------
                case Mission::Phase::GIRO_RETORNO: {
                    estado_led = EstadoVisible::AGARRADA;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kGiroRetornoMs) {
                        const int v = Mission::kVelocidadGiroRetorno;
                        if (Mission::kGiroRetornoHaciaDerecha) SetDrive(motor, v, -v); else SetDrive(motor, -v, v);
                    } else {
                        DEBUG_LINK.println("[Mission] vuelta terminada -- a salir de la zona rival.");
                        phase = Mission::Phase::SALIR_ZONA_RIVAL;
                        phase_started_ms = millis();
                    }
                    break;
                }

                // Sin sensor trasero: la salida se confirma al volver a leer la franja RIVAL.
                case Mission::Phase::SALIR_ZONA_RIVAL: {
                    estado_led = EstadoVisible::AGARRADA;
                    if (color_activo == enemy_color) {
                        DEBUG_LINK.println("[Mission] franja rival leida: salio de la zona rival -- a volver.");
                        phase = Mission::Phase::VOLVER_ZONA_PROPIA;
                        phase_started_ms = millis();
                        break;
                    }
                    if ((uint32_t)(millis() - phase_started_ms) >= Mission::kSalirZonaRivalTopeMs) {
                        DEBUG_LINK.println("[Mission] AVISO: no leyo la franja rival en el tiempo tope -- asume que salio y sigue.");
                        phase = Mission::Phase::VOLVER_ZONA_PROPIA;
                        phase_started_ms = millis();
                        break;
                    }
                    SetDrive(motor, Mission::kVelocidadRetorno, Mission::kVelocidadRetorno);
                    break;
                }

                case Mission::Phase::VOLVER_ZONA_PROPIA: {
                    estado_led = EstadoVisible::AGARRADA;
                    if (color_activo == own_color) {
                        DEBUG_LINK.printf("[Mission] franja propia leida -- sigue %u ms y para.\n",
                                           (unsigned)Mission::kAvanceTrasLeerZonaPropiaMs);
                        phase = Mission::Phase::AVANCE_EN_ZONA_PROPIA;
                        phase_started_ms = millis();
                        break;
                    }
                    if (Mission::kRetornoEvitaAmarillo && color_activo == ColorLabel::YELLOW) {
                        DEBUG_LINK.println("[Mission] leyo AMARILLO al volver (zona neutra con la caja) -- se aparta.");
                        phase = Mission::Phase::EVITAR_AMARILLO;
                        phase_started_ms = millis();
                        break;
                    }
                    if ((uint32_t)(millis() - phase_started_ms) >= Mission::kVolverTopeMs) {
                        DEBUG_LINK.println("[Mission] AVISO: no leyo su franja en el tiempo tope -- para y suelta la bandera aqui.");
                        phase = Mission::Phase::DETENER_ZONA_PROPIA;
                        phase_started_ms = millis();
                        break;
                    }
                    SetDrive(motor, Mission::kVelocidadRetorno, Mission::kVelocidadRetorno);
                    break;
                }

                // Se aparta del amarillo: para, retrocede corto, pivota y reanuda la vuelta.
                case Mission::Phase::EVITAR_AMARILLO: {
                    estado_led = EstadoVisible::AGARRADA;
                    const uint32_t t = (uint32_t)(millis() - phase_started_ms);
                    const uint32_t t_retro = Mission::kBordeParadaMs;
                    const uint32_t t_giro  = t_retro + Mission::kEvitarAmarilloRetrocesoMs;
                    const uint32_t t_fin   = t_giro + Mission::kEvitarAmarilloGiroMs;
                    if (t < t_retro) {
                        // quieto
                    } else if (t < t_giro) {
                        SetDrive(motor, -Mission::kBordeVelocidadRetroceso, -Mission::kBordeVelocidadRetroceso);
                    } else if (t < t_fin) {
                        const int v = Mission::kVelocidadGiroRetorno;
                        if (Mission::kEvitarAmarilloHaciaDerecha) SetDrive(motor, v, -v); else SetDrive(motor, -v, v);
                    } else {
                        phase = Mission::Phase::VOLVER_ZONA_PROPIA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::AVANCE_EN_ZONA_PROPIA: {
                    estado_led = EstadoVisible::AGARRADA;
                    if ((uint32_t)(millis() - phase_started_ms) < Mission::kAvanceTrasLeerZonaPropiaMs) {
                        SetDrive(motor, Mission::kVelocidadEntradaZonaPropia, Mission::kVelocidadEntradaZonaPropia);
                    } else {
                        phase = Mission::Phase::DETENER_ZONA_PROPIA;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::DETENER_ZONA_PROPIA: {
                    estado_led = EstadoVisible::AGARRADA;
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kFullStopZonaPropiaMs) {
                        phase = Mission::Phase::SOLTAR_BANDERA_LENTO;
                        phase_started_ms = millis();
                        soltar_lento_enviado = false;
                    }
                    break;
                }

                case Mission::Phase::SOLTAR_BANDERA_LENTO: {
                    estado_led = EstadoVisible::AGARRADA;
                    if (!soltar_lento_enviado) {
                        gripper.action = GripperAction::OPEN_SLOW;
                        send_gripper = true;
                        soltar_lento_enviado = true;
                    }
                    if ((uint32_t)(millis() - phase_started_ms) > Mission::kSoltarLentoEsperaMs) {
                        DEBUG_LINK.println("[Mission] bandera soltada en la zona propia -- MISION COMPLETA.");
                        phase = Mission::Phase::TERMINADO;
                        phase_started_ms = millis();
                    }
                    break;
                }

                case Mission::Phase::FIN_M2:
                    estado_led = EstadoVisible::TERMINADO;
                    break;

                case Mission::Phase::FIN_M3:
                    estado_led = EstadoVisible::AGARRADA;
                    break;

                // M1: pasos 1-6 terminados. Quieto (STOP por defecto) y LED verde.
                case Mission::Phase::FIN_M1:
                    estado_led = EstadoVisible::TERMINADO;
                    break;

                case Mission::Phase::TERMINADO:
                    estado_led = EstadoVisible::TERMINADO;
                    break;

                case Mission::Phase::FALLO_AJUSTE:
                default:
                    estado_led = EstadoVisible::FALLO;
                    break;
            }

        } // if (Mission::kMotionEnabled)

        xQueueOverwrite(g_motorCmdQueue, &motor);
        if (send_gripper) xQueueSend(g_gripperCmdQueue, &gripper, 0);

        LedCommand led;
        led.estado = estado_led;
        led.color  = led_color;
        xQueueOverwrite(g_ledCmdQueue, &led);

        Heartbeat(TaskId::MISSION);
        vTaskDelayUntil(&last_wake, period);
    }
}

// ===========================================================================
//  [11] SUPERVISORTASK, setup() / loop()
// ===========================================================================

namespace {

void SupervisorTask(void *) {
    const TickType_t period = pdMS_TO_TICKS(TaskPeriodMs::SUPERVISOR);
    TickType_t last_wake = xTaskGetTickCount();

    uint8_t previous_faults = 0;

    for (;;) {
        const uint8_t faulted = WatchdogCheck();

        if (faulted != 0) {
            HealthReport report;
            report.timestamp_ms = millis();
            report.faulted_tasks_bitmask = faulted;
            xQueueOverwrite(g_healthQueue, &report);
        }

        if (faulted != previous_faults) {
            DEBUG_LINK.printf("[Supervisor] tareas colgadas: 0x%02X\n", faulted);
            previous_faults = faulted;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace anónimo

static bool CreateQueues() {
    g_motorCmdQueue   = xQueueCreate(1, sizeof(MotorCommand));
    g_gripperCmdQueue = xQueueCreate(4, sizeof(GripperCommand));
    g_ledCmdQueue     = xQueueCreate(1, sizeof(LedCommand));
    g_colorQueue      = xQueueCreate(4, sizeof(ColorReading));
    g_tofQueue        = xQueueCreate(4, sizeof(TofReading));
    g_reflectQueue    = xQueueCreate(4, sizeof(ReflectanceReading));
    g_healthQueue     = xQueueCreate(1, sizeof(HealthReport));

    return g_motorCmdQueue && g_gripperCmdQueue && g_ledCmdQueue &&
           g_colorQueue && g_tofQueue && g_reflectQueue && g_healthQueue;
}

constexpr uint32_t SERIAL_BAUD_RATE = 115200;

static const char *ResetReasonToString(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON (se energizo desde cero)";
        case ESP_RST_EXT:       return "EXTERNO (pin de reset)";
        case ESP_RST_SW:        return "SOFTWARE (esp_restart() o similar)";
        case ESP_RST_PANIC:     return "PANIC (excepcion/crash del firmware)";
        case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";
        case ESP_RST_TASK_WDT:  return "TASK WATCHDOG (una tarea no volvio a tiempo)";
        case ESP_RST_WDT:       return "OTRO WATCHDOG";
        case ESP_RST_DEEPSLEEP: return "DEEP SLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT: el voltaje cayo debajo del minimo -- revisar bateria/alimentacion";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "DESCONOCIDO";
    }
}

// Sobrescrito por el switch de equipo en setup(), ANTES de crear
// MissionTask -- se pasa por puntero (global, no de pila) igual que
// v1-confirmado.
static TeamColor g_myTeam = TeamColor::BLUE;

void setup() {
    DEBUG_LINK.begin(SERIAL_BAUD_RATE);
    CAM_LINK.begin(SERIAL_BAUD_RATE);   // puerto USB nativo -- ver CAM_LINK más arriba

    constexpr uint32_t kSerialWaitMs = 1500;
    const uint32_t wait_start = millis();
    while (!DEBUG_LINK && (millis() - wait_start) < kSerialWaitMs) {
        delay(10);
    }
    delay(200);

    DEBUG_LINK.println("\nAthena Rover 2026 - v8-logica-completa (hito M5: pasos 1-12 completos)");
    DEBUG_LINK.printf("[Setup] Motivo del ultimo reinicio: %s\n",
                       ResetReasonToString(esp_reset_reason()));

    // -- Selector de equipo: switch físico de 3 posiciones -------------------
    pinMode(Pins::TEAM_SWITCH_BLUE, INPUT_PULLUP);
    pinMode(Pins::TEAM_SWITCH_RED,  INPUT_PULLUP);
    delay(5);

    // Bus 1 (PCA9685 + TCS34725 delantero) temprano, gripper a 0 (abierto)
    // mientras se espera el switch -- mismo criterio que los standalones
    // anteriores.
    Wire1.begin(Pins::I2C1_SDA, Pins::I2C1_SCL, 400000);
    Wire1.setTimeOut(25);
    if (Pca9685::Init(Pwm::SERVO_FREQ_HZ)) {
        Pca9685::SetChannel(ServoChannel::CLAW, ServoAngleToTicks(kClawOpenDeg));
        DEBUG_LINK.println("[Setup] Gripper a 0 (abierto) mientras se espera el switch de equipo.");
    } else {
        DEBUG_LINK.println("[Setup] PCA9685 no respondio al intentar abrir el gripper temprano "
                            "-- GripperTask lo reintentara despues de elegir equipo.");
    }

    RgbLed::Setup();
    DEBUG_LINK.println("[Setup] Esperando el switch de equipo (posicion 0 = esperando)...");
    uint32_t blink_ms = millis();
    bool blink_on = false;
    for (;;) {
        const bool blue_closed = digitalRead(Pins::TEAM_SWITCH_BLUE) == LOW;
        const bool red_closed  = digitalRead(Pins::TEAM_SWITCH_RED)  == LOW;
        if (blue_closed && !red_closed) { g_myTeam = TeamColor::BLUE; break; }
        if (red_closed  && !blue_closed) { g_myTeam = TeamColor::RED;  break; }

        if ((uint32_t)(millis() - blink_ms) > 300) {
            blink_ms = millis();
            blink_on = !blink_on;
            RgbLed::SetRaw(blink_on ? 40 : 0, blink_on ? 40 : 0, blink_on ? 40 : 0);
        }
        delay(20);
    }
    RgbLed::SetRaw(0, 0, 0);
    DEBUG_LINK.printf("[Setup] Switch de equipo -> %s (zona enemiga = %s)\n",
                       g_myTeam == TeamColor::RED ? "ROJO" : "AZUL",
                       g_myTeam == TeamColor::RED ? "AZUL" : "ROJO");

    if (!CreateQueues()) {
        DEBUG_LINK.println("[FATAL] no se pudieron crear las colas. Arranque detenido.");
        for (;;) delay(1000);
    }

    g_i2c1Mutex = xSemaphoreCreateMutex();
    if (g_i2c1Mutex == nullptr) {
        DEBUG_LINK.println("[FATAL] no se pudo crear el mutex del bus I2C. Arranque detenido.");
        for (;;) delay(1000);
    }

    WatchdogInit();

    xTaskCreatePinnedToCore(SupervisorTask, "Supervisor", TaskStack::SUPERVISOR,
                            nullptr, TaskPriority::SUPERVISOR, nullptr, 0);
    xTaskCreatePinnedToCore(MissionTask, "Mission", TaskStack::MISSION,
                            &g_myTeam, TaskPriority::MISSION, nullptr, 1);
    xTaskCreatePinnedToCore(MotorTask, "Motors", TaskStack::MOTOR_CONTROL,
                            nullptr, TaskPriority::MOTOR_CONTROL, nullptr, 1);
    xTaskCreatePinnedToCore(GripperTask, "Gripper", TaskStack::GRIPPER_CONTROL,
                            nullptr, TaskPriority::GRIPPER_CONTROL, nullptr, 1);
    xTaskCreatePinnedToCore(ColorSensorTask, "ColorSensor", TaskStack::COLOR_SENSOR,
                            nullptr, TaskPriority::COLOR_SENSOR, nullptr, 0);
    xTaskCreatePinnedToCore(TofSensorTask, "ToF", TaskStack::TOF_SENSOR,
                            nullptr, TaskPriority::TOF_SENSOR, nullptr, 0);
    xTaskCreatePinnedToCore(ReflectanceTask, "Reflectance", TaskStack::REFLECTANCE,
                            nullptr, TaskPriority::REFLECTANCE, nullptr, 0);
    xTaskCreatePinnedToCore(LedTask, "Led", TaskStack::LED_STATUS,
                            nullptr, TaskPriority::LED_STATUS, nullptr, 0);

    DEBUG_LINK.println("Todas las tareas lanzadas. Mision en marcha.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
