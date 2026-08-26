#pragma once
// watchdog.h — Mecanismo de "heartbeat" cooperativo entre tareas.
//
// Cada tarea llama a Watchdog::ReportHeartbeat(miId) en cada iteración de su
// loop. SupervisorTask llama periódicamente a Watchdog::CheckHeartbeats() y
// detecta qué tareas dejaron de reportar dentro de WATCHDOG_TIMEOUT_MS.
//
// Nota de diseño: cada tarea es la única que escribe su propia posición del
// arreglo, así que no hace falta un mutex para escribir (evita que un
// subsistema lento bloquee a otro solo por reportar su propio heartbeat).
// El supervisor solo lee.

#include "config.h"

namespace Watchdog {

void Init();
void ReportHeartbeat(TaskId id);

// Devuelve un bitmask (bit i = TaskId i) de las tareas cuyo último heartbeat
// superó WATCHDOG_TIMEOUT_MS. No tiene efectos secundarios sobre el estado.
uint8_t CheckHeartbeats();

uint32_t MsSinceLastHeartbeat(TaskId id);

} // namespace Watchdog
