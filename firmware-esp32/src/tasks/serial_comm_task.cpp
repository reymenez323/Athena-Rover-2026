#include <Arduino.h>
#include "serial_comm_task.h"
#include "config.h"
#include "queues.h"
#include "watchdog.h"

// ---------------------------------------------------------------------------
// Protocolo serial (simple, con resincronización ante bytes corruptos):
//
//   [0xAA][TYPE][LEN][PAYLOAD...][CHECKSUM]
//
//   TYPE    : 1 byte, ver enum PacketType abajo.
//   LEN     : 1 byte, tamaño de PAYLOAD en bytes.
//   CHECKSUM: 1 byte, XOR de TYPE+LEN+PAYLOAD.
//
// Si el checksum no calza o el byte de inicio no aparece donde se espera,
// el parser descarta un byte y sigue buscando 0xAA — un paquete corrupto
// jamás debe trabar el enlace completo.
// ---------------------------------------------------------------------------

namespace {

constexpr uint8_t kStartByte = 0xAA;

enum class PacketType : uint8_t {
    // RPi -> ESP32
    CMD_MOTOR   = 0x01,
    CMD_GRIPPER = 0x02,
    CMD_LED     = 0x03,
    // ESP32 -> RPi
    TLM_COLOR       = 0x10,
    TLM_REFLECTANCE = 0x11,
    TLM_HEALTH      = 0x12,
};

uint8_t Checksum(uint8_t type, uint8_t len, const uint8_t *payload) {
    uint8_t c = type ^ len;
    for (uint8_t i = 0; i < len; ++i) c ^= payload[i];
    return c;
}

void SendPacket(HardwareSerial &port, PacketType type, const uint8_t *payload, uint8_t len) {
    uint8_t buf[3 + 32];
    buf[0] = kStartByte;
    buf[1] = static_cast<uint8_t>(type);
    buf[2] = len;
    memcpy(&buf[3], payload, len);
    buf[3 + len] = Checksum(buf[1], buf[2], payload);
    port.write(buf, 4 + len);
}

// Intenta leer y despachar UN paquete completo desde `port`. No bloquea más
// de lo que ya está disponible en el buffer (timeout corto del propio
// HardwareSerial) para no acaparar el procesador ni retrasar la telemetría
// saliente.
void TryReceivePacket(HardwareSerial &port) {
    if (port.available() < 1) return;
    if (port.peek() != kStartByte) { port.read(); return; } // resync: descartar byte suelto

    // Esperamos a tener el header completo (TYPE+LEN) antes de consumir el start byte.
    if (port.available() < 3) return;

    // Snapshot sin consumir aún, para poder abortar limpio si el payload no llegó completo.
    uint8_t header[3];
    // NOTA: Stream no soporta "peek" múltiple; en la práctica se implementa
    // con un pequeño buffer circular propio. Aquí se deja la intención clara;
    // ver TODO de implementación completa antes de la primera prueba en banco.
    // TODO: implementar buffer circular robusto para tramas fragmentadas.
    port.readBytes(header, 3);
    uint8_t type = header[1];
    uint8_t len  = header[2];
    if (len > 32) return; // paquete inválido, se descarta (resync ocurrirá en el próximo loop)

    uint8_t payload[32] = {0};
    uint32_t start = millis();
    uint8_t received = 0;
    while (received < len && (millis() - start) < SerialLink::RX_TIMEOUT_MS) {
        if (port.available()) payload[received++] = port.read();
    }
    if (received != len) return; // trama incompleta, se descarta

    uint8_t checksum = port.available() ? port.read() : 0;
    if (checksum != Checksum(type, len, payload)) return; // corrupta, se descarta

    switch (static_cast<PacketType>(type)) {
        case PacketType::CMD_MOTOR: {
            if (len == sizeof(MotorCommand)) {
                MotorCommand cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                xQueueOverwrite(g_motorCmdQueue, &cmd); // solo nos interesa el comando más reciente
            }
            break;
        }
        case PacketType::CMD_GRIPPER: {
            if (len == sizeof(GripperCommand)) {
                GripperCommand cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                xQueueSend(g_gripperCmdQueue, &cmd, 0);
            }
            break;
        }
        case PacketType::CMD_LED: {
            if (len == sizeof(LedCommand)) {
                LedCommand cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                xQueueOverwrite(g_ledCmdQueue, &cmd);
            }
            break;
        }
        default: break; // tipo desconocido: se ignora, no se cae el enlace
    }
}

template <typename T>
void DrainTelemetryQueue(HardwareSerial &port, QueueHandle_t queue, PacketType type) {
    T item;
    while (xQueueReceive(queue, &item, 0) == pdTRUE) {
        SendPacket(port, type, reinterpret_cast<uint8_t *>(&item), sizeof(item));
    }
}

void SerialCommTaskFn(void *) {
    HardwareSerial &rpiPort = Serial1; // UART dedicado a la RPi (no el USB de programación)
    rpiPort.begin(SerialLink::BAUD_RATE, SERIAL_8N1, Pins::RPI_UART_RX, Pins::RPI_UART_TX);

    for (;;) {
        TryReceivePacket(rpiPort);

        DrainTelemetryQueue<ColorReading>(rpiPort, g_colorTelemetryQueue, PacketType::TLM_COLOR);
        DrainTelemetryQueue<ReflectanceReading>(rpiPort, g_reflectanceTelemetryQueue, PacketType::TLM_REFLECTANCE);
        DrainTelemetryQueue<HealthReport>(rpiPort, g_healthTelemetryQueue, PacketType::TLM_HEALTH);

        Watchdog::ReportHeartbeat(TaskId::SERIAL_COMM);
        vTaskDelay(pdMS_TO_TICKS(5)); // ceder CPU; este loop es intencionalmente ajustado
    }
}

} // namespace

void SerialCommTask_Start() {
    xTaskCreatePinnedToCore(
        SerialCommTaskFn, "SerialCommTask",
        TaskStack::SERIAL_COMM, nullptr,
        TaskPriority::SERIAL_COMM, nullptr,
        /*core=*/1);
}
