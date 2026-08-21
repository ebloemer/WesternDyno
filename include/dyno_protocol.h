#pragma once

#include <Arduino.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// BLE identity
static const char* DYNO_BLE_DEVICE_NAME = "DynoController";
static const char* DYNO_SERVICE_UUID    = "8d3b7f60-1c9b-4f31-9a56-51d7d6c30001";
static const char* DYNO_CMD_UUID        = "8d3b7f60-1c9b-4f31-9a56-51d7d6c30002";
static const char* DYNO_TEL_UUID        = "8d3b7f60-1c9b-4f31-9a56-51d7d6c30003";

static const uint32_t DYNO_PACKET_MAGIC   = 0x44594E4FUL; // Numeric value for "DYNO"
static const uint8_t  DYNO_PACKET_VERSION = 2;

enum DynoMode : uint8_t {
  MODE_MANUAL = 0,
  MODE_TORQUE = 1,
  MODE_RPM    = 2,
  MODE_CVT    = 3, // Reserved; firmware rejects this mode until implemented.
};

enum DynoSystemState : uint8_t {
  STATE_BOOT       = 0,
  STATE_DISARMED   = 1,
  STATE_ARMED      = 2,
  STATE_ESTOP      = 3,
  STATE_FAULT      = 4,
};

enum DynoTelemetryFlags : uint8_t {
  TEL_FLAG_ARMED           = 1U << 0,
  TEL_FLAG_ESTOP           = 1U << 1,
  TEL_FLAG_BLE_CONNECTED   = 1U << 2,
  TEL_FLAG_SCALE_CONNECTED = 1U << 3,
  TEL_FLAG_ENGINE_RPM_VALID= 1U << 4,
  TEL_FLAG_PUMP_RPM_VALID  = 1U << 5,
  TEL_FLAG_TORQUE_VALID    = 1U << 6,
  TEL_FLAG_SOFT_OVERSPEED  = 1U << 7,
};

enum DynoFault : uint32_t {
  FAULT_NONE                 = 0,
  FAULT_BLE_DISCONNECTED     = 1UL << 0,
  FAULT_COMMAND_TIMEOUT      = 1UL << 1,
  FAULT_INVALID_COMMAND      = 1UL << 2,
  FAULT_UNSUPPORTED_MODE     = 1UL << 3,
  FAULT_PUMP_OVERSPEED       = 1UL << 4,
  FAULT_ENGINE_OVERSPEED     = 1UL << 5,
  FAULT_PUMP_RPM_SENSOR      = 1UL << 6,
  FAULT_ENGINE_RPM_SENSOR    = 1UL << 7,
  FAULT_TORQUE_SENSOR        = 1UL << 8,
  FAULT_TORQUE_OVERRANGE     = 1UL << 9,
  FAULT_SCALE_UNAVAILABLE    = 1UL << 10,
};

// All multibyte fields are little-endian on the ESP32. The future Python
// application must use explicit little-endian packing ("<" with struct.pack).
struct __attribute__((packed)) CommandPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t emergency; // Non-zero latches STATE_ESTOP. Only the physical reset clears it.
  uint8_t arm;       // 0 = disarm, 1 = request arm.
  uint8_t mode;
  uint32_t sequence;
  uint16_t manualFlowPct;
  uint16_t manualPressurePct; // 0 = minimum braking, 100 = maximum braking.
  uint16_t manualThrottlePct;
  uint16_t reserved0;
  float targetTorqueNm;
  float targetPumpRpm;        // Direct pump-shaft target; no sprocket conversion.
  float targetEngineRpm;
  uint32_t crc32;
};

struct __attribute__((packed)) TelemetryPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t state;
  uint8_t mode;
  uint8_t flags;
  uint32_t faultFlags;
  uint32_t lastCommandSequence;
  float engineRpm;
  float targetEngineRpm;
  float pumpRpm;
  float targetPumpRpm;
  float torqueNm;
  float targetTorqueNm;
  float flowValveDutyPct;
  float pressureValveDutyPct; // Raw duty: higher duty produces lower pressure.
  float throttlePct;
  float powerKw;
  uint32_t crc32;
};

static_assert(sizeof(CommandPacket) == 36, "Unexpected CommandPacket packing");
static_assert(sizeof(TelemetryPacket) == 60, "Unexpected TelemetryPacket packing");

inline uint32_t dynoCrc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;

  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
  }

  return crc ^ 0xFFFFFFFFUL;
}

inline void dynoFinalizeCommandPacket(CommandPacket& pkt) {
  pkt.crc32 = dynoCrc32(reinterpret_cast<const uint8_t*>(&pkt),
                        offsetof(CommandPacket, crc32));
}

inline void dynoFinalizeTelemetryPacket(TelemetryPacket& pkt) {
  pkt.crc32 = dynoCrc32(reinterpret_cast<const uint8_t*>(&pkt),
                        offsetof(TelemetryPacket, crc32));
}

inline void dynoInitCommandPacket(CommandPacket& pkt) {
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic = DYNO_PACKET_MAGIC;
  pkt.version = DYNO_PACKET_VERSION;
  pkt.mode = MODE_MANUAL;
  pkt.targetTorqueNm = 60.0f;
  pkt.targetPumpRpm = 2500.0f;
  pkt.targetEngineRpm = 3600.0f;
  dynoFinalizeCommandPacket(pkt);
}

inline bool dynoCommandPacketValid(const CommandPacket& pkt) {
  if (pkt.magic != DYNO_PACKET_MAGIC) return false;
  if (pkt.version != DYNO_PACKET_VERSION) return false;

  const uint32_t expected = dynoCrc32(
      reinterpret_cast<const uint8_t*>(&pkt), offsetof(CommandPacket, crc32));
  return pkt.crc32 == expected;
}

inline bool dynoTelemetryPacketValid(const TelemetryPacket& pkt) {
  if (pkt.magic != DYNO_PACKET_MAGIC) return false;
  if (pkt.version != DYNO_PACKET_VERSION) return false;

  const uint32_t expected = dynoCrc32(
      reinterpret_cast<const uint8_t*>(&pkt), offsetof(TelemetryPacket, crc32));
  return pkt.crc32 == expected;
}
