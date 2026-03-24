#pragma once

#include <Arduino.h>
#include <stdint.h>

static const char* DYNO_BLE_DEVICE_NAME = "DynoController";
static const char* DYNO_SERVICE_UUID    = "8d3b7f60-1c9b-4f31-9a56-51d7d6c30001";
static const char* DYNO_CMD_UUID        = "8d3b7f60-1c9b-4f31-9a56-51d7d6c30002";
static const char* DYNO_TEL_UUID        = "8d3b7f60-1c9b-4f31-9a56-51d7d6c30003";

static const uint32_t DYNO_PACKET_MAGIC   = 0x44594E4FUL; // "DYNO"
static const uint8_t  DYNO_PACKET_VERSION = 1;

enum DynoMode : uint8_t {
  MODE_MANUAL = 0,
  MODE_TORQUE = 1,
  MODE_RPM    = 2,
  MODE_CVT    = 3,
};

struct __attribute__((packed)) CommandPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t emergency;
  uint8_t mode;
  uint8_t reserved0;
  uint16_t manualFlowPct;
  uint16_t manualPressurePct;
  float targetTorque;
  float targetRpm;
  float targetEngineRpm;
};

struct __attribute__((packed)) TelemetryPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t emergency;
  uint8_t scaleConnected;
  uint8_t mode;
  float engineRpm;
  float targetEngineRpm;
  float pumpRpm;
  float targetPumpRpm;
  float torque;
  float targetTorque;
  float flowValvePercent;
  float pressureValvePercent;
  float throttlePercent;
  float powerKw;
};

inline void dynoInitCommandPacket(CommandPacket& pkt) {
  pkt.magic = DYNO_PACKET_MAGIC;
  pkt.version = DYNO_PACKET_VERSION;
  pkt.emergency = 0;
  pkt.mode = MODE_MANUAL;
  pkt.reserved0 = 0;
  pkt.manualFlowPct = 0;
  pkt.manualPressurePct = 0;
  pkt.targetTorque = 60.0f;
  pkt.targetRpm = 4200.0f;
  pkt.targetEngineRpm = 3800.0f;
}

inline bool dynoCommandPacketValid(const CommandPacket& pkt) {
  return pkt.magic == DYNO_PACKET_MAGIC && pkt.version == DYNO_PACKET_VERSION;
}

inline bool dynoTelemetryPacketValid(const TelemetryPacket& pkt) {
  return pkt.magic == DYNO_PACKET_MAGIC && pkt.version == DYNO_PACKET_VERSION;
}
