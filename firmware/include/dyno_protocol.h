#pragma once
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static const char* DYNO_BLE_DEVICE_NAME="DynoController";
static const char* DYNO_SERVICE_UUID="8d3b7f60-1c9b-4f31-9a56-51d7d6c30001";
static const char* DYNO_CMD_UUID="8d3b7f60-1c9b-4f31-9a56-51d7d6c30002";
static const char* DYNO_TEL_UUID="8d3b7f60-1c9b-4f31-9a56-51d7d6c30003";
static const char* DYNO_CONFIG_UUID="8d3b7f60-1c9b-4f31-9a56-51d7d6c30004";
static const uint32_t DYNO_PACKET_MAGIC=0x44594E4FUL;
static const uint8_t DYNO_PACKET_VERSION=5;

enum DynoMode:uint8_t{MODE_MANUAL=0,MODE_TORQUE=1,MODE_RPM=2,MODE_ENGINE_RPM=3};
enum DynoSystemState:uint8_t{STATE_BOOT=0,STATE_DISARMED=1,STATE_ARMED=2,STATE_ESTOP=3,STATE_FAULT=4};
enum DynoCommandFlags:uint16_t{CMD_MANUAL_THROTTLE=1U<<0,CMD_CLEAR_LATCH=1U<<1,CMD_PRESSURE_ACTUATOR=1U<<2,CMD_SATURATION_FAULT=1U<<3};
enum DynoTelemetryFlags:uint16_t{TEL_ARMED=1U<<0,TEL_ESTOP=1U<<1,TEL_BLE=1U<<2,TEL_SCALE=1U<<3,TEL_ENGINE_VALID=1U<<4,TEL_PUMP_VALID=1U<<5,TEL_TORQUE_VALID=1U<<6,TEL_SOFT_OVERSPEED=1U<<7,TEL_ANTI_STALL=1U<<8,TEL_SETUP_VALID=1U<<9,TEL_PRESSURE_ACTUATOR=1U<<10};
enum DynoFault:uint32_t{FAULT_NONE=0,FAULT_BLE_DISCONNECTED=1UL<<0,FAULT_COMMAND_TIMEOUT=1UL<<1,FAULT_INVALID_COMMAND=1UL<<2,FAULT_UNSUPPORTED_MODE=1UL<<3,FAULT_PUMP_OVERSPEED=1UL<<4,FAULT_ENGINE_OVERSPEED=1UL<<5,FAULT_PUMP_RPM_SENSOR=1UL<<6,FAULT_ENGINE_RPM_SENSOR=1UL<<7,FAULT_TORQUE_SENSOR=1UL<<8,FAULT_TORQUE_OVERRANGE=1UL<<9,FAULT_SCALE_UNAVAILABLE=1UL<<10,FAULT_TARGET_SATURATION=1UL<<11,FAULT_ANTI_STALL_TIMEOUT=1UL<<12,FAULT_SETUP_REQUIRED=1UL<<13};
enum DynoConfigKind:uint8_t{CONFIG_MACHINE=0,CONFIG_PID=1,CONFIG_ACTIVE_SETUP=2};
enum DynoConfigFlags:uint16_t{CONFIG_APPLY=1U<<0,CONFIG_SAVE_NVS=1U<<1,CONFIG_RESTORE_DEFAULTS=1U<<2,CONFIG_TARE=1U<<3,CONFIG_ENABLED=1U<<4,CONFIG_PUMP_LIMIT=1U<<5,CONFIG_ENGINE_LIMIT=1U<<6,CONFIG_SETUP_CVT=1U<<7,CONFIG_ENGINE_SENSOR=1U<<8};

struct __attribute__((packed)) CommandPacket{uint32_t magic;uint8_t version;uint8_t emergency;uint8_t arm;uint8_t mode;uint32_t sequence;uint16_t manualFlowPct;uint16_t manualPressurePct;uint16_t manualThrottlePct;uint16_t controlFlags;float targetTorqueNm;float targetPumpRpm;float targetEngineRpm;uint32_t crc32;};
struct __attribute__((packed)) TelemetryPacket{uint32_t magic;uint8_t version;uint8_t state;uint8_t mode;uint8_t reserved0;uint16_t flags;uint16_t reserved1;uint32_t faultFlags;uint32_t lastCommandSequence;float engineSensorRpm;float selectedEngineRpm;float targetEngineRpm;float pumpRpm;float targetPumpRpm;float torqueNm;float targetTorqueNm;float flowValveDutyPct;float pressureValveDutyPct;float throttlePct;uint32_t crc32;};
struct __attribute__((packed)) ConfigPacket{uint32_t magic;uint8_t version;uint8_t kind;uint16_t actionFlags;uint32_t sequence;uint8_t index;uint8_t reserved[3];float values[10];uint32_t crc32;};
static_assert(sizeof(CommandPacket)==36,"Command packet must be 36 bytes");static_assert(sizeof(TelemetryPacket)==64,"Telemetry packet must be 64 bytes");static_assert(sizeof(ConfigPacket)==60,"Config packet must be 60 bytes");

inline uint32_t dynoCrc32(const uint8_t*data,size_t length){uint32_t crc=0xffffffffUL;for(size_t i=0;i<length;++i){crc^=data[i];for(uint8_t bit=0;bit<8;++bit)crc=(crc>>1)^(0xedb88320UL&(0UL-(crc&1UL)));}return crc^0xffffffffUL;}
template<typename T>inline void dynoFinalize(T&pkt){pkt.crc32=dynoCrc32(reinterpret_cast<const uint8_t*>(&pkt),offsetof(T,crc32));}
template<typename T>inline bool dynoPacketValid(const T&pkt){return pkt.magic==DYNO_PACKET_MAGIC&&pkt.version==DYNO_PACKET_VERSION&&pkt.crc32==dynoCrc32(reinterpret_cast<const uint8_t*>(&pkt),offsetof(T,crc32));}
