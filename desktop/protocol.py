"""Binary BLE protocol shared with dyno_protocol.h (protocol version 5)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import math
import struct
import zlib


DEVICE_NAME = "DynoController"
SERVICE_UUID = "8d3b7f60-1c9b-4f31-9a56-51d7d6c30001"
COMMAND_UUID = "8d3b7f60-1c9b-4f31-9a56-51d7d6c30002"
TELEMETRY_UUID = "8d3b7f60-1c9b-4f31-9a56-51d7d6c30003"
CONFIG_UUID = "8d3b7f60-1c9b-4f31-9a56-51d7d6c30004"

PACKET_MAGIC = 0x44594E4F
PACKET_VERSION = 5
COMMAND_FORMAT_NO_CRC = "<IBBBBIHHHHfff"
COMMAND_FORMAT = COMMAND_FORMAT_NO_CRC + "I"
TELEMETRY_FORMAT_NO_CRC = "<IBBBBHHIIffffffffff"
TELEMETRY_FORMAT = TELEMETRY_FORMAT_NO_CRC + "I"
CONFIG_FORMAT_NO_CRC = "<IBBHIB3xffffffffff"
CONFIG_FORMAT = CONFIG_FORMAT_NO_CRC + "I"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)
CONFIG_SIZE = struct.calcsize(CONFIG_FORMAT)


class ProtocolError(ValueError):
    """Raised when a packet is malformed or incompatible."""


class DynoMode(IntEnum):
    MANUAL = 0
    TORQUE = 1
    RPM = 2
    ENGINE_RPM = 3


class SystemState(IntEnum):
    BOOT = 0
    DISARMED = 1
    ARMED = 2
    ESTOP = 3
    FAULT = 4


class CommandFlag(IntFlag):
    MANUAL_THROTTLE = 1 << 0
    CLEAR_LATCH = 1 << 1
    PRESSURE_ACTUATOR = 1 << 2
    SATURATION_FAULT = 1 << 3


class TelemetryFlag(IntFlag):
    ARMED = 1 << 0
    ESTOP = 1 << 1
    BLE_CONNECTED = 1 << 2
    SCALE_CONNECTED = 1 << 3
    ENGINE_RPM_VALID = 1 << 4
    PUMP_RPM_VALID = 1 << 5
    TORQUE_VALID = 1 << 6
    SOFT_OVERSPEED = 1 << 7
    ANTI_STALL = 1 << 8
    ACTIVE_SETUP_VALID = 1 << 9
    PRESSURE_ACTUATOR = 1 << 10


class Fault(IntFlag):
    NONE = 0
    BLE_DISCONNECTED = 1 << 0
    COMMAND_TIMEOUT = 1 << 1
    INVALID_COMMAND = 1 << 2
    UNSUPPORTED_MODE = 1 << 3
    PUMP_OVERSPEED = 1 << 4
    ENGINE_OVERSPEED = 1 << 5
    PUMP_RPM_SENSOR = 1 << 6
    ENGINE_RPM_SENSOR = 1 << 7
    TORQUE_SENSOR = 1 << 8
    TORQUE_OVERRANGE = 1 << 9
    SCALE_UNAVAILABLE = 1 << 10
    TARGET_SATURATION = 1 << 11
    ANTI_STALL_TIMEOUT = 1 << 12
    SETUP_REQUIRED = 1 << 13


class ConfigKind(IntEnum):
    MACHINE = 0
    PID = 1
    ACTIVE_SETUP = 2


class ConfigFlag(IntFlag):
    APPLY = 1 << 0
    SAVE_TO_NVS = 1 << 1
    RESTORE_DEFAULTS = 1 << 2
    TARE_LOAD_CELL = 1 << 3
    ENABLED = 1 << 4
    PUMP_LIMIT_ENABLED = 1 << 5
    ENGINE_LIMIT_ENABLED = 1 << 6
    SETUP_CVT = 1 << 7
    ENGINE_SENSOR_SOURCE = 1 << 8


FAULT_LABELS = {
    Fault.BLE_DISCONNECTED: "BLE disconnected",
    Fault.COMMAND_TIMEOUT: "Command watchdog timed out",
    Fault.INVALID_COMMAND: "Invalid command received",
    Fault.UNSUPPORTED_MODE: "Unsupported control mode",
    Fault.PUMP_OVERSPEED: "Pump overspeed",
    Fault.ENGINE_OVERSPEED: "Engine overspeed",
    Fault.PUMP_RPM_SENSOR: "Pump RPM sensor invalid",
    Fault.ENGINE_RPM_SENSOR: "Engine RPM sensor invalid",
    Fault.TORQUE_SENSOR: "Torque signal invalid",
    Fault.TORQUE_OVERRANGE: "Torque overrange",
    Fault.SCALE_UNAVAILABLE: "Load-cell interface unavailable",
    Fault.TARGET_SATURATION: "Target remained unreachable while output was saturated",
    Fault.ANTI_STALL_TIMEOUT: "Engine did not recover from anti-stall intervention",
    Fault.SETUP_REQUIRED: "No acknowledged active setup",
}


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def fault_names(value: int) -> list[str]:
    flags = Fault(value)
    if flags == Fault.NONE:
        return []
    names = [label for bit, label in FAULT_LABELS.items() if flags & bit]
    known = sum(int(bit) for bit in FAULT_LABELS)
    unknown = value & ~known
    if unknown:
        names.append(f"Unknown fault bits 0x{unknown:08X}")
    return names


@dataclass(slots=True)
class Command:
    sequence: int = 0
    emergency: bool = False
    arm: bool = False
    mode: DynoMode = DynoMode.MANUAL
    manual_flow_pct: int = 0
    manual_pressure_pct: int = 0
    manual_throttle_pct: int = 0
    manual_throttle_override: bool = False
    clear_latch: bool = False
    pressure_actuator: bool = False
    saturation_fault: bool = False
    target_torque_nm: float = 0.0
    target_pump_rpm: float = 0.0
    target_engine_rpm: float = 0.0

    def validate(self) -> None:
        for name, value in (
            ("manual_flow_pct", self.manual_flow_pct),
            ("manual_pressure_pct", self.manual_pressure_pct),
            ("manual_throttle_pct", self.manual_throttle_pct),
        ):
            if not 0 <= int(value) <= 100:
                raise ProtocolError(f"{name} must be between 0 and 100")
        for name, value, maximum in (
            ("target_torque_nm", self.target_torque_nm, 10_000.0),
            ("target_pump_rpm", self.target_pump_rpm, 100_000.0),
            ("target_engine_rpm", self.target_engine_rpm, 100_000.0),
        ):
            if not math.isfinite(value) or not 0.0 <= value <= maximum:
                raise ProtocolError(f"{name} must be finite and between 0 and {maximum:g}")
        if not 0 <= self.sequence <= 0xFFFFFFFF:
            raise ProtocolError("sequence must fit in uint32")

    @property
    def control_flags(self) -> CommandFlag:
        flags = CommandFlag(0)
        if self.manual_throttle_override:
            flags |= CommandFlag.MANUAL_THROTTLE
        if self.clear_latch:
            flags |= CommandFlag.CLEAR_LATCH
        if self.pressure_actuator:
            flags |= CommandFlag.PRESSURE_ACTUATOR
        if self.saturation_fault:
            flags |= CommandFlag.SATURATION_FAULT
        return flags

    def pack(self) -> bytes:
        self.validate()
        body = struct.pack(
            COMMAND_FORMAT_NO_CRC,
            PACKET_MAGIC, PACKET_VERSION, int(self.emergency), int(self.arm), int(self.mode),
            self.sequence, int(self.manual_flow_pct), int(self.manual_pressure_pct),
            int(self.manual_throttle_pct), int(self.control_flags),
            float(self.target_torque_nm), float(self.target_pump_rpm), float(self.target_engine_rpm),
        )
        return body + struct.pack("<I", crc32(body))


@dataclass(frozen=True, slots=True)
class Telemetry:
    state: SystemState
    mode: DynoMode
    flags: TelemetryFlag
    fault_flags: int
    last_command_sequence: int
    engine_sensor_rpm: float
    selected_engine_rpm: float
    target_engine_rpm: float
    pump_rpm: float
    target_pump_rpm: float
    torque_nm: float
    target_torque_nm: float
    flow_valve_duty_pct: float
    pressure_valve_duty_pct: float
    throttle_pct: float

    @classmethod
    def unpack(cls, packet: bytes) -> "Telemetry":
        if len(packet) != TELEMETRY_SIZE:
            raise ProtocolError(f"Telemetry is {len(packet)} bytes; expected {TELEMETRY_SIZE}")
        expected_crc = struct.unpack_from("<I", packet, TELEMETRY_SIZE - 4)[0]
        if expected_crc != crc32(packet[:-4]):
            raise ProtocolError("Telemetry CRC mismatch")
        values = struct.unpack(TELEMETRY_FORMAT, packet)
        if values[0] != PACKET_MAGIC or values[1] != PACKET_VERSION:
            raise ProtocolError("Incompatible telemetry packet")
        floats = values[9:19]
        if not all(math.isfinite(value) for value in floats):
            raise ProtocolError("Telemetry contains a non-finite value")
        try:
            state, mode = SystemState(values[2]), DynoMode(values[3])
        except ValueError as exc:
            raise ProtocolError(f"Unknown state or mode: {exc}") from exc
        return cls(
            state, mode, TelemetryFlag(values[5]), values[7], values[8],
            *floats,
        )

    @property
    def engine_rpm(self) -> float:
        return self.selected_engine_rpm

    @property
    def power_kw(self) -> float:
        return self.torque_nm * self.pump_rpm / 9549.296596

    @property
    def faults(self) -> list[str]:
        return fault_names(self.fault_flags)


@dataclass(frozen=True, slots=True)
class Config:
    sequence: int = 0
    kind: ConfigKind = ConfigKind.MACHINE
    index: int = 0
    action_flags: ConfigFlag = ConfigFlag.APPLY
    values: tuple[float, ...] = (0.0,) * 10

    def validate(self) -> None:
        allowed = ConfigFlag(0)
        for flag in ConfigFlag:
            allowed |= flag
        if self.action_flags & ~allowed:
            raise ProtocolError("Unknown configuration flag")
        if not 0 <= self.index <= 255:
            raise ProtocolError("Configuration index must fit in uint8")
        if len(self.values) != 10 or not all(math.isfinite(float(value)) for value in self.values):
            raise ProtocolError("Configuration must contain ten finite values")
        if self.kind == ConfigKind.PID and not 0 <= self.index < 6:
            raise ProtocolError("PID bank index must be from 0 through 5")

    def pack(self) -> bytes:
        self.validate()
        body = struct.pack(
            CONFIG_FORMAT_NO_CRC,
            PACKET_MAGIC, PACKET_VERSION, int(self.kind), int(self.action_flags),
            self.sequence, self.index, *(float(value) for value in self.values),
        )
        return body + struct.pack("<I", crc32(body))

    @classmethod
    def unpack(cls, packet: bytes) -> "Config":
        if len(packet) != CONFIG_SIZE:
            raise ProtocolError(f"Configuration is {len(packet)} bytes; expected {CONFIG_SIZE}")
        if struct.unpack_from("<I", packet, CONFIG_SIZE - 4)[0] != crc32(packet[:-4]):
            raise ProtocolError("Configuration CRC mismatch")
        values = struct.unpack(CONFIG_FORMAT, packet)
        if values[0] != PACKET_MAGIC or values[1] != PACKET_VERSION:
            raise ProtocolError("Incompatible configuration packet")
        config = cls(
            sequence=values[4], kind=ConfigKind(values[2]), index=values[5],
            action_flags=ConfigFlag(values[3]), values=tuple(values[6:16]),
        )
        config.validate()
        return config

    @classmethod
    def machine(cls, machine, *, save: bool = True, sequence: int = 0) -> "Config":
        flags = ConfigFlag.APPLY | (ConfigFlag.SAVE_TO_NVS if save else ConfigFlag(0))
        if machine.pump_limit_enabled:
            flags |= ConfigFlag.PUMP_LIMIT_ENABLED
        return cls(sequence, ConfigKind.MACHINE, 0, flags, (
            machine.flow_pwm_min_pct, machine.flow_pwm_max_pct,
            machine.pressure_pwm_min_pct, machine.pressure_pwm_max_pct,
            machine.pump_pulses_per_rev, machine.pump_max_rpm,
            machine.load_cell_scale_factor, machine.torque_filter_alpha, 0.0, 0.0,
        ))

    @classmethod
    def pid(cls, index: int, tuning, *, save: bool = True, sequence: int = 0) -> "Config":
        flags = ConfigFlag.APPLY | (ConfigFlag.SAVE_TO_NVS if save else ConfigFlag(0))
        if tuning.enabled:
            flags |= ConfigFlag.ENABLED
        return cls(sequence, ConfigKind.PID, index, flags, (
            tuning.p, tuning.i, tuning.d, tuning.output_slew_pct_s, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
        ))

    @classmethod
    def active_setup(cls, profile, *, sequence: int = 0) -> "Config":
        flags = ConfigFlag.APPLY
        if profile.configuration == "cvt":
            flags |= ConfigFlag.SETUP_CVT
        if profile.uses_engine_sensor:
            flags |= ConfigFlag.ENGINE_SENSOR_SOURCE
        if profile.engine_overspeed_enabled:
            flags |= ConfigFlag.ENGINE_LIMIT_ENABLED
        return cls(sequence, ConfigKind.ACTIVE_SETUP, 0, flags, (
            profile.engine_to_pump_ratio, profile.cvt_output_to_pump_ratio,
            profile.engine_pulses_per_rev, profile.throttle_closed_us,
            profile.throttle_full_us, profile.throttle_frequency_hz,
            profile.engine_overspeed_rpm, profile.stall_warning_rpm,
            profile.recovery_rpm, profile.recovery_timeout_s,
        ))


assert COMMAND_SIZE == 36
assert TELEMETRY_SIZE == 64
assert CONFIG_SIZE == 60
