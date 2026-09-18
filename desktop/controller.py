"""Application-level connection, safety command, and telemetry coordinator."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from PySide6.QtCore import QObject, QTimer, Signal

from ble_client import BleClientThread
from protocol import Command, Config, DynoMode, ProtocolError, SystemState, Telemetry
from simulator import DynoSimulator


class DynoController(QObject):
    connection_changed = Signal(bool, str)
    telemetry_changed = Signal(object)
    command_changed = Signal(object)
    config_changed = Signal(object)
    error = Signal(str)

    def __init__(self, simulator: bool = False, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.backend: Any = DynoSimulator(self) if simulator else BleClientThread(self)
        self.backend.connection_changed.connect(self._on_connection)
        self.backend.telemetry_received.connect(self._on_packet)
        self.backend.error.connect(self.error)
        self.backend.config_received.connect(self._on_config)
        self.command = Command()
        self.telemetry: Telemetry | None = None
        self.current_config = Config()
        self.connected = False
        self.config_sequence = 0

    def connect_dyno(self) -> None:
        self.backend.connect_dyno()

    def disconnect_dyno(self) -> None:
        self.disarm()
        self.backend.disconnect_dyno()

    def shutdown(self) -> None:
        if self.connected:
            self.disarm()
        self.backend.stop_worker()

    def update_command(self, **changes: Any) -> None:
        candidate = replace(self.command, **changes)
        try:
            candidate.validate()
        except ProtocolError as exc:
            self.error.emit(str(exc))
            return
        self.command = candidate
        self.backend.set_command(candidate)
        self.command_changed.emit(candidate)

    def arm(self) -> None:
        if not self.connected:
            self.error.emit("Connect to the dyno before arming")
            return
        self.update_command(arm=True, emergency=False)

    def disarm(self) -> None:
        self.update_command(arm=False)

    def emergency_stop(self) -> None:
        self.update_command(arm=False, emergency=True)
        # Emergency is a pulse request, not a level that should be held forever.
        # Clearing the packet bit does not clear the firmware latch; it merely
        # allows a later physical reset to remain in DISARMED.
        QTimer.singleShot(250, lambda: self.update_command(arm=False, emergency=False))

    def clear_latched_fault(self) -> None:
        """Request a safe reset; firmware decides whether physical conditions permit it."""
        self.update_command(arm=False, emergency=False, clear_latch=True)
        QTimer.singleShot(250, lambda: self.update_command(clear_latch=False))

    def report_saturation_fault(self) -> None:
        self.update_command(arm=False, saturation_fault=True)
        QTimer.singleShot(250, lambda: self.update_command(saturation_fault=False))

    def safe_manual(self) -> None:
        self.update_command(
            arm=False,
            emergency=False,
            mode=DynoMode.MANUAL,
            manual_flow_pct=0,
            manual_pressure_pct=0,
            manual_throttle_pct=0,
            manual_throttle_override=False,
            target_torque_nm=0.0,
            target_pump_rpm=0.0,
            target_engine_rpm=0.0,
        )

    def apply_config(self, config: Config) -> None:
        if not self.connected or self.telemetry is None:
            self.error.emit("Connect and receive telemetry before configuring the dyno")
            return
        if self.telemetry.state != SystemState.DISARMED:
            self.error.emit("Configuration is accepted only while the dyno is DISARMED")
            return
        self.config_sequence = (self.config_sequence + 1) & 0xFFFFFFFF or 1
        candidate = replace(config, sequence=self.config_sequence)
        try:
            candidate.validate()
        except ProtocolError as exc:
            self.error.emit(str(exc))
            return
        self.backend.set_config(candidate)

    def _on_connection(self, connected: bool, message: str) -> None:
        self.connected = connected
        if not connected:
            self.command = replace(self.command, arm=False)
        self.connection_changed.emit(connected, message)

    def _on_packet(self, packet: bytes) -> None:
        try:
            self.telemetry = Telemetry.unpack(packet)
        except ProtocolError as exc:
            self.error.emit(str(exc))
            return
        self.telemetry_changed.emit(self.telemetry)

    def _on_config(self, packet: bytes) -> None:
        try:
            config = Config.unpack(packet)
        except ProtocolError as exc:
            self.error.emit(str(exc))
            return
        self.current_config = config
        self.config_changed.emit(config)
