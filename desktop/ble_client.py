"""Bleak connection worker isolated from Qt's GUI thread."""

from __future__ import annotations

import asyncio
from dataclasses import replace
import threading
from typing import Any

from PySide6.QtCore import QThread, Signal

from protocol import (
    COMMAND_UUID,
    CONFIG_SIZE,
    CONFIG_UUID,
    DEVICE_NAME,
    SERVICE_UUID,
    TELEMETRY_SIZE,
    TELEMETRY_UUID,
    Command,
    Config,
)


class BleClientThread(QThread):
    connection_changed = Signal(bool, str)
    telemetry_received = Signal(bytes)
    config_received = Signal(bytes)
    error = Signal(str)
    sequence_sent = Signal(int)

    def __init__(self, parent: Any = None) -> None:
        super().__init__(parent)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._actions: asyncio.Queue[tuple[str, Any]] | None = None
        self._ready = threading.Event()

    def run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._actions = asyncio.Queue()
        self._ready.set()
        try:
            self._loop.run_until_complete(self._main())
        finally:
            self._loop.close()
            self._loop = None
            self._actions = None

    def _submit(self, action: str, value: Any = None) -> None:
        if not self.isRunning():
            self.start()
        if not self._ready.wait(timeout=2.0) or self._loop is None or self._actions is None:
            self.error.emit("BLE worker did not start")
            return
        self._loop.call_soon_threadsafe(self._actions.put_nowait, (action, value))

    def connect_dyno(self) -> None:
        self._submit("connect")

    def disconnect_dyno(self) -> None:
        self._submit("disconnect")

    def set_command(self, command: Command) -> None:
        self._submit("command", replace(command))

    def set_config(self, config: Config) -> None:
        self._submit("config", config)

    def stop_worker(self) -> None:
        if self.isRunning():
            self._submit("stop")
            self.wait(5000)

    async def _main(self) -> None:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError:
            self.error.emit("Bleak is not installed. Run: python -m pip install -r requirements.txt")
            return

        client: BleakClient | None = None
        command = Command()
        sequence = 0
        keepalive_task: asyncio.Task[None] | None = None
        stopping = False

        def disconnected(_: Any) -> None:
            self.connection_changed.emit(False, "Dyno disconnected")

        def notification(_: Any, data: bytearray) -> None:
            packet = bytes(data)
            if len(packet) != TELEMETRY_SIZE:
                self.error.emit(
                    f"Telemetry notification was {len(packet)} bytes, expected "
                    f"{TELEMETRY_SIZE}. BLE MTU is too small."
                )
                return
            self.telemetry_received.emit(packet)

        async def keepalive() -> None:
            nonlocal sequence, command
            while client is not None and client.is_connected:
                sequence = (sequence + 1) & 0xFFFFFFFF
                if sequence == 0:
                    sequence = 1
                outgoing = replace(command, sequence=sequence)
                try:
                    await client.write_gatt_char(COMMAND_UUID, outgoing.pack(), response=False)
                    self.sequence_sent.emit(sequence)
                except Exception as exc:  # BLE backend exceptions vary by OS.
                    self.error.emit(f"BLE command write failed: {exc}")
                    return
                await asyncio.sleep(0.1)

        while not stopping:
            action, value = await self._actions.get()
            if action == "command":
                command = value
                continue
            if action == "config":
                if client is None or not client.is_connected:
                    self.error.emit("Connect before sending configuration")
                    continue
                try:
                    await client.write_gatt_char(CONFIG_UUID, value.pack(), response=True)
                    response = b""
                    for _ in range(12):
                        await asyncio.sleep(0.2)
                        response = bytes(await client.read_gatt_char(CONFIG_UUID))
                        if len(response) == CONFIG_SIZE:
                            acknowledged = Config.unpack(response)
                            if acknowledged.sequence == value.sequence:
                                break
                    else:
                        raise RuntimeError("Firmware did not acknowledge the configuration sequence")
                    self.config_received.emit(response)
                except Exception as exc:
                    self.error.emit(f"Configuration update failed: {exc}")
                continue
            if action == "connect":
                if client is not None and client.is_connected:
                    continue
                self.connection_changed.emit(False, "Scanning for DynoController…")
                try:
                    device = await BleakScanner.find_device_by_filter(
                        lambda candidate, advertisement: (
                            candidate.name == DEVICE_NAME
                            or SERVICE_UUID.lower()
                            in [item.lower() for item in advertisement.service_uuids]
                        ),
                        timeout=10.0,
                    )
                    if device is None:
                        raise RuntimeError("DynoController was not found")
                    client = BleakClient(device, disconnected_callback=disconnected)
                    await client.connect()
                    await client.start_notify(TELEMETRY_UUID, notification)
                    config_packet = bytes(await client.read_gatt_char(CONFIG_UUID))
                    if len(config_packet) == CONFIG_SIZE:
                        self.config_received.emit(config_packet)
                    sequence = 0
                    self.connection_changed.emit(True, f"Connected to {device.name or DEVICE_NAME}")
                    keepalive_task = asyncio.create_task(keepalive())
                except Exception as exc:
                    client = None
                    self.connection_changed.emit(False, "Not connected")
                    self.error.emit(f"BLE connection failed: {exc}")
            elif action in {"disconnect", "stop"}:
                if keepalive_task is not None:
                    keepalive_task.cancel()
                    try:
                        await keepalive_task
                    except asyncio.CancelledError:
                        pass
                    keepalive_task = None
                if client is not None:
                    try:
                        if client.is_connected:
                            # Send a fresh disarm before voluntarily disconnecting.
                            sequence = (sequence + 1) & 0xFFFFFFFF or 1
                            safe = replace(command, sequence=sequence, arm=False)
                            await client.write_gatt_char(COMMAND_UUID, safe.pack(), response=False)
                            await asyncio.sleep(0.05)
                            await client.disconnect()
                    except Exception as exc:
                        self.error.emit(f"BLE disconnect warning: {exc}")
                    client = None
                self.connection_changed.emit(False, "Not connected")
                stopping = action == "stop"
