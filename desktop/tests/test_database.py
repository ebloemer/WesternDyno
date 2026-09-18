from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from database import DynoDatabase
from protocol import DynoMode, SystemState, Telemetry, TelemetryFlag


def sample() -> Telemetry:
    return Telemetry(
        state=SystemState.ARMED,
        mode=DynoMode.ENGINE_RPM,
        flags=TelemetryFlag.ARMED,
        fault_flags=0,
        last_command_sequence=1,
        engine_sensor_rpm=1998,
        selected_engine_rpm=2000,
        target_engine_rpm=2000,
        pump_rpm=1200,
        target_pump_rpm=0,
        torque_nm=40,
        target_torque_nm=0,
        flow_valve_duty_pct=50,
        pressure_valve_duty_pct=0,
        throttle_pct=100,
    )


class DatabaseTests(unittest.TestCase):
    def test_database_lifecycle_and_export(self):
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            database = DynoDatabase(temp / "test.sqlite3")
            project = database.create_project("Powertrain A")
            test = database.start_test(project, "Sweep 1", "peak_power", {"configuration": "direct"}, {"duration_s": 10})
            database.append_telemetry(test, 0, 0.0, sample(), stage={"index": 1, "name": "Upshift", "direction": "upshift", "record": True})
            database.append_telemetry(test, 1, 0.05, replace(sample(), torque_nm=41))
            database.finish_test(test, "completed", {"max_power_kw": 5.2})
            output = database.export_test_csv(test, temp / "export.csv")
            text = output.read_text(encoding="utf-8")
            self.assertIn("Sweep 1", text)
            self.assertIn("engine_rpm", text)
            self.assertIn("engine_sensor_rpm", text)
            self.assertIn("sweep_direction", text)
            self.assertIn("upshift", text)
            summary = database.export_summary_csv(test, temp / "summary.csv").read_text(encoding="utf-8")
            self.assertIn("max_power_kw", summary)
            database.close()
