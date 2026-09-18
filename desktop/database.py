"""SQLite project/test storage with transaction-safe telemetry logging."""

from __future__ import annotations

import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
from typing import Any, Iterable

from protocol import Telemetry


SCHEMA_VERSION = 3


class DynoDatabase:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.connection = sqlite3.connect(self.path)
        self.connection.row_factory = sqlite3.Row
        self.connection.execute("PRAGMA foreign_keys = ON")
        self.connection.execute("PRAGMA journal_mode = WAL")
        self._create_schema()

    def _create_schema(self) -> None:
        self.connection.executescript(
            """
            CREATE TABLE IF NOT EXISTS app_metadata (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS projects (
                id INTEGER PRIMARY KEY,
                name TEXT NOT NULL,
                description TEXT NOT NULL DEFAULT '',
                created_utc TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS tests (
                id INTEGER PRIMARY KEY,
                project_id INTEGER REFERENCES projects(id) ON DELETE SET NULL,
                name TEXT NOT NULL,
                test_type TEXT NOT NULL,
                status TEXT NOT NULL,
                started_utc TEXT NOT NULL,
                finished_utc TEXT,
                setup_json TEXT NOT NULL,
                parameters_json TEXT NOT NULL,
                results_json TEXT NOT NULL DEFAULT '{}',
                notes TEXT NOT NULL DEFAULT ''
            );
            CREATE TABLE IF NOT EXISTS telemetry (
                test_id INTEGER NOT NULL REFERENCES tests(id) ON DELETE CASCADE,
                sample_index INTEGER NOT NULL,
                elapsed_s REAL NOT NULL,
                system_state INTEGER NOT NULL,
                mode INTEGER NOT NULL,
                flags INTEGER NOT NULL,
                fault_flags INTEGER NOT NULL,
                command_sequence INTEGER NOT NULL,
                engine_rpm REAL NOT NULL,
                target_engine_rpm REAL NOT NULL,
                pump_rpm REAL NOT NULL,
                target_pump_rpm REAL NOT NULL,
                torque_nm REAL NOT NULL,
                target_torque_nm REAL NOT NULL,
                flow_valve_duty_pct REAL NOT NULL,
                pressure_valve_duty_pct REAL NOT NULL,
                throttle_pct REAL NOT NULL,
                power_kw REAL NOT NULL,
                cvt_ratio REAL,
                estimated_efficiency_pct REAL,
                wheel_rpm REAL,
                stage_index INTEGER NOT NULL DEFAULT 0,
                stage_name TEXT NOT NULL DEFAULT '',
                sweep_direction TEXT NOT NULL DEFAULT '',
                record_enabled INTEGER NOT NULL DEFAULT 1,
                engine_sensor_rpm REAL,
                selected_engine_rpm REAL,
                derived_engine_rpm REAL,
                rpm_source_difference REAL,
                actuator TEXT NOT NULL DEFAULT '',
                target_type TEXT NOT NULL DEFAULT '',
                target_value REAL,
                target_error REAL,
                flow_command_pct REAL,
                pressure_command_pct REAL,
                anti_stall_active INTEGER NOT NULL DEFAULT 0,
                cvt_position_pct REAL,
                PRIMARY KEY (test_id, sample_index)
            );
            CREATE INDEX IF NOT EXISTS telemetry_test_elapsed
            ON telemetry(test_id, elapsed_s);
            """
        )
        # Forward-compatible migration for databases created by early UI builds.
        existing = {
            row[1] for row in self.connection.execute("PRAGMA table_info(telemetry)")
        }
        migrations = {
            "cvt_ratio": "REAL",
            "estimated_efficiency_pct": "REAL",
            "wheel_rpm": "REAL",
            "stage_index": "INTEGER NOT NULL DEFAULT 0",
            "stage_name": "TEXT NOT NULL DEFAULT ''",
            "sweep_direction": "TEXT NOT NULL DEFAULT ''",
            "record_enabled": "INTEGER NOT NULL DEFAULT 1",
            "engine_sensor_rpm": "REAL",
            "selected_engine_rpm": "REAL",
            "derived_engine_rpm": "REAL",
            "rpm_source_difference": "REAL",
            "actuator": "TEXT NOT NULL DEFAULT ''",
            "target_type": "TEXT NOT NULL DEFAULT ''",
            "target_value": "REAL",
            "target_error": "REAL",
            "flow_command_pct": "REAL",
            "pressure_command_pct": "REAL",
            "anti_stall_active": "INTEGER NOT NULL DEFAULT 0",
            "cvt_position_pct": "REAL",
        }
        for column, definition in migrations.items():
            if column not in existing:
                self.connection.execute(f"ALTER TABLE telemetry ADD COLUMN {column} {definition}")
        self.connection.execute(
            "INSERT OR REPLACE INTO app_metadata(key, value) VALUES('schema_version', ?)",
            (str(SCHEMA_VERSION),),
        )
        self.connection.commit()

    def close(self) -> None:
        self.connection.close()

    def create_project(self, name: str, description: str = "") -> int:
        if not name.strip():
            raise ValueError("Project name cannot be empty")
        cursor = self.connection.execute(
            "INSERT INTO projects(name, description, created_utc) VALUES(?, ?, ?)",
            (name.strip(), description.strip(), _utc_now()),
        )
        self.connection.commit()
        return int(cursor.lastrowid)

    def ensure_default_project(self) -> int:
        row = self.connection.execute("SELECT id FROM projects ORDER BY id LIMIT 1").fetchone()
        return int(row[0]) if row else self.create_project("Default Project")

    def start_test(
        self,
        project_id: int,
        name: str,
        test_type: str,
        setup: dict[str, Any],
        parameters: dict[str, Any],
    ) -> int:
        cursor = self.connection.execute(
            """
            INSERT INTO tests(
                project_id, name, test_type, status, started_utc,
                setup_json, parameters_json
            ) VALUES(?, ?, ?, 'running', ?, ?, ?)
            """,
            (
                project_id,
                name,
                test_type,
                _utc_now(),
                json.dumps(setup, sort_keys=True),
                json.dumps(parameters, sort_keys=True),
            ),
        )
        self.connection.commit()
        return int(cursor.lastrowid)

    def append_telemetry(
        self,
        test_id: int,
        sample_index: int,
        elapsed_s: float,
        value: Telemetry,
        calculated: dict[str, float | None] | None = None,
        stage: dict[str, Any] | None = None,
    ) -> None:
        calculated = calculated or {}
        stage = stage or {}
        self.connection.execute(
            """
            INSERT INTO telemetry(
                test_id, sample_index, elapsed_s, system_state, mode, flags,
                fault_flags, command_sequence, engine_rpm, target_engine_rpm,
                pump_rpm, target_pump_rpm, torque_nm, target_torque_nm,
                flow_valve_duty_pct, pressure_valve_duty_pct, throttle_pct,
                power_kw, cvt_ratio, estimated_efficiency_pct, wheel_rpm,
                stage_index, stage_name, sweep_direction, record_enabled,
                engine_sensor_rpm, selected_engine_rpm, derived_engine_rpm,
                rpm_source_difference, actuator, target_type, target_value,
                target_error, flow_command_pct, pressure_command_pct,
                anti_stall_active, cvt_position_pct
            ) VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                test_id,
                sample_index,
                elapsed_s,
                int(value.state),
                int(value.mode),
                int(value.flags),
                value.fault_flags,
                value.last_command_sequence,
                value.engine_rpm,
                value.target_engine_rpm,
                value.pump_rpm,
                value.target_pump_rpm,
                value.torque_nm,
                value.target_torque_nm,
                value.flow_valve_duty_pct,
                value.pressure_valve_duty_pct,
                value.throttle_pct,
                value.power_kw,
                calculated.get("cvt_ratio"),
                calculated.get("estimated_efficiency_pct"),
                calculated.get("wheel_rpm"),
                int(stage.get("index", 0)),
                str(stage.get("name", "")),
                str(stage.get("direction", "")),
                int(bool(stage.get("record", True))),
                value.engine_sensor_rpm,
                value.selected_engine_rpm,
                calculated.get("derived_engine_rpm"),
                calculated.get("rpm_source_difference"),
                str(stage.get("actuator", "")),
                str(stage.get("target_type", "")),
                calculated.get("target_value"),
                calculated.get("target_error"),
                calculated.get("flow_command_pct"),
                calculated.get("pressure_command_pct"),
                int(bool(calculated.get("anti_stall_active", False))),
                calculated.get("cvt_position_pct"),
            ),
        )
        if sample_index % 20 == 0:
            self.connection.commit()

    def finish_test(
        self, test_id: int, status: str, results: dict[str, Any] | None = None
    ) -> None:
        self.connection.execute(
            "UPDATE tests SET status=?, finished_utc=?, results_json=? WHERE id=?",
            (status, _utc_now(), json.dumps(results or {}, sort_keys=True), test_id),
        )
        self.connection.commit()

    def export_telemetry_csv(self, test_id: int, destination: str | Path) -> Path:
        destination = Path(destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        test = self.connection.execute("SELECT * FROM tests WHERE id=?", (test_id,)).fetchone()
        if test is None:
            raise KeyError(f"No test with id {test_id}")
        samples = self.connection.execute(
            "SELECT * FROM telemetry WHERE test_id=? ORDER BY sample_index", (test_id,)
        )
        with destination.open("w", newline="", encoding="utf-8") as handle:
            handle.write(f"# western_dyno_schema,{SCHEMA_VERSION}\n")
            for field in ("name", "test_type", "status", "started_utc", "finished_utc"):
                handle.write(f"# {field},{test[field] or ''}\n")
            handle.write(f"# setup_json,{test['setup_json']}\n")
            handle.write(f"# parameters_json,{test['parameters_json']}\n")
            handle.write(f"# results_json,{test['results_json']}\n")
            writer = csv.writer(handle)
            columns = [item[0] for item in samples.description]
            writer.writerow(columns)
            writer.writerows(samples)
        return destination

    # Compatibility with v0.2 callers.
    export_test_csv = export_telemetry_csv

    def export_summary_csv(self, test_id: int, destination: str | Path) -> Path:
        destination = Path(destination); destination.parent.mkdir(parents=True, exist_ok=True)
        test = self.test(test_id)
        results = json.loads(test["results_json"] or "{}")
        with destination.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["metric", "value"])
            for key, value in _flatten(results):
                writer.writerow([key, json.dumps(value) if isinstance(value, (dict, list)) else value])
        return destination

    def list_tests(self, limit: int = 100) -> list[sqlite3.Row]:
        return list(
            self.connection.execute(
                "SELECT * FROM tests ORDER BY started_utc DESC LIMIT ?", (limit,)
            )
        )

    def test(self, test_id: int) -> sqlite3.Row:
        row = self.connection.execute("SELECT * FROM tests WHERE id=?", (test_id,)).fetchone()
        if row is None:
            raise KeyError(f"No test with id {test_id}")
        return row

    def samples(self, test_id: int) -> list[sqlite3.Row]:
        return list(
            self.connection.execute(
                "SELECT * FROM telemetry WHERE test_id=? ORDER BY sample_index", (test_id,)
            )
        )


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _flatten(value: dict[str, Any], prefix: str = ""):
    for key, item in value.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(item, dict):
            yield from _flatten(item, name)
        else:
            yield name, item
