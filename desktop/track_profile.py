"""CSV-defined automatic test profiles and their on-disk libraries."""

from __future__ import annotations

from dataclasses import dataclass
import csv
import math
from pathlib import Path
import shutil

VALID_ACTUATORS = {"pressure", "flow"}
VALID_TARGETS = {"torque", "engine_rpm", "pump_rpm", "wheel_rpm"}
VALID_UNITS = {"metric", "imperial"}


class ProfileError(ValueError):
    pass


def _read_profile(path: str | Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    path = Path(path)
    metadata: dict[str, str] = {}
    data_lines: list[str] = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        for raw in handle:
            stripped = raw.strip()
            if stripped.startswith("#"):
                item = stripped[1:].strip()
                if "=" in item:
                    key, value = item.split("=", 1)
                    metadata[key.strip().lower()] = value.strip()
            elif stripped:
                data_lines.append(raw)
    if not data_lines:
        raise ProfileError(f"{path.name} contains no CSV table")
    return metadata, list(csv.DictReader(data_lines))


def _metadata(metadata: dict[str, str], path: Path) -> tuple[str, str, str, str]:
    name = metadata.get("profile_name", path.stem.replace("_", " ").title())
    actuator = metadata.get("actuator", "").lower()
    target = metadata.get("target", "").lower()
    units = metadata.get("units", "metric").lower()
    if actuator not in VALID_ACTUATORS:
        raise ProfileError("Metadata actuator must be pressure or flow")
    if target not in VALID_TARGETS:
        raise ProfileError("Metadata target must be torque, engine_rpm, pump_rpm, or wheel_rpm")
    if units not in VALID_UNITS:
        raise ProfileError("Metadata units must be metric or imperial")
    return name, actuator, target, units


def _number(row: dict[str, str], name: str, *, default: float | None = None) -> float:
    raw = row.get(name, "")
    if raw is None or not str(raw).strip():
        if default is not None:
            return default
        raise ProfileError(f"Missing required column value: {name}")
    try:
        value = float(raw)
    except ValueError as exc:
        raise ProfileError(f"{name} must be numeric") from exc
    if not math.isfinite(value):
        raise ProfileError(f"{name} must be finite")
    return value


@dataclass(frozen=True, slots=True)
class TrackPoint:
    time_s: float
    target: float
    throttle_pct: float
    extras: dict[str, str]


@dataclass(frozen=True, slots=True)
class TrackProfile:
    name: str
    actuator: str
    target_type: str
    units: str
    points: tuple[TrackPoint, ...]
    source_path: str

    @classmethod
    def from_csv(cls, path: str | Path) -> "TrackProfile":
        path = Path(path)
        metadata, rows = _read_profile(path)
        if "actuator" not in metadata:
            metadata["actuator"] = "pressure"
        if "target" not in metadata and rows:
            fields = set(rows[0])
            if "target_engine_rpm" in fields:
                metadata["target"] = "engine_rpm"
            elif "target_wheel_rpm" in fields:
                metadata["target"] = "wheel_rpm"
        name, actuator, target_type, units = _metadata(metadata, path)
        points: list[TrackPoint] = []
        legacy_column = f"target_{target_type}"
        for row in rows:
            normalized = dict(row)
            normalized["target"] = row.get("target", row.get(legacy_column, "")) or ""
            time_s = _number(normalized, "time_s")
            target = _number(normalized, "target")
            throttle = _number(normalized, "throttle_pct")
            if time_s < 0 or target < 0 or not 0 <= throttle <= 100:
                raise ProfileError("time and target must be non-negative; throttle must be 0–100")
            if units == "imperial" and target_type == "torque":
                target *= 1.3558179483
            points.append(TrackPoint(time_s, target, throttle, {
                key: value for key, value in row.items()
                if key not in {"time_s", "target", legacy_column, "throttle_pct"}
            }))
        if len(points) < 2:
            raise ProfileError("A track profile requires at least two points")
        if points[0].time_s != 0:
            raise ProfileError("The first track point must start at time_s = 0")
        if any(b.time_s <= a.time_s for a, b in zip(points, points[1:])):
            raise ProfileError("Track time_s values must be strictly increasing")
        return cls(name, actuator, target_type, units, tuple(points), str(path))

    @property
    def duration_s(self) -> float:
        return self.points[-1].time_s

    def target_at(self, elapsed_s: float) -> TrackPoint:
        if elapsed_s <= 0:
            return self.points[0]
        if elapsed_s >= self.duration_s:
            return self.points[-1]
        for left, right in zip(self.points, self.points[1:]):
            if left.time_s <= elapsed_s <= right.time_s:
                fraction = (elapsed_s - left.time_s) / (right.time_s - left.time_s)
                return TrackPoint(elapsed_s, left.target + (right.target - left.target) * fraction,
                                  left.throttle_pct + (right.throttle_pct - left.throttle_pct) * fraction,
                                  left.extras)
        return self.points[-1]


@dataclass(frozen=True, slots=True)
class CharacterizationStage:
    stage: str
    target: float
    transition_s: float
    hold_s: float
    throttle_pct: float
    throttle_transition_s: float
    group: str
    repeat_index: int
    record: bool
    notes: str = ""


@dataclass(frozen=True, slots=True)
class CharacterizationProfile:
    name: str
    actuator: str
    target_type: str
    units: str
    stages: tuple[CharacterizationStage, ...]
    source_path: str

    @classmethod
    def from_csv(cls, path: str | Path) -> "CharacterizationProfile":
        path = Path(path)
        metadata, rows = _read_profile(path)
        name, actuator, target_type, units = _metadata(metadata, path)
        if actuator != "pressure" or target_type != "torque":
            raise ProfileError("CVT characterization requires actuator=pressure and target=torque; the actuator may not switch during a run")
        grouped: dict[str, list[tuple[str, float, float, float, float, float, bool, str, int]]] = {}
        group_order: list[str] = []
        for row_index, row in enumerate(rows, 1):
            stage = (row.get("stage") or f"Stage {row_index}").strip()
            target = _number(row, "target")
            transition = _number(row, "transition_s", default=0.0)
            hold = _number(row, "hold_s", default=0.0)
            throttle = _number(row, "throttle_pct")
            throttle_transition = _number(row, "throttle_transition_s", default=transition)
            repeats = int(_number(row, "group_repeats", default=1.0))
            group = (row.get("group") or "main").strip()
            record = (row.get("record") or "true").strip().lower() not in {"0", "false", "no"}
            if units == "imperial":
                target *= 1.3558179483
            if target < 0 or transition < 0 or hold < 0 or throttle_transition < 0:
                raise ProfileError("Targets and stage times must be non-negative")
            if transition + hold <= 0 or not 0 <= throttle <= 100:
                raise ProfileError("Each stage needs positive total time and throttle must be 0–100")
            if not 1 <= repeats <= 1000:
                raise ProfileError("group_repeats must be from 1 through 1000")
            if group not in grouped:
                grouped[group] = []
                group_order.append(group)
            grouped[group].append((stage, target, transition, hold, throttle,
                throttle_transition, record, (row.get("notes") or "").strip(), repeats))
        parsed: list[CharacterizationStage] = []
        for group in group_order:
            rows_in_group = grouped[group]
            repeat_counts = {item[-1] for item in rows_in_group}
            if len(repeat_counts) != 1:
                raise ProfileError(f"All rows in group '{group}' must use the same group_repeats")
            for repeat in range(1, repeat_counts.pop() + 1):
                for stage, target, transition, hold, throttle, throttle_transition, record, notes, _ in rows_in_group:
                    parsed.append(CharacterizationStage(stage, target, transition, hold, throttle,
                        throttle_transition, group, repeat, record, notes))
        if not parsed:
            raise ProfileError("A characterization profile requires at least one stage")
        return cls(name, actuator, target_type, units, tuple(parsed), str(path))

    @property
    def duration_s(self) -> float:
        return sum(stage.transition_s + stage.hold_s for stage in self.stages)


@dataclass(frozen=True, slots=True)
class ProfileEntry:
    name: str
    path: Path
    builtin: bool


class ProfileLibrary:
    """Merges protected bundled examples with operator-imported profiles."""

    def __init__(self, bundled_dir: Path, user_dir: Path, parser) -> None:
        self.bundled_dir, self.user_dir, self.parser = Path(bundled_dir), Path(user_dir), parser
        self.user_dir.mkdir(parents=True, exist_ok=True)

    def entries(self) -> list[ProfileEntry]:
        result: list[ProfileEntry] = []
        names: set[str] = set()
        for directory, builtin in ((self.bundled_dir, True), (self.user_dir, False)):
            if not directory.exists():
                continue
            for path in sorted(directory.glob("*.csv")):
                try:
                    profile = self.parser(path)
                except (OSError, ValueError):
                    continue
                key = profile.name.casefold()
                if key in names:
                    continue
                names.add(key)
                result.append(ProfileEntry(profile.name, path, builtin))
        return result

    def import_file(self, source: str | Path) -> ProfileEntry:
        source = Path(source)
        self.parser(source)
        destination = self.user_dir / source.name
        counter = 2
        while destination.exists():
            destination = self.user_dir / f"{source.stem}_{counter}{source.suffix}"
            counter += 1
        shutil.copy2(source, destination)
        profile = self.parser(destination)
        return ProfileEntry(profile.name, destination, False)

    def delete(self, entry: ProfileEntry) -> None:
        if entry.builtin:
            raise ProfileError("Bundled profiles are protected and cannot be deleted")
        entry.path.unlink()

    @staticmethod
    def export(entry: ProfileEntry, destination: str | Path) -> None:
        shutil.copy2(entry.path, destination)
