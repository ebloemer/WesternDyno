"""Persistent preferences, machine parameters, and named mechanical setups."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields, replace
import json
import math
from pathlib import Path
from uuid import NAMESPACE_URL, uuid4, uuid5


APP_DIR = Path.home() / ".western_dyno"
SETTINGS_PATH = APP_DIR / "settings.json"
DATABASE_PATH = APP_DIR / "western_dyno.sqlite3"
CHARACTERIZATION_PROFILE_DIR = APP_DIR / "profiles" / "characterization"
TRACK_PROFILE_DIR = APP_DIR / "profiles" / "tracks"
DEFAULT_PROFILE_ID = "default-direct-engine"

PID_BANK_NAMES = (
    "pressure_torque", "pressure_engine_rpm", "pressure_pump_rpm",
    "flow_torque", "flow_engine_rpm", "flow_pump_rpm",
)


@dataclass(slots=True)
class PidTuning:
    p: float = 0.25
    i: float = 0.05
    d: float = 0.01
    output_slew_pct_s: float = 100.0
    enabled: bool = False

    def validate(self, label: str = "PID") -> None:
        for name in ("p", "i", "d"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{label} {name.upper()} must be finite and non-negative")
        if not math.isfinite(self.output_slew_pct_s) or self.output_slew_pct_s <= 0:
            raise ValueError(f"{label} output slew must be positive")


def _default_pid_banks() -> dict[str, PidTuning]:
    return {
        "pressure_torque": PidTuning(1.0, 0.2, 0.05),
        "pressure_engine_rpm": PidTuning(0.1, 0.02, 0.005),
        "pressure_pump_rpm": PidTuning(0.1, 0.02, 0.005),
        "flow_torque": PidTuning(0.25, 0.0625, 0.025),
        "flow_engine_rpm": PidTuning(0.25, 0.0625, 0.025),
        "flow_pump_rpm": PidTuning(0.25, 0.0625, 0.025),
    }


@dataclass(slots=True)
class MachineParameters:
    flow_pwm_min_pct: float = 0.0
    flow_pwm_max_pct: float = 98.0
    pressure_pwm_min_pct: float = 0.0
    pressure_pwm_max_pct: float = 98.0
    pump_pulses_per_rev: float = 4.0
    pump_limit_enabled: bool = True
    pump_max_rpm: float = 3000.0
    load_cell_scale_factor: float = 2280.0
    torque_filter_alpha: float = 0.2
    pid_banks: dict[str, PidTuning] = field(default_factory=_default_pid_banks)

    def validate(self) -> None:
        for label, minimum, maximum in (
            ("Flow valve", self.flow_pwm_min_pct, self.flow_pwm_max_pct),
            ("Pressure valve", self.pressure_pwm_min_pct, self.pressure_pwm_max_pct),
        ):
            if not (0.0 <= minimum < maximum <= 99.0):
                raise ValueError(f"{label} active PWM range must satisfy 0 ≤ minimum < maximum ≤ 99")
        if not math.isfinite(self.pump_pulses_per_rev) or self.pump_pulses_per_rev <= 0:
            raise ValueError("Pump pulses per revolution must be positive")
        if not math.isfinite(self.pump_max_rpm) or self.pump_max_rpm <= 0:
            raise ValueError("Maximum pump RPM must be positive")
        if not math.isfinite(self.load_cell_scale_factor) or not 1 <= abs(self.load_cell_scale_factor) <= 10_000_000:
            raise ValueError("Load-cell scale magnitude must be between 1 and 10,000,000")
        if not math.isfinite(self.torque_filter_alpha) or not 0.001 <= self.torque_filter_alpha <= 1.0:
            raise ValueError("Torque filter alpha must be between 0.001 and 1")
        if set(self.pid_banks) != set(PID_BANK_NAMES):
            raise ValueError("Machine parameters must contain all six PID banks")
        for name, bank in self.pid_banks.items():
            bank.validate(name.replace("_", " ").title())

    @classmethod
    def from_dict(cls, data: dict) -> "MachineParameters":
        allowed = {item.name for item in fields(cls)}
        values = {key: value for key, value in data.items() if key in allowed and key != "pid_banks"}
        machine = cls(**values)
        raw_banks = data.get("pid_banks", {})
        defaults = _default_pid_banks()
        machine.pid_banks = {}
        allowed_pid = {item.name for item in fields(PidTuning)}
        for name in PID_BANK_NAMES:
            raw = raw_banks.get(name, {})
            merged = asdict(defaults[name])
            merged.update({key: value for key, value in raw.items() if key in allowed_pid})
            machine.pid_banks[name] = PidTuning(**merged)
        machine.validate()
        return machine


@dataclass(slots=True)
class SetupProfile:
    profile_id: str = DEFAULT_PROFILE_ID
    name: str = "Default direct-engine setup"
    configuration: str = "direct"
    direct_rpm_source: str = "derived"
    engine_to_pump_ratio: float = 0.7
    cvt_output_to_pump_ratio: float = 0.7
    vehicle_data_enabled: bool = False
    final_drive_ratio: float = 8.5
    tire_diameter_in: float = 23.0
    engine_pulses_per_rev: float = 1.0
    throttle_closed_us: float = 500.0
    throttle_full_us: float = 1200.0
    throttle_frequency_hz: float = 150.0
    engine_overspeed_enabled: bool = True
    engine_overspeed_rpm: float = 4100.0
    idle_rpm: float = 1400.0
    stall_warning_rpm: float = 900.0
    recovery_rpm: float = 1200.0
    recovery_timeout_s: float = 3.0
    expected_cvt_min_ratio: float = 0.9
    expected_cvt_max_ratio: float = 3.9
    cvt_primary: str = ""
    cvt_primary_spring: str = ""
    cvt_flyweights: str = ""
    cvt_secondary: str = ""
    cvt_secondary_spring: str = ""
    cvt_helix: str = ""
    cvt_preload: str = ""
    cvt_belt: str = ""
    cvt_belt_notes: str = ""
    notes: str = ""

    def validate(self) -> None:
        if not self.profile_id.strip() or not self.name.strip():
            raise ValueError("Setup ID and name cannot be empty")
        if self.configuration not in {"direct", "cvt"}:
            raise ValueError("configuration must be 'direct' or 'cvt'")
        if self.direct_rpm_source not in {"derived", "sensor"}:
            raise ValueError("Direct-engine RPM source must be 'derived' or 'sensor'")
        for name in ("engine_to_pump_ratio", "cvt_output_to_pump_ratio", "engine_pulses_per_rev"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.vehicle_data_enabled and (self.final_drive_ratio <= 0 or self.tire_diameter_in <= 0):
            raise ValueError("Vehicle final-drive ratio and tire diameter must be positive")
        if not math.isfinite(self.throttle_closed_us) or self.throttle_closed_us <= 0:
            raise ValueError("Closed-throttle pulse width must be positive")
        if not math.isfinite(self.throttle_full_us) or self.throttle_full_us <= 0:
            raise ValueError("Full-throttle pulse width must be positive")
        if self.throttle_closed_us == self.throttle_full_us:
            raise ValueError("Closed- and full-throttle pulse widths cannot be equal")
        if not math.isfinite(self.throttle_frequency_hz) or self.throttle_frequency_hz <= 0:
            raise ValueError("Throttle-servo frequency must be positive")
        for name in ("engine_overspeed_rpm", "idle_rpm", "stall_warning_rpm", "recovery_rpm", "recovery_timeout_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.recovery_rpm <= self.stall_warning_rpm:
            raise ValueError("Recovery RPM must be greater than stall-warning RPM")
        if self.configuration == "cvt" and (
            self.expected_cvt_min_ratio <= 0 or self.expected_cvt_max_ratio <= self.expected_cvt_min_ratio
        ):
            raise ValueError("Expected CVT ratio bounds must satisfy 0 < minimum < maximum")

    @property
    def uses_engine_sensor(self) -> bool:
        return self.configuration == "cvt" or self.direct_rpm_source == "sensor"

    def clone(self, *, temporary: bool = False) -> "SetupProfile":
        return replace(self, profile_id="temporary" if temporary else str(uuid4()))


@dataclass(slots=True)
class Preferences:
    unit_system: str = "metric"
    theme: str = "western_dark"
    accent_rgb: tuple[int, int, int] = (93, 54, 160)
    background_rgb: tuple[int, int, int] = (11, 13, 18)
    panel_rgb: tuple[int, int, int] = (19, 23, 32)
    warning_rgb: tuple[int, int, int] = (155, 31, 45)
    database_path: str = str(DATABASE_PATH)
    export_directory: str = str(Path.home() / "Documents" / "WesternDyno")
    simulator_enabled: bool = False
    last_setup_id: str = DEFAULT_PROFILE_ID
    nominal_valve_voltage: float = 12.0
    window_width: int = 1440
    window_height: int = 900


class SettingsStore:
    def __init__(self, path: Path = SETTINGS_PATH) -> None:
        self.path = Path(path)
        self.preferences = Preferences()
        self.machine = MachineParameters()
        self.profiles: list[SetupProfile] = [SetupProfile()]
        self.load()

    def load(self) -> None:
        if not self.path.exists():
            return
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
            preference_data = dict(raw.get("preferences", {}))
            legacy_last_name = preference_data.pop("last_setup", "")
            allowed_preferences = {item.name for item in fields(Preferences)}
            preference_data = {key: value for key, value in preference_data.items() if key in allowed_preferences}
            for color_name in ("accent_rgb", "background_rgb", "panel_rgb", "warning_rgb"):
                if color_name in preference_data:
                    preference_data[color_name] = tuple(preference_data[color_name])
            self.preferences = Preferences(**preference_data)
            self.machine = MachineParameters.from_dict(raw.get("machine", {}))
            loaded: list[SetupProfile] = []
            allowed_profile = {item.name for item in fields(SetupProfile)}
            for item in raw.get("profiles", []):
                profile_data = {key: value for key, value in item.items() if key in allowed_profile}
                if not profile_data.get("profile_id"):
                    legacy_name = str(profile_data.get("name", "setup"))
                    profile_data["profile_id"] = str(uuid5(NAMESPACE_URL, f"western-dyno:{legacy_name}"))
                profile = SetupProfile(**profile_data)
                profile.validate()
                loaded.append(profile)
            if loaded:
                self.profiles = loaded
            if legacy_last_name:
                match = next((p for p in self.profiles if p.name == legacy_last_name), None)
                if match is not None:
                    self.preferences.last_setup_id = match.profile_id
            if not any(p.profile_id == self.preferences.last_setup_id for p in self.profiles):
                self.preferences.last_setup_id = self.profiles[0].profile_id
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            try:
                self.path.replace(self.path.with_suffix(".invalid.json"))
            except OSError:
                pass

    def save(self) -> None:
        self.machine.validate()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "schema_version": 3,
            "preferences": asdict(self.preferences),
            "machine": asdict(self.machine),
            "profiles": [asdict(profile) for profile in self.profiles],
        }
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, indent=2), encoding="utf-8")
        temporary.replace(self.path)

    def profile(self, profile_id: str) -> SetupProfile:
        return next((p for p in self.profiles if p.profile_id == profile_id), self.profiles[0])

    def _assert_unique_name(self, name: str, *, excluding_id: str = "") -> None:
        normalized = name.strip().casefold()
        if any(p.profile_id != excluding_id and p.name.strip().casefold() == normalized for p in self.profiles):
            raise ValueError(f'A setup named "{name.strip()}" already exists')

    def create_profile(self, profile: SetupProfile) -> SetupProfile:
        profile.validate()
        self._assert_unique_name(profile.name)
        if any(current.profile_id == profile.profile_id for current in self.profiles):
            profile = replace(profile, profile_id=str(uuid4()))
        self.profiles.append(profile)
        self.save()
        return profile

    def update_profile(self, profile_id: str, profile: SetupProfile) -> SetupProfile:
        profile.validate()
        self._assert_unique_name(profile.name, excluding_id=profile_id)
        for index, current in enumerate(self.profiles):
            if current.profile_id == profile_id:
                updated = replace(profile, profile_id=profile_id)
                self.profiles[index] = updated
                self.save()
                return updated
        raise KeyError(f"Unknown setup profile: {profile_id}")

    def delete_profile(self, profile_id: str) -> None:
        if len(self.profiles) <= 1:
            raise ValueError("At least one setup profile must remain")
        remaining = [p for p in self.profiles if p.profile_id != profile_id]
        if len(remaining) == len(self.profiles):
            raise KeyError(f"Unknown setup profile: {profile_id}")
        self.profiles = remaining
        if self.preferences.last_setup_id == profile_id:
            self.preferences.last_setup_id = self.profiles[0].profile_id
        self.save()
