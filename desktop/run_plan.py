"""Autonomous, serializable test plans for peak-power and CVT testing."""

from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass(frozen=True, slots=True)
class TestStage:
    name: str
    start_rpm: float
    end_rpm: float
    duration_s: float
    throttle_pct: float
    direction: str = "hold"  # prepare, upshift, downshift, hold
    record: bool = True

    def validate(self) -> None:
        if not self.name.strip():
            raise ValueError("Test-stage name cannot be empty")
        if self.duration_s <= 0:
            raise ValueError("Test-stage duration must be positive")
        if not 0 <= self.start_rpm <= 100_000 or not 0 <= self.end_rpm <= 100_000:
            raise ValueError("Test-stage RPM must be between 0 and 100,000")
        if not 0 <= self.throttle_pct <= 100:
            raise ValueError("Test-stage throttle must be between 0 and 100%")
        if self.direction not in {"prepare", "upshift", "downshift", "hold"}:
            raise ValueError(f"Unsupported test-stage direction: {self.direction}")

    def to_dict(self) -> dict:
        self.validate()
        return asdict(self)


def total_duration(stages: list[TestStage]) -> float:
    return sum(stage.duration_s for stage in stages)


def peak_power_plan(
    lower_rpm: float,
    upper_rpm: float,
    sweep_duration_s: float,
    direction: str,
    through_cvt: bool,
    throttle_pct: float = 100.0,
) -> list[TestStage]:
    """Build a full-throttle plan, including the preparation a down sweep needs."""
    lower, upper = sorted((float(lower_rpm), float(upper_rpm)))
    if lower == upper:
        raise ValueError("Peak-power start and end RPM must be different")
    if direction not in {"up", "down", "both"}:
        raise ValueError("Peak-power direction must be up, down, or both")
    prefix = "CVT " if through_cvt else ""
    settle_s = max(3.0, min(8.0, float(sweep_duration_s) * 0.20))
    stages: list[TestStage] = []
    if direction in {"up", "both"}:
        stages.append(TestStage(f"Prepare at {lower:.0f} RPM", lower, lower, settle_s, throttle_pct, "prepare", False))
        stages.append(TestStage(f"{prefix}upshift peak-power sweep", lower, upper, sweep_duration_s, throttle_pct, "upshift", True))
    if direction == "both":
        stages.append(TestStage("High-speed stabilization", upper, upper, settle_s, throttle_pct, "hold", False))
    if direction in {"down", "both"}:
        if direction == "down":
            approach_s = max(5.0, min(20.0, float(sweep_duration_s) * 0.50))
            stages.append(TestStage(f"Prepare at {upper:.0f} RPM", lower, upper, approach_s, throttle_pct, "prepare", False))
            stages.append(TestStage("High-speed stabilization", upper, upper, settle_s, throttle_pct, "hold", False))
        stages.append(TestStage(f"{prefix}downshift peak-power sweep", upper, lower, sweep_duration_s, throttle_pct, "downshift", True))
    for stage in stages:
        stage.validate()
    return stages

