"""Interpolation of a measured direct-engine power map for CVT estimates."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class PowerPoint:
    engine_rpm: float
    power_kw: float


class EnginePowerMap:
    def __init__(self, points: list[PowerPoint], source: str = "") -> None:
        points = sorted(points, key=lambda point: point.engine_rpm)
        if len(points) < 2:
            raise ValueError("Engine power map requires at least two valid samples")
        self.points = points
        self.source = source

    @classmethod
    def from_csv(cls, path: str | Path) -> "EnginePowerMap":
        path = Path(path)
        with path.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(
                line for line in handle if not line.lstrip().startswith("#")
            )
            fields = set(reader.fieldnames or [])
            if not {"engine_rpm", "power_kw"}.issubset(fields):
                raise ValueError("Engine map CSV requires engine_rpm and power_kw columns")
            points: list[PowerPoint] = []
            for row in reader:
                try:
                    rpm = float(row["engine_rpm"])
                    power = float(row["power_kw"])
                except (TypeError, ValueError):
                    continue
                if rpm > 0 and power > 0:
                    points.append(PowerPoint(rpm, power))
        return cls(points, str(path))

    def power_at(self, rpm: float) -> float | None:
        # Do not extrapolate: an efficiency estimate outside the measured map
        # would look precise while being physically unsupported.
        if rpm < self.points[0].engine_rpm or rpm > self.points[-1].engine_rpm:
            return None
        for left, right in zip(self.points, self.points[1:]):
            if left.engine_rpm <= rpm <= right.engine_rpm:
                if right.engine_rpm == left.engine_rpm:
                    return (left.power_kw + right.power_kw) / 2.0
                fraction = (rpm - left.engine_rpm) / (right.engine_rpm - left.engine_rpm)
                return left.power_kw + fraction * (right.power_kw - left.power_kw)
        return None

