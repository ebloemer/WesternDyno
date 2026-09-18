from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QSlider, QSpinBox, QVBoxLayout, QWidget


class MetricCard(QFrame):
    def __init__(self, title: str, unit: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("card")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 11, 14, 11)
        title_label = QLabel(title.upper())
        title_label.setObjectName("muted")
        self.value_label = QLabel("—")
        self.value_label.setObjectName("metricValue")
        self.unit_label = QLabel(unit)
        self.unit_label.setObjectName("metricUnit")
        layout.addWidget(title_label)
        row = QHBoxLayout()
        row.addWidget(self.value_label)
        row.addWidget(self.unit_label, 0, Qt.AlignmentFlag.AlignBottom)
        row.addStretch()
        layout.addLayout(row)

    def set_value(self, value: float | None, decimals: int = 0) -> None:
        self.value_label.setText("—" if value is None else f"{value:,.{decimals}f}")

    def set_text(self, value: str) -> None:
        self.value_label.setText(value)

    def set_unit(self, unit: str) -> None:
        self.unit_label.setText(unit)


class SliderSpinControl(QWidget):
    """A synchronized slider and numeric field with one consolidated signal."""

    value_changed = Signal(int)

    def __init__(self, minimum: int, maximum: int, value: int, suffix: str = "", parent=None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(minimum, maximum)
        self.slider.setValue(value)
        self.field = QSpinBox()
        self.field.setRange(minimum, maximum)
        self.field.setValue(value)
        self.field.setSuffix(suffix)
        self.field.setKeyboardTracking(False)
        self.field.setMinimumWidth(112)
        self.slider.valueChanged.connect(self._from_slider)
        self.field.valueChanged.connect(self._from_field)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.field)

    def value(self) -> int:
        return self.field.value()

    def set_value(self, value: int) -> None:
        self._sync(int(value), emit=False)

    def _from_slider(self, value: int) -> None:
        self._sync(value, emit=True)

    def _from_field(self, value: int) -> None:
        self._sync(value, emit=True)

    def _sync(self, value: int, *, emit: bool) -> None:
        self.slider.blockSignals(True)
        self.field.blockSignals(True)
        self.slider.setValue(value)
        self.field.setValue(value)
        self.slider.blockSignals(False)
        self.field.blockSignals(False)
        if emit:
            self.value_changed.emit(value)


@dataclass(slots=True)
class PlotSeries:
    name: str
    color: QColor
    values: deque[tuple[float, float]]


class LivePlot(QWidget):
    """Dependency-free rolling plot suited to live telemetry."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(250)
        self.setAutoFillBackground(False)
        self.window_s = 30.0
        self.series = {
            "engine": PlotSeries("Engine RPM", QColor("#a77bed"), deque(maxlen=2000)),
            "pump": PlotSeries("Pump RPM", QColor("#36c5d7"), deque(maxlen=2000)),
            "target": PlotSeries("Target", QColor("#f0b44d"), deque(maxlen=2000)),
        }

    def append(self, elapsed_s: float, engine: float, pump: float, target: float) -> None:
        for key, value in (("engine", engine), ("pump", pump), ("target", target)):
            self.series[key].values.append((elapsed_s, value))
        self.update()

    def clear(self) -> None:
        for series in self.series.values():
            series.values.clear()
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        area = QRectF(48, 14, max(1, self.width() - 62), max(1, self.height() - 42))
        painter.fillRect(self.rect(), QColor("#10141c"))
        painter.setPen(QPen(QColor("#292f3b"), 1))
        for index in range(6):
            y = area.top() + area.height() * index / 5
            painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))
        for index in range(7):
            x = area.left() + area.width() * index / 6
            painter.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()))

        all_values = [item for series in self.series.values() for item in series.values]
        if not all_values:
            painter.setPen(QColor("#737a8b"))
            painter.drawText(area, Qt.AlignmentFlag.AlignCenter, "Live telemetry will appear here")
            return
        latest = max(point[0] for point in all_values)
        start = max(0.0, latest - self.window_s)
        visible = [point[1] for point in all_values if point[0] >= start]
        maximum = max(1000.0, max(visible, default=1000.0) * 1.08)
        painter.setPen(QColor("#8e95a5"))
        for index in range(6):
            value = maximum * (5 - index) / 5
            y = area.top() + area.height() * index / 5
            painter.drawText(QRectF(0, y - 9, 43, 18), Qt.AlignmentFlag.AlignRight, f"{value:.0f}")

        span = max(self.window_s, latest)
        for series in self.series.values():
            points = [(t, value) for t, value in series.values if t >= start]
            if len(points) < 2:
                continue
            path = QPainterPath()
            for index, (timestamp, value) in enumerate(points):
                x = area.left() + (timestamp - start) / span * area.width()
                y = area.bottom() - max(0.0, value) / maximum * area.height()
                if index == 0:
                    path.moveTo(x, y)
                else:
                    path.lineTo(x, y)
            painter.setPen(QPen(series.color, 2))
            painter.drawPath(path)

        x = area.left()
        for series in self.series.values():
            painter.setPen(QPen(series.color, 3))
            painter.drawLine(QPointF(x, area.bottom() + 18), QPointF(x + 16, area.bottom() + 18))
            painter.setPen(QColor("#c5cad5"))
            painter.drawText(QPointF(x + 21, area.bottom() + 22), series.name)
            x += 120


class CurvePlot(QWidget):
    """Static multi-series XY result plot with automatic bounds."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(260)
        self.title = "Select a completed test"
        self.x_label = ""
        self.y_label = ""
        self.curves: list[tuple[str, QColor, list[tuple[float, float]]]] = []

    def set_curves(
        self,
        title: str,
        x_label: str,
        y_label: str,
        curves: list[tuple[str, str, list[tuple[float, float]]]],
    ) -> None:
        self.title = title
        self.x_label = x_label
        self.y_label = y_label
        self.curves = [(name, QColor(color), points) for name, color, points in curves]
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#10141c"))
        area = QRectF(62, 36, max(1, self.width() - 82), max(1, self.height() - 80))
        painter.setPen(QColor("#e2e5ec"))
        painter.drawText(QRectF(0, 6, self.width(), 24), Qt.AlignmentFlag.AlignCenter, self.title)
        all_points = [point for _, _, points in self.curves for point in points]
        if not all_points:
            painter.setPen(QColor("#737a8b"))
            painter.drawText(area, Qt.AlignmentFlag.AlignCenter, "No recorded samples")
            return
        x_values = [point[0] for point in all_points]
        y_values = [point[1] for point in all_points]
        x_min, x_max = min(x_values), max(x_values)
        y_min, y_max = min(y_values), max(y_values)
        if x_max <= x_min: x_max = x_min + 1.0
        if y_max <= y_min: y_max = y_min + 1.0
        y_padding = (y_max - y_min) * 0.08
        y_min -= y_padding; y_max += y_padding
        painter.setPen(QPen(QColor("#292f3b"), 1))
        for index in range(6):
            x = area.left() + area.width() * index / 5
            y = area.top() + area.height() * index / 5
            painter.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()))
            painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))
            painter.setPen(QColor("#8e95a5"))
            painter.drawText(QRectF(x - 35, area.bottom() + 4, 70, 18), Qt.AlignmentFlag.AlignCenter, f"{x_min + (x_max-x_min)*index/5:.0f}")
            painter.drawText(QRectF(0, y - 9, 56, 18), Qt.AlignmentFlag.AlignRight, f"{y_max - (y_max-y_min)*index/5:.1f}")
            painter.setPen(QPen(QColor("#292f3b"), 1))
        painter.setPen(QColor("#a9afbc"))
        painter.drawText(QRectF(area.left(), area.bottom() + 24, area.width(), 20), Qt.AlignmentFlag.AlignCenter, self.x_label)
        painter.save(); painter.translate(14, area.center().y()); painter.rotate(-90); painter.drawText(QRectF(-area.height()/2, -10, area.height(), 20), Qt.AlignmentFlag.AlignCenter, self.y_label); painter.restore()
        for name, color, points in self.curves:
            valid = sorted(points)
            if len(valid) < 2: continue
            path = QPainterPath()
            for index, (x_value, y_value) in enumerate(valid):
                x = area.left() + (x_value - x_min) / (x_max - x_min) * area.width()
                y = area.bottom() - (y_value - y_min) / (y_max - y_min) * area.height()
                if index == 0: path.moveTo(x, y)
                else: path.lineTo(x, y)
            painter.setPen(QPen(color, 2)); painter.drawPath(path)
        legend_x = area.left()
        for name, color, _ in self.curves:
            painter.setPen(QPen(color, 3)); painter.drawLine(QPointF(legend_x, area.top() - 10), QPointF(legend_x + 14, area.top() - 10))
            painter.setPen(QColor("#c5cad5")); painter.drawText(QPointF(legend_x + 19, area.top() - 6), name)
            legend_x += 125
