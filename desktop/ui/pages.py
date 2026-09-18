from __future__ import annotations

from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel,
    QLineEdit, QMessageBox, QPushButton, QSpinBox, QSplitter, QStackedWidget,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from track_profile import CharacterizationProfile, ProfileEntry, ProfileLibrary, TrackProfile
from ui.widgets import CurvePlot, SliderSpinControl


def _heading(title: str, subtitle: str) -> tuple[QLabel, QLabel]:
    heading = QLabel(title); heading.setObjectName("pageTitle")
    detail = QLabel(subtitle); detail.setObjectName("muted"); detail.setWordWrap(True)
    return heading, detail


def _saturation_controls(parent_layout: QVBoxLayout) -> tuple[QCheckBox, QDoubleSpinBox, QDoubleSpinBox]:
    box = QGroupBox("Unreachable-target protection")
    form = QFormLayout(box)
    enabled = QCheckBox("Fault if the selected actuator remains saturated outside tolerance")
    enabled.setChecked(True)
    tolerance = QDoubleSpinBox(); tolerance.setRange(0, 100000); tolerance.setValue(100); tolerance.setSuffix(" target units")
    timeout = QDoubleSpinBox(); timeout.setRange(0.1, 300); timeout.setValue(5); timeout.setSuffix(" s")
    form.addRow(enabled); form.addRow("Error tolerance", tolerance); form.addRow("Continuous timeout", timeout)
    parent_layout.addWidget(box)
    return enabled, tolerance, timeout


class PeakPowerPage(QWidget):
    run_requested = Signal(dict)

    def __init__(self) -> None:
        super().__init__(); self.configuration = "direct"; self.pump_max_rpm = 3000
        layout = QVBoxLayout(self)
        self.title, self.subtitle = _heading("Peak Power", "Continuous flow-valve / RPM sweep with configurable throttle.")
        layout.addWidget(self.title); layout.addWidget(self.subtitle)
        box = QGroupBox("Sweep configuration"); form = QFormLayout(box)
        self.name = QLineEdit("Peak power sweep")
        self.start_rpm = QSpinBox(); self.start_rpm.setRange(0, 100000); self.start_rpm.setValue(2000); self.start_rpm.setSuffix(" RPM")
        self.end_rpm = QSpinBox(); self.end_rpm.setRange(0, 100000); self.end_rpm.setValue(3800); self.end_rpm.setSuffix(" RPM")
        self.duration = QDoubleSpinBox(); self.duration.setRange(1, 3600); self.duration.setValue(30); self.duration.setSuffix(" s")
        self.throttle = QSpinBox(); self.throttle.setRange(0, 100); self.throttle.setValue(100); self.throttle.setSuffix(" %")
        self.direction = QComboBox(); self.direction.addItem("Upshift / increasing RPM", "up"); self.direction.addItem("Downshift / decreasing RPM", "down"); self.direction.addItem("Both directions", "both")
        self.auto_max = QCheckBox("Automatically detect maximum RPM at full flow")
        self.pump_limit = QLabel(); self.pump_limit.setObjectName("muted")
        self.rise_threshold = QDoubleSpinBox(); self.rise_threshold.setRange(0, 10000); self.rise_threshold.setValue(15); self.rise_threshold.setSuffix(" RPM/s")
        self.plateau_dwell = QDoubleSpinBox(); self.plateau_dwell.setRange(0.1, 120); self.plateau_dwell.setValue(3); self.plateau_dwell.setSuffix(" s")
        self.overall_timeout = QDoubleSpinBox(); self.overall_timeout.setRange(1, 3600); self.overall_timeout.setValue(60); self.overall_timeout.setSuffix(" s")
        for label, widget in (("Test name", self.name), ("Start engine speed", self.start_rpm), ("End engine speed", self.end_rpm), ("Fixed / return sweep duration", self.duration), ("Engine throttle", self.throttle), ("Direction", self.direction)):
            form.addRow(label, widget)
        form.addRow(self.auto_max); form.addRow("Auto-max hard ceiling", self.pump_limit)
        form.addRow("RPM-rise plateau threshold", self.rise_threshold); form.addRow("Plateau dwell", self.plateau_dwell); form.addRow("Overall test timeout", self.overall_timeout)
        layout.addWidget(box)
        self.saturation_enabled, self.saturation_tolerance, self.saturation_timeout = _saturation_controls(layout)
        self.note = QLabel(); self.note.setWordWrap(True); self.note.setObjectName("muted"); layout.addWidget(self.note)
        run = QPushButton("Review and start peak-power plan"); run.setObjectName("primary"); run.clicked.connect(self._request); layout.addWidget(run); layout.addStretch()
        self.auto_max.toggled.connect(self._auto_changed); self._auto_changed(False); self.set_configuration("direct")

    def set_configuration(self, configuration: str, pump_max_rpm: float = 3000, pump_limit_enabled: bool = True) -> None:
        self.configuration = configuration; self.pump_max_rpm = pump_max_rpm
        self.pump_limit.setText(f"{pump_max_rpm:.0f} pump RPM" if pump_limit_enabled else "DISABLED in Machine Configuration")
        through_cvt = configuration == "cvt"
        self.direction.setEnabled(through_cvt)
        self.direction.setCurrentIndex(self.direction.findData("both" if through_cvt else "up"))
        self.subtitle.setText("CVT downstream-output power sweep; upshift, downshift, or paired curves." if through_cvt else "Direct-engine power sweep; engine speed may be derived from pump speed according to the selected setup.")
        self.note.setText("Peak Power always uses the flow valve for RPM control; the pressure valve remains raw 0 for the entire test. Auto-max detects a plateau only at normalized 100% flow, and the machine pump limit remains the hard ceiling.")

    def _auto_changed(self, checked: bool) -> None:
        self.end_rpm.setEnabled(not checked)

    def _request(self) -> None:
        self.run_requested.emit({"test_type":"peak_power", "name":self.name.text().strip() or "Peak power sweep", "start_rpm":self.start_rpm.value(), "end_rpm":self.end_rpm.value(), "sweep_duration_s":self.duration.value(), "direction":str(self.direction.currentData()), "throttle_pct":self.throttle.value(), "auto_max":self.auto_max.isChecked(), "rpm_rise_threshold":self.rise_threshold.value(), "plateau_dwell_s":self.plateau_dwell.value(), "overall_timeout_s":self.overall_timeout.value(), "saturation_enabled":self.saturation_enabled.isChecked(), "saturation_tolerance":self.saturation_tolerance.value(), "saturation_timeout_s":self.saturation_timeout.value(), "actuator":"flow", "target_type":"engine_rpm"})


class _ProfilePage(QWidget):
    run_requested = Signal(dict)
    parser = None

    def __init__(self, library: ProfileLibrary, title: str, subtitle: str, button_text: str) -> None:
        super().__init__(); self.library = library; self.current_profile = None; self.entries: list[ProfileEntry] = []
        layout = QVBoxLayout(self); heading, detail = _heading(title, subtitle); layout.addWidget(heading); layout.addWidget(detail)
        box = QGroupBox("Saved CSV profiles"); row = QHBoxLayout(box)
        self.combo = QComboBox(); self.combo.setMinimumWidth(320)
        for text, slot in (("Import new…", self._import), ("Export…", self._export), ("Delete", self._delete)):
            button = QPushButton(text); button.clicked.connect(slot); row.addWidget(button) if text != "Import new…" else None
            if text == "Import new…": self.import_button = button
        row.insertWidget(0, self.combo, 1); row.insertWidget(1, self.import_button)
        layout.addWidget(box)
        self.summary = QLabel(); self.summary.setObjectName("muted"); self.summary.setWordWrap(True); layout.addWidget(self.summary)
        self.preview = QTableWidget(0, 5); self.preview.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers); self.preview.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch); layout.addWidget(self.preview)
        self.saturation_enabled, self.saturation_tolerance, self.saturation_timeout = _saturation_controls(layout)
        run = QPushButton(button_text); run.setObjectName("primary"); run.clicked.connect(self._request); layout.addWidget(run)
        self.combo.currentIndexChanged.connect(self._load); self.refresh()

    def refresh(self, select_path: Path | None = None) -> None:
        self.entries = self.library.entries(); self.combo.blockSignals(True); self.combo.clear()
        selected = 0
        for index, entry in enumerate(self.entries):
            self.combo.addItem(f"{entry.name}{'  [built-in]' if entry.builtin else ''}")
            if select_path and entry.path == select_path: selected = index
        self.combo.blockSignals(False)
        if self.entries: self.combo.setCurrentIndex(selected); self._load(selected)
        else: self.current_profile = None; self.summary.setText("No valid profiles are available.")

    def _load(self, index: int) -> None:
        if not 0 <= index < len(self.entries): return
        try: self.current_profile = self.library.parser(self.entries[index].path); self._show_profile()
        except ValueError as exc: self.current_profile = None; self.summary.setText(f"Invalid profile: {exc}")

    def _import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Import CSV profile", "", "CSV files (*.csv)")
        if not path: return
        try: entry = self.library.import_file(path); self.refresh(entry.path)
        except (OSError, ValueError) as exc: QMessageBox.warning(self, "Profile import failed", str(exc))

    def _export(self) -> None:
        if not self.entries: return
        entry = self.entries[self.combo.currentIndex()]
        path, _ = QFileDialog.getSaveFileName(self, "Export CSV profile", entry.path.name, "CSV files (*.csv)")
        if path:
            try: self.library.export(entry, path)
            except OSError as exc: QMessageBox.warning(self, "Export failed", str(exc))

    def _delete(self) -> None:
        if not self.entries: return
        entry = self.entries[self.combo.currentIndex()]
        if entry.builtin: QMessageBox.information(self, "Protected profile", "Bundled profiles cannot be deleted. Export one, edit it externally, then import the new file."); return
        if QMessageBox.question(self, "Delete profile", f'Delete “{entry.name}”?') == QMessageBox.StandardButton.Yes:
            try: self.library.delete(entry); self.refresh()
            except (OSError, ValueError) as exc: QMessageBox.warning(self, "Delete failed", str(exc))


class CharacterizationPage(_ProfilePage):
    def __init__(self, library: ProfileLibrary) -> None:
        super().__init__(library, "Efficiency & Characterization", "Fully autonomous, CSV-defined CVT behavior testing. Pressure control is retained throughout every run.", "Review and run characterization profile")
        self.map_path = QLineEdit(); self.map_path.setReadOnly(True); self.map_path.setPlaceholderText("Optional engine input-power map CSV")
        choose = QPushButton("Select map…"); choose.clicked.connect(self._choose_map)
        row = QHBoxLayout(); row.addWidget(self.map_path); row.addWidget(choose); self.layout().insertLayout(self.layout().count()-1, row)

    def _choose_map(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Select engine power map", "", "CSV files (*.csv)")
        if path: self.map_path.setText(path)

    def _show_profile(self) -> None:
        p: CharacterizationProfile = self.current_profile
        self.summary.setText(f"{len(p.stages)} expanded stages · {p.duration_s:.1f} s recorded plan · actuator {p.actuator} · target {p.target_type} · {p.units}. Automatic startup and shutdown ramps are added and are not recorded.")
        self.preview.setHorizontalHeaderLabels(["Stage", "Target", "Transition", "Hold", "Throttle"]); self.preview.setRowCount(min(100, len(p.stages)))
        for row, s in enumerate(p.stages[:100]):
            for col, value in enumerate((s.stage, f"{s.target:g} N·m", f"{s.transition_s:g} s", f"{s.hold_s:g} s", f"{s.throttle_pct:g}%")): self.preview.setItem(row,col,QTableWidgetItem(str(value)))

    def _request(self) -> None:
        if not self.current_profile: self.summary.setText("Select a valid profile first."); return
        p = self.current_profile
        self.run_requested.emit({"test_type":"cvt_characterization", "name":f"CVT characterization — {p.name}", "duration_s":p.duration_s, "profile_path":p.source_path, "profile":p, "actuator":p.actuator, "target_type":p.target_type, "units":p.units, "engine_map_csv":self.map_path.text(), "saturation_enabled":self.saturation_enabled.isChecked(), "saturation_tolerance":self.saturation_tolerance.value(), "saturation_timeout_s":self.saturation_timeout.value()})


class TrackSimulationPage(_ProfilePage):
    def __init__(self, library: ProfileLibrary) -> None:
        super().__init__(library, "Track Simulation", "Long-duration CSV replay with throttle plus torque, engine-, pump-, or wheel-speed targets.", "Review and start track replay")

    def _show_profile(self) -> None:
        p: TrackProfile = self.current_profile
        self.summary.setText(f"{len(p.points)} points · {p.duration_s:.1f} s · actuator {p.actuator} · target {p.target_type} · {p.units}. The chosen actuator never changes during the replay.")
        self.preview.setHorizontalHeaderLabels(["Time", "Target", "Throttle", "Reference fields", ""]); self.preview.setRowCount(min(100, len(p.points)))
        for row, point in enumerate(p.points[:100]):
            for col, value in enumerate((f"{point.time_s:g} s", f"{point.target:g}", f"{point.throttle_pct:g}%", ", ".join(point.extras), "")): self.preview.setItem(row,col,QTableWidgetItem(str(value)))

    def _request(self) -> None:
        if not self.current_profile: self.summary.setText("Select a valid profile first."); return
        p = self.current_profile
        self.run_requested.emit({"test_type":"track_simulation", "name":f"Track replay — {p.name}", "duration_s":p.duration_s, "profile_path":p.source_path, "profile":p, "actuator":p.actuator, "target_type":p.target_type, "units":p.units, "saturation_enabled":self.saturation_enabled.isChecked(), "saturation_tolerance":self.saturation_tolerance.value(), "saturation_timeout_s":self.saturation_timeout.value()})


class EngineeringPage(QWidget):
    manual_changed = Signal(dict); manual_enable_requested = Signal(bool, dict)
    def __init__(self) -> None:
        super().__init__(); self._live=False; self._update_timer=QTimer(self); self._update_timer.setSingleShot(True); self._update_timer.setInterval(75); self._update_timer.timeout.connect(self._emit_live_values)
        layout=QVBoxLayout(self); title,subtitle=_heading("Engineering / Manual","Live outputs apply immediately while enabled. Calibration and tuning are under Machine Configuration."); layout.addWidget(title); layout.addWidget(subtitle)
        self.live_banner=QLabel("MANUAL CONTROL DISABLED"); self.live_banner.setObjectName("manualBanner"); layout.addWidget(self.live_banner)
        split=QSplitter(Qt.Orientation.Vertical); split.setChildrenCollapsible(False)
        control=QGroupBox("Manual control"); box=QVBoxLayout(control); top=QHBoxLayout(); self.enable=QCheckBox("Enable live manual control"); self.control_mode=QComboBox()
        for label,data in (("Direct PWM outputs","direct"),("Engine RPM target","engine_rpm"),("Pump RPM target","pump_rpm"),("Torque target","torque")): self.control_mode.addItem(label,data)
        self.actuator=QComboBox(); self.actuator.addItem("Pressure valve","pressure"); self.actuator.addItem("Flow valve","flow")
        top.addWidget(self.enable); top.addStretch(); top.addWidget(QLabel("Mode")); top.addWidget(self.control_mode); top.addWidget(QLabel("Actuator")); top.addWidget(self.actuator); box.addLayout(top)
        self.stack=QStackedWidget(); direct=QWidget(); form=QFormLayout(direct); self.flow=SliderSpinControl(0,100,0," %"); self.brake=SliderSpinControl(0,100,100," %"); self.throttle=SliderSpinControl(0,100,0," %"); form.addRow("Flow output",self.flow); form.addRow("Pressure / braking",self.brake); form.addRow("Throttle",self.throttle); self.stack.addWidget(direct)
        self.targets=[]
        for label,minimum,maximum,default,suffix in (("Engine speed",0,10000,2000," RPM"),("Pump speed",0,10000,1200," RPM"),("Torque",0,1000,25," N·m")):
            widget=QWidget(); target_form=QFormLayout(widget); target=SliderSpinControl(minimum,maximum,default,suffix); throttle=SliderSpinControl(0,100,40," %"); target_form.addRow(f"Target {label.lower()}",target); target_form.addRow("Throttle output",throttle); self.targets.append((target,throttle)); self.stack.addWidget(widget)
        box.addWidget(self.stack); split.addWidget(control)
        telemetry=QGroupBox("Live engineering telemetry"); grid=QGridLayout(telemetry)
        labels=(("engine_sensor_rpm","Physical engine sensor"),("engine_rpm","Selected engine speed"),("derived_engine_rpm","Derived engine speed"),("rpm_disagreement","RPM-source difference"),("pump_rpm","Pump speed"),("wheel_rpm","Calculated wheel speed"),("cvt_ratio","Calculated CVT ratio"),("torque","Measured torque"),("power","Output power"),("flow_duty","Flow raw duty"),("flow_voltage","Flow output voltage"),("pressure_duty","Pressure raw duty"),("pressure_voltage","Pressure output voltage"),("brake_load","Pressure command"),("throttle","Throttle command"),("servo_pulse","Servo pulse width"),("servo_duty","Servo duty cycle"),("active_target","Active target"),("control_error","Control error"),("controller","Selected PID bank"))
        self.telemetry_values={}
        for index,(key,text) in enumerate(labels):
            row,pair=divmod(index,2); name=QLabel(text); name.setObjectName("muted"); value=QLabel("—"); value.setObjectName("engineeringValue"); self.telemetry_values[key]=value; grid.addWidget(name,row,pair*2); grid.addWidget(value,row,pair*2+1)
        split.addWidget(telemetry); split.setSizes([320,420]); layout.addWidget(split,1)
        self.enable.toggled.connect(lambda enabled:self.manual_enable_requested.emit(enabled,self.values())); self.control_mode.currentIndexChanged.connect(self._mode_changed); self.actuator.currentIndexChanged.connect(self._schedule_emit)
        for item in (self.flow,self.brake,self.throttle,*[x for pair in self.targets for x in pair]): item.value_changed.connect(self._schedule_emit)
        self._mode_changed()
    @property
    def is_live(self): return self._live
    def set_live_enabled(self,enabled):
        self._live=enabled; self.enable.blockSignals(True); self.enable.setChecked(enabled); self.enable.blockSignals(False); self.live_banner.setText("LIVE MANUAL CONTROL — CHANGES APPLY IMMEDIATELY" if enabled else "MANUAL CONTROL DISABLED")
    def values(self):
        mode=str(self.control_mode.currentData()); index=max(0,self.control_mode.currentIndex()-1); target,throttle=self.targets[index] if mode!="direct" else (None,None)
        return {"control_mode":mode,"actuator":str(self.actuator.currentData()),"flow":self.flow.value(),"brake":self.brake.value(),"throttle":self.throttle.value(),"target":target.value() if target else 0,"closed_throttle":throttle.value() if throttle else 0}
    def _mode_changed(self,*_): self.stack.setCurrentIndex(self.control_mode.currentIndex()); self.actuator.setEnabled(self.control_mode.currentData()!="direct"); self._schedule_emit()
    def _schedule_emit(self,*_):
        if self._live:self._update_timer.start()
    def _emit_live_values(self):
        if self._live:self.manual_changed.emit(self.values())
    def show_telemetry(self,values):
        for key,label in self.telemetry_values.items():label.setText(values.get(key,"—"))


class ResultsPage(QWidget):
    test_selected=Signal(int); export_requested=Signal(int,str)
    def __init__(self):
        super().__init__(); layout=QVBoxLayout(self); title,subtitle=_heading("Results","Read-only SQLite test history. Export enriched telemetry, calculated summary, or both."); layout.addWidget(title); layout.addWidget(subtitle)
        self.table=QTableWidget(0,5); self.table.setHorizontalHeaderLabels(["ID","Test","Type","Status","Started"]); self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers); self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows); self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection); self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch); self.table.itemSelectionChanged.connect(self._selected); self.table.doubleClicked.connect(self._selected); layout.addWidget(self.table)
        self.plot=CurvePlot(); self.plot_secondary=CurvePlot(); layout.addWidget(self.plot); layout.addWidget(self.plot_secondary)
        row=QHBoxLayout(); row.addStretch(); self.buttons=[]
        for text,kind in (("Export telemetry","telemetry"),("Export summary","summary"),("Export both","both")):
            button=QPushButton(text); button.setEnabled(False); button.clicked.connect(lambda checked=False,k=kind:self._export(k)); row.addWidget(button); self.buttons.append(button)
        layout.addLayout(row)
    def set_tests(self,rows):
        self.table.setRowCount(len(rows))
        for ri,row in enumerate(rows):
            try: started=datetime.fromisoformat(row["started_utc"]).astimezone().strftime("%Y-%m-%d %H:%M:%S")
            except ValueError: started=row["started_utc"]
            for ci,value in enumerate((row["id"],row["name"],row["test_type"],row["status"],started)): self.table.setItem(ri,ci,QTableWidgetItem(str(value)))
    def _selected(self,*_):
        rows=self.table.selectionModel().selectedRows(); [button.setEnabled(bool(rows)) for button in self.buttons]
        if rows:self.test_selected.emit(int(self.table.item(rows[0].row(),0).text()))
    def _export(self,kind):
        rows=self.table.selectionModel().selectedRows()
        if rows:self.export_requested.emit(int(self.table.item(rows[0].row(),0).text()),kind)
