from __future__ import annotations

from dataclasses import replace

from PySide6.QtWidgets import (QCheckBox,QComboBox,QDialog,QDialogButtonBox,QDoubleSpinBox,QFileDialog,QFormLayout,QGroupBox,QHBoxLayout,QLabel,QLineEdit,QMessageBox,QPushButton,QScrollArea,QSpinBox,QTabWidget,QVBoxLayout,QWidget)

from settings import MachineParameters, PID_BANK_NAMES, PidTuning, Preferences, SetupProfile


def _double(value, minimum=0.0, maximum=100000.0, decimals=3, suffix=""):
    field=QDoubleSpinBox(); field.setRange(minimum,maximum); field.setDecimals(decimals); field.setValue(value); field.setSuffix(suffix); field.setMinimumWidth(170); return field


class RgbEditor(QWidget):
    def __init__(self,value,parent=None):
        super().__init__(parent); row=QHBoxLayout(self); row.setContentsMargins(0,0,0,0); self.fields=[]
        for label,component in zip(("R","G","B"),value):
            field=QSpinBox(); field.setRange(0,255); field.setValue(int(component)); field.setPrefix(label+" "); field.valueChanged.connect(self._swatch); self.fields.append(field); row.addWidget(field)
        self.sample=QLabel(); self.sample.setFixedSize(42,26); row.addWidget(self.sample); self._swatch()
    def value(self): return tuple(field.value() for field in self.fields)
    def _swatch(self):
        r,g,b=self.value(); self.sample.setStyleSheet(f"background:rgb({r},{g},{b});border:1px solid #777;border-radius:4px")


class OptionsDialog(QDialog):
    def __init__(self,p:Preferences,parent=None):
        super().__init__(parent); self.original=p; self.setWindowTitle("User Preferences"); self.setMinimumWidth(620); layout=QVBoxLayout(self); form=QFormLayout()
        self.units=QComboBox(); self.units.addItem("Metric (N·m, kW)","metric"); self.units.addItem("Imperial (lb·ft, hp)","imperial"); self.units.addItem("Metric and imperial","both"); self.units.setCurrentIndex(max(0,self.units.findData(p.unit_system)))
        self.simulator=QCheckBox("Use offline simulator instead of Bluetooth"); self.simulator.setChecked(p.simulator_enabled)
        self.export_dir=QLineEdit(p.export_directory); browse=QPushButton("Browse…"); browse.clicked.connect(self._browse); row=QHBoxLayout(); row.addWidget(self.export_dir); row.addWidget(browse)
        self.nominal=_double(p.nominal_valve_voltage,1,60,2," V")
        self.accent=RgbEditor(p.accent_rgb); self.background=RgbEditor(p.background_rgb); self.panel=RgbEditor(p.panel_rgb); self.warning=RgbEditor(p.warning_rgb)
        for label,widget in (("Display units",self.units),("Connection",self.simulator),("CSV export folder",row),("Nominal valve supply",self.nominal),("Accent RGB",self.accent),("Background RGB",self.background),("Panel/card RGB",self.panel),("Warning/E-stop RGB",self.warning)): form.addRow(label,widget)
        layout.addLayout(form); note=QLabel("Voltage values in Engineering are calculated from the nominal supply and raw PWM; they are not measured voltages."); note.setWordWrap(True); note.setObjectName("muted"); layout.addWidget(note)
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Save|QDialogButtonBox.StandardButton.Cancel); buttons.accepted.connect(self.accept); buttons.rejected.connect(self.reject); layout.addWidget(buttons)
    def _browse(self):
        path=QFileDialog.getExistingDirectory(self,"Select export directory",self.export_dir.text())
        if path:self.export_dir.setText(path)
    def result_preferences(self): return replace(self.original,unit_system=str(self.units.currentData()),simulator_enabled=self.simulator.isChecked(),export_directory=self.export_dir.text().strip() or self.original.export_directory,nominal_valve_voltage=self.nominal.value(),accent_rgb=self.accent.value(),background_rgb=self.background.value(),panel_rgb=self.panel.value(),warning_rgb=self.warning.value())


class SetupProfileDialog(QDialog):
    def __init__(self,p:SetupProfile,parent=None,*,title="Configuration Setup"):
        super().__init__(parent); self.profile_id=p.profile_id; self.setWindowTitle(title); self.resize(760,800)
        outer=QVBoxLayout(self); scroll=QScrollArea(); scroll.setWidgetResizable(True); body=QWidget(); layout=QVBoxLayout(body); scroll.setWidget(body); outer.addWidget(scroll)
        calc=QGroupBox("Calculation & safety parameters"); form=QFormLayout(calc)
        self.name=QLineEdit(p.name); self.configuration=QComboBox(); self.configuration.addItem("Direct engine","direct"); self.configuration.addItem("Engine through CVT","cvt"); self.configuration.setCurrentIndex(max(0,self.configuration.findData(p.configuration)))
        self.direct_source=QComboBox(); self.direct_source.addItem("Derived from pump sensor","derived"); self.direct_source.addItem("Physical engine sensor","sensor"); self.direct_source.setCurrentIndex(max(0,self.direct_source.findData(p.direct_rpm_source)))
        self.engine_to_pump=_double(p.engine_to_pump_ratio,.0001,1000,6); self.output_to_pump=_double(p.cvt_output_to_pump_ratio,.0001,1000,6); self.engine_ppr=_double(p.engine_pulses_per_rev,.001,1000,4)
        self.closed=_double(p.throttle_closed_us,1,10000,1," µs"); self.full=_double(p.throttle_full_us,1,10000,1," µs"); self.frequency=_double(p.throttle_frequency_hz,1,1000,1," Hz")
        self.engine_limit=QCheckBox("Enable engine overspeed protection"); self.engine_limit.setChecked(p.engine_overspeed_enabled); self.engine_max=_double(p.engine_overspeed_rpm,1,100000,0," RPM")
        self.idle=_double(p.idle_rpm,1,100000,0," RPM"); self.stall=_double(p.stall_warning_rpm,1,100000,0," RPM"); self.recovery=_double(p.recovery_rpm,1,100000,0," RPM"); self.recovery_timeout=_double(p.recovery_timeout_s,.1,300,1," s")
        self.expected_min=_double(p.expected_cvt_min_ratio,.001,100,3); self.expected_max=_double(p.expected_cvt_max_ratio,.001,100,3)
        self.vehicle_enabled=QCheckBox("Enable vehicle/wheel calculations"); self.vehicle_enabled.setChecked(p.vehicle_data_enabled); self.final_drive=_double(p.final_drive_ratio,.001,1000,4); self.tire=_double(p.tire_diameter_in,.1,100,2," in")
        rows=(("Setup name",self.name),("Configuration",self.configuration),("Direct-engine RPM source",self.direct_source),("Pump RPM / engine RPM",self.engine_to_pump),("Pump RPM / CVT-secondary RPM",self.output_to_pump),("Engine sensor pulses/rev",self.engine_ppr),("Closed-throttle pulse",self.closed),("Full-throttle pulse",self.full),("Servo frequency",self.frequency),("Engine overspeed",self.engine_limit),("Engine overspeed limit",self.engine_max),("Idle threshold",self.idle),("Stall warning threshold",self.stall),("Recovery threshold",self.recovery),("Recovery timeout",self.recovery_timeout),("Expected CVT high ratio (engine/secondary)",self.expected_min),("Expected CVT low ratio (engine/secondary)",self.expected_max),("Vehicle calculations",self.vehicle_enabled),("Final drive ratio",self.final_drive),("Tire diameter",self.tire))
        for label,widget in rows:form.addRow(label,widget)
        layout.addWidget(calc)
        docs=QGroupBox("Documentation only — not used in calculations"); dform=QFormLayout(docs)
        self.doc={}
        for key,label in (("cvt_primary","Primary model"),("cvt_primary_spring","Primary spring"),("cvt_flyweights","Flyweights"),("cvt_secondary","Secondary model"),("cvt_secondary_spring","Secondary spring"),("cvt_helix","Helix"),("cvt_preload","Secondary preload"),("cvt_belt","Belt part number"),("cvt_belt_notes","Belt width/condition"),("notes","General notes")):
            field=QLineEdit(getattr(p,key)); self.doc[key]=field; dform.addRow(label,field)
        layout.addWidget(docs); layout.addStretch()
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Save|QDialogButtonBox.StandardButton.Cancel); buttons.accepted.connect(self._accept); buttons.rejected.connect(self.reject); outer.addWidget(buttons)
        self.configuration.currentIndexChanged.connect(self._visibility); self.vehicle_enabled.toggled.connect(self._visibility); self._visibility()
    def _visibility(self,*_):
        cvt=self.configuration.currentData()=="cvt"; self.direct_source.setVisible(not cvt); self.engine_to_pump.setVisible(not cvt); self.output_to_pump.setVisible(cvt); self.expected_min.setVisible(cvt); self.expected_max.setVisible(cvt)
        for field in self.doc.values():field.setEnabled(cvt or field is self.doc["notes"])
        self.final_drive.setEnabled(cvt and self.vehicle_enabled.isChecked()); self.tire.setEnabled(cvt and self.vehicle_enabled.isChecked())
    def _accept(self):
        try:self.result_profile(); self.accept()
        except ValueError as exc:QMessageBox.warning(self,"Invalid setup",str(exc))
    def result_profile(self):
        result=SetupProfile(profile_id=self.profile_id,name=self.name.text().strip(),configuration=str(self.configuration.currentData()),direct_rpm_source=str(self.direct_source.currentData()),engine_to_pump_ratio=self.engine_to_pump.value(),cvt_output_to_pump_ratio=self.output_to_pump.value(),vehicle_data_enabled=self.vehicle_enabled.isChecked(),final_drive_ratio=self.final_drive.value(),tire_diameter_in=self.tire.value(),engine_pulses_per_rev=self.engine_ppr.value(),throttle_closed_us=self.closed.value(),throttle_full_us=self.full.value(),throttle_frequency_hz=self.frequency.value(),engine_overspeed_enabled=self.engine_limit.isChecked(),engine_overspeed_rpm=self.engine_max.value(),idle_rpm=self.idle.value(),stall_warning_rpm=self.stall.value(),recovery_rpm=self.recovery.value(),recovery_timeout_s=self.recovery_timeout.value(),expected_cvt_min_ratio=self.expected_min.value(),expected_cvt_max_ratio=self.expected_max.value(),**{k:v.text().strip() for k,v in self.doc.items()}); result.validate(); return result


class MachineParametersDialog(QDialog):
    def __init__(self,m:MachineParameters,parent=None):
        super().__init__(parent); self.original=m; self.setWindowTitle("Machine Configuration"); self.resize(780,760); layout=QVBoxLayout(self); tabs=QTabWidget(); layout.addWidget(tabs)
        general=QWidget(); form=QFormLayout(general); self.flow_min=_double(m.flow_pwm_min_pct,0,99,2," % raw"); self.flow_max=_double(m.flow_pwm_max_pct,0,99,2," % raw"); self.press_min=_double(m.pressure_pwm_min_pct,0,99,2," % raw"); self.press_max=_double(m.pressure_pwm_max_pct,0,99,2," % raw"); self.pump_ppr=_double(m.pump_pulses_per_rev,.001,1000,4); self.pump_limit=QCheckBox("Enable hard pump overspeed protection"); self.pump_limit.setChecked(m.pump_limit_enabled); self.pump_max=_double(m.pump_max_rpm,1,100000,0," RPM"); self.scale=_double(m.load_cell_scale_factor,-10000000,10000000,4); self.filter=_double(m.torque_filter_alpha,.001,1,4)
        for label,widget in (("Flow active PWM minimum",self.flow_min),("Flow active PWM maximum",self.flow_max),("Pressure active PWM minimum",self.press_min),("Pressure active PWM maximum",self.press_max),("Pump pulses/rev",self.pump_ppr),("Pump overspeed",self.pump_limit),("Maximum pump RPM",self.pump_max),("Load-cell scale factor",self.scale),("Torque filter alpha",self.filter)):form.addRow(label,widget)
        note=QLabel("Raw duty 100% is prohibited. Disarmed, unused, E-stop, and faulted outputs are raw 0 with their enable pins disabled. Pressure UI 100% maps to maximum configured braking; flow UI 100% maps to the verified maximum-flow PWM."); note.setWordWrap(True); note.setObjectName("muted"); form.addRow(note); tabs.addTab(general,"Machine Parameters")
        self.pid_fields={}
        for key in PID_BANK_NAMES:
            widget=QWidget(); pform=QFormLayout(widget); bank=m.pid_banks[key]; enabled=QCheckBox("Validated and enabled for automatic tests"); enabled.setChecked(bank.enabled); p=_double(bank.p,0,10000,6); i=_double(bank.i,0,10000,6); d=_double(bank.d,0,10000,6); slew=_double(bank.output_slew_pct_s,.001,100000,2," %/s")
            for label,control in (("Availability",enabled),("P",p),("I",i),("D",d),("Output slew",slew)):pform.addRow(label,control)
            self.pid_fields[key]=(enabled,p,i,d,slew); tabs.addTab(widget,key.replace("_"," → ").title())
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Save|QDialogButtonBox.StandardButton.Cancel); buttons.accepted.connect(self.accept); buttons.rejected.connect(self.reject); layout.addWidget(buttons)
    def result_machine(self):
        banks={key:PidTuning(p.value(),i.value(),d.value(),slew.value(),enabled.isChecked()) for key,(enabled,p,i,d,slew) in self.pid_fields.items()}
        m=MachineParameters(self.flow_min.value(),self.flow_max.value(),self.press_min.value(),self.press_max.value(),self.pump_ppr.value(),self.pump_limit.isChecked(),self.pump_max.value(),self.scale.value(),self.filter.value(),banks); m.validate(); return m


class LoadCellCalibrationDialog(QDialog):
    def __init__(self,machine:MachineParameters,parent=None):
        super().__init__(parent); self.setWindowTitle("Load-Cell Calibration"); self.action=""; self.machine=machine; layout=QVBoxLayout(self); form=QFormLayout(); self.scale=_double(machine.load_cell_scale_factor,-10000000,10000000,4); self.filter=_double(machine.torque_filter_alpha,.001,1,4); form.addRow("HX711 scale factor",self.scale); form.addRow("Torque filter alpha",self.filter); layout.addLayout(form); row=QHBoxLayout(); row.addStretch()
        for text,action in (("Cancel","cancel"),("Tare now","tare"),("Save calibration","save")):
            button=QPushButton(text); button.clicked.connect(lambda checked=False,a=action:self._done(a)); row.addWidget(button)
        layout.addLayout(row)
    def _done(self,action): self.action=action; self.reject() if action=="cancel" else self.accept()
    def result_machine(self): return replace(self.machine,load_cell_scale_factor=self.scale.value(),torque_filter_alpha=self.filter.value())


# Backward-compatible name used by older imports; tuning now edits all six banks.
ControllerTuningDialog=MachineParametersDialog
