from __future__ import annotations

from dataclasses import asdict, replace
import json, math, time
from pathlib import Path

from PySide6.QtCore import Qt,QTimer
from PySide6.QtGui import QFont,QFontDatabase,QPixmap
from PySide6.QtWidgets import QApplication,QComboBox,QDialog,QFileDialog,QFrame,QHBoxLayout,QLabel,QMainWindow,QMenu,QMessageBox,QPushButton,QScrollArea,QSplitter,QStackedWidget,QToolButton,QVBoxLayout,QWidget

from analysis import behavior_summary,track_summary,common_summary
from controller import DynoController
from database import DynoDatabase
from engine_map import EnginePowerMap
from protocol import Config,ConfigFlag,ConfigKind,DynoMode,SystemState,Telemetry,TelemetryFlag
from run_plan import TestStage,peak_power_plan,total_duration
from settings import CHARACTERIZATION_PROFILE_DIR,PID_BANK_NAMES,TRACK_PROFILE_DIR,SettingsStore,SetupProfile
from track_profile import CharacterizationProfile,CharacterizationStage,ProfileLibrary,TrackProfile
from ui.dialogs import LoadCellCalibrationDialog,MachineParametersDialog,OptionsDialog,SetupProfileDialog
from ui.pages import CharacterizationPage,EngineeringPage,PeakPowerPage,ResultsPage,TrackSimulationPage
from ui.theme import build_theme
from ui.widgets import LivePlot,MetricCard


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__(); self.settings_store=SettingsStore(); p=self.settings_store.preferences; QApplication.instance().setStyleSheet(build_theme(p)); self.database=DynoDatabase(p.database_path); self.project_id=self.database.ensure_default_project(); self.controller=DynoController(p.simulator_enabled,self)
        base=Path(__file__).resolve().parent.parent; self.character_library=ProfileLibrary(base/"examples"/"characterization",CHARACTERIZATION_PROFILE_DIR,CharacterizationProfile.from_csv); self.track_library=ProfileLibrary(base/"examples"/"tracks",TRACK_PROFILE_DIR,TrackProfile.from_csv)
        self.active_test_id=None; self.active_parameters=None; self.active_profile=None; self.active_plan=[]; self.active_stage_index=0; self.test_started=0.; self.stage_started=0.; self.sample_index=0; self.active_engine_map=None; self.pending_start=None; self.pending_manual=None; self.temporary_profile=None; self.current_page_index=0; self.app_started=time.monotonic(); self.stall_started=None; self.saturation_started=None; self.plateau_started=None; self.last_auto_rpm=None; self.last_auto_time=None; self.auto_endpoint=None
        self.setWindowTitle("Western Dynamometer"); self.resize(p.window_width,p.window_height); self.setMinimumSize(980,620); self._build_ui(base); self._connect(); self._refresh_profiles(); self._update_units(); self._refresh_results(); self.run_timer=QTimer(self); self.run_timer.setInterval(100); self.run_timer.timeout.connect(self._advance_test)

    def _build_ui(self,base):
        root=QWidget(); self.setCentralWidget(root); outer=QHBoxLayout(root); outer.setContentsMargins(0,0,0,0); outer.setSpacing(0)
        sidebar=QFrame(); sidebar.setObjectName("sidebar"); sidebar.setFixedWidth(220); side=QVBoxLayout(sidebar); side.setContentsMargins(14,20,14,16)
        logo_path=base/"assets"/"logo.png"
        if logo_path.exists():
            logo=QLabel(); logo.setPixmap(QPixmap(str(logo_path)).scaled(185,105,Qt.AspectRatioMode.KeepAspectRatio,Qt.TransformationMode.SmoothTransformation)); logo.setAlignment(Qt.AlignmentFlag.AlignCenter); side.addWidget(logo)
        else:
            placeholder=QLabel("WESTERN DYNO"); placeholder.setObjectName("brand"); placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter); side.addWidget(placeholder)
        caption=QLabel("Dynamometer"); caption.setAlignment(Qt.AlignmentFlag.AlignCenter); caption.setObjectName("brand")
        font_path=base/"assets"/"fonts"/"Ethnocentric-Regular.otf"
        if font_path.exists():
            font_id=QFontDatabase.addApplicationFont(str(font_path)); families=QFontDatabase.applicationFontFamilies(font_id)
            if families:caption.setFont(QFont(families[0],13))
        side.addWidget(caption); side.addSpacing(18)
        self.stack=QStackedWidget(); self.peak_page=PeakPowerPage(); self.character_page=CharacterizationPage(self.character_library); self.track_page=TrackSimulationPage(self.track_library); self.engineering_page=EngineeringPage(); self.results_page=ResultsPage(); self.nav_buttons=[]
        for index,(label,page) in enumerate((("Peak Power",self.peak_page),("CVT Characterization",self.character_page),("Track Simulation",self.track_page),("Engineering / Manual",self.engineering_page),("Results",self.results_page))):
            scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded); scroll.setWidget(page); self.stack.addWidget(scroll); button=QPushButton(label); button.setObjectName("nav"); button.setCheckable(True); button.clicked.connect(lambda checked=False,i=index:self._navigate(i)); side.addWidget(button); self.nav_buttons.append(button)
        self.nav_buttons[0].setChecked(True); side.addStretch(); self.settings_button=QToolButton(); self.settings_button.setText("Settings"); self.settings_button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup); menu=QMenu(self); user=menu.addAction("User Preferences…"); machine_menu=menu.addMenu("Machine Configuration"); machine=machine_menu.addAction("Parameters & Controller Tuning…"); calibration=machine_menu.addAction("Load-Cell Calibration / Tare…"); user.triggered.connect(self._show_options); machine.triggered.connect(self._show_machine); calibration.triggered.connect(self._show_load_cell); self.settings_button.setMenu(menu); side.addWidget(self.settings_button); outer.addWidget(sidebar)
        content=QVBoxLayout(); content.setContentsMargins(18,14,18,14); content.setSpacing(12); top=QFrame(); top.setObjectName("topbar"); row=QHBoxLayout(top); self.connection_dot=QLabel("●"); self.connection_text=QLabel("Not connected"); self.connect_button=QPushButton("Connect"); row.addWidget(self.connection_dot); row.addWidget(self.connection_text); row.addWidget(self.connect_button); row.addStretch(); row.addWidget(QLabel("Setup")); self.profile_combo=QComboBox(); self.profile_combo.setMinimumWidth(245); self.setup_status=QLabel(); self.setup_status.setObjectName("muted"); self.manage=QToolButton(); self.manage.setText("Manage"); self.manage.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup); sm=QMenu(self); self.new_action=sm.addAction("Create New Setup…"); self.edit_action=sm.addAction("Edit Selected Setup…"); self.override_action=sm.addAction("Create Temporary Override…"); sm.addSeparator(); self.delete_action=sm.addAction("Delete Selected Setup…"); self.new_action.triggered.connect(self._new_setup); self.edit_action.triggered.connect(self._edit_setup); self.override_action.triggered.connect(self._override_setup); self.delete_action.triggered.connect(self._delete_setup); self.manage.setMenu(sm); row.addWidget(self.profile_combo); row.addWidget(self.setup_status); row.addWidget(self.manage); content.addWidget(top)
        metrics=QHBoxLayout(); self.engine_metric=MetricCard("Engine speed","RPM"); self.pump_metric=MetricCard("Pump speed","RPM"); self.torque_metric=MetricCard("Torque","N·m"); self.power_metric=MetricCard("Output power","kW")
        for card in (self.engine_metric,self.pump_metric,self.torque_metric,self.power_metric):metrics.addWidget(card)
        content.addLayout(metrics); center=QSplitter(Qt.Orientation.Horizontal); center.setChildrenCollapsible(False); center.addWidget(self.stack); self.plot=LivePlot(); center.addWidget(self.plot); center.setStretchFactor(0,3); center.setStretchFactor(1,2); center.setSizes([850,520]); content.addWidget(center,1)
        safety=QFrame(); safety.setObjectName("safetybar"); sr=QHBoxLayout(safety); self.state_label=QLabel("STATE: UNKNOWN"); self.fault_label=QLabel("No telemetry"); self.progress_label=QLabel(); self.abort=QPushButton("ABORT TEST"); self.abort.setObjectName("disarm"); self.abort.setEnabled(False); self.reset=QPushButton("RESET / CLEAR"); self.disarm=QPushButton("DISARM"); self.disarm.setObjectName("disarm"); self.estop=QPushButton("EMERGENCY STOP"); self.estop.setObjectName("danger"); sr.addWidget(self.state_label); sr.addWidget(self.fault_label); sr.addStretch(); sr.addWidget(self.progress_label); sr.addWidget(self.abort); sr.addWidget(self.reset); sr.addWidget(self.disarm); sr.addWidget(self.estop); content.addWidget(safety); outer.addLayout(content,1)

    def _connect(self):
        self.connect_button.clicked.connect(self._toggle_connection); self.disarm.clicked.connect(self._disarm); self.estop.clicked.connect(self.controller.emergency_stop); self.reset.clicked.connect(self._reset_latch); self.abort.clicked.connect(lambda:self._finish_test("aborted")); self.profile_combo.currentIndexChanged.connect(self._profile_selected); self.controller.connection_changed.connect(self._connection_changed); self.controller.telemetry_changed.connect(self._telemetry); self.controller.config_changed.connect(self._config_changed); self.controller.error.connect(self._error); self.peak_page.run_requested.connect(self._review_test); self.character_page.run_requested.connect(self._review_test); self.track_page.run_requested.connect(self._review_test); self.engineering_page.manual_enable_requested.connect(self._manual_enable); self.engineering_page.manual_changed.connect(self._manual_command); self.results_page.test_selected.connect(self._show_result); self.results_page.export_requested.connect(self._export_test)

    def _navigate(self,index):
        if self.current_page_index==3 and index!=3 and self.engineering_page.is_live:self._disable_manual()
        self.current_page_index=index; self.stack.setCurrentIndex(index)
        for i,b in enumerate(self.nav_buttons):b.setChecked(i==index)
        if index==4:self._refresh_results()
    def _toggle_connection(self): self.controller.disconnect_dyno() if self.controller.connected else self.controller.connect_dyno()
    def _connection_changed(self,connected,message):
        self.connection_text.setText(message); self.connection_dot.setStyleSheet(f"color:{'#42c982' if connected else '#d64a57'};font-size:16pt"); self.connect_button.setText("Disconnect" if connected else "Connect")
        if not connected:
            self.engineering_page.set_live_enabled(False)
            if self.active_test_id is not None:self._finish_test("connection_lost")

    def _telemetry(self,v:Telemetry):
        profile=self.active_profile or self._current_profile(); now=time.monotonic()
        if v.engine_rpm<profile.stall_warning_rpm:
            self.stall_started=self.stall_started or now
        else:self.stall_started=None
        stalled=self.stall_started is not None and now-self.stall_started>=.5
        self.engine_metric.set_text("STALL" if stalled else f"{v.engine_rpm:,.0f}"); self.pump_metric.set_value(v.pump_rpm)
        units=self.settings_store.preferences.unit_system
        if units=="imperial":self.torque_metric.set_value(v.torque_nm*.737562,1); self.power_metric.set_value(v.power_kw*1.341022,2)
        elif units=="both":self.torque_metric.set_text(f"{v.torque_nm:.1f} / {v.torque_nm*.737562:.1f}"); self.power_metric.set_text(f"{v.power_kw:.2f} / {v.power_kw*1.341022:.2f}")
        else:self.torque_metric.set_value(v.torque_nm,1); self.power_metric.set_value(v.power_kw,2)
        self.state_label.setText(f"STATE: {v.state.name}"); self.fault_label.setText(" · ".join(v.faults) if v.faults else ("Anti-stall intervention" if v.flags&TelemetryFlag.ANTI_STALL else "Sensors / setup valid")); self.plot.append(now-self.app_started,v.engine_rpm,v.pump_rpm,v.target_engine_rpm or v.target_pump_rpm or v.target_torque_nm)
        calculated=self._calculated(v,profile); self._engineering(v,calculated,profile)
        if self.active_test_id is not None:
            elapsed=now-self.test_started; stage=self._stage_metadata(); self.database.append_telemetry(self.active_test_id,self.sample_index,elapsed,v,calculated,stage); self.sample_index+=1; self._check_saturation(v,calculated)
            if v.state in {SystemState.ESTOP,SystemState.FAULT}:self._finish_test(v.state.name.lower())

    def _calculated(self,v,p):
        derived=v.pump_rpm/p.engine_to_pump_ratio if p.configuration=="direct" and p.engine_to_pump_ratio else None; cvt_ratio=(v.engine_rpm*p.cvt_output_to_pump_ratio/v.pump_rpm) if p.configuration=="cvt" and v.pump_rpm>1 else None; wheel=(v.pump_rpm/p.cvt_output_to_pump_ratio/p.final_drive_ratio) if p.configuration=="cvt" and p.vehicle_data_enabled else None; target_type=(self.active_parameters or {}).get("target_type",""); target={"torque":v.target_torque_nm,"engine_rpm":v.target_engine_rpm,"pump_rpm":v.target_pump_rpm,"wheel_rpm":wheel}.get(target_type,0); measured={"torque":v.torque_nm,"engine_rpm":v.engine_rpm,"pump_rpm":v.pump_rpm,"wheel_rpm":wheel}.get(target_type,0); m=self.settings_store.machine; flow_norm=100*(v.flow_valve_duty_pct-m.flow_pwm_min_pct)/(m.flow_pwm_max_pct-m.flow_pwm_min_pct) if v.flow_valve_duty_pct else 0; pressure_norm=100*(m.pressure_pwm_max_pct-v.pressure_valve_duty_pct)/(m.pressure_pwm_max_pct-m.pressure_pwm_min_pct) if v.pressure_valve_duty_pct else 0; efficiency=None
        if self.active_engine_map and v.engine_rpm>0:
            input_kw=self.active_engine_map.power_at(v.engine_rpm)
            if input_kw is not None and input_kw>0:efficiency=100*v.power_kw/input_kw
        position=None if cvt_ratio is None else 100*(p.expected_cvt_max_ratio-cvt_ratio)/(p.expected_cvt_max_ratio-p.expected_cvt_min_ratio)
        return {"derived_engine_rpm":derived,"rpm_source_difference":abs(v.engine_sensor_rpm-derived) if derived is not None and v.engine_sensor_rpm>0 else None,"cvt_ratio":cvt_ratio,"cvt_position_pct":position,"wheel_rpm":wheel,"estimated_efficiency_pct":efficiency,"target_value":target,"target_error":None if measured is None else target-measured,"flow_command_pct":max(0,min(100,flow_norm)),"pressure_command_pct":max(0,min(100,pressure_norm)),"anti_stall_active":bool(v.flags&TelemetryFlag.ANTI_STALL)}

    def _engineering(self,v,c,p):
        nominal=self.settings_store.preferences.nominal_valve_voltage; pulse=p.throttle_closed_us+(p.throttle_full_us-p.throttle_closed_us)*v.throttle_pct/100; duty=pulse/(1_000_000/p.throttle_frequency_hz)*100; target_type=(self.active_parameters or {}).get("target_type",{DynoMode.TORQUE:"torque",DynoMode.RPM:"pump_rpm",DynoMode.ENGINE_RPM:"engine_rpm"}.get(v.mode,"direct")); actuator="pressure" if v.flags&TelemetryFlag.PRESSURE_ACTUATOR else "flow"; bank=f"{actuator} → {target_type.replace('_',' ')}" if target_type!="direct" else "Direct PWM"
        self.engineering_page.show_telemetry({"engine_sensor_rpm":f"{v.engine_sensor_rpm:,.0f} RPM","engine_rpm":f"{v.engine_rpm:,.0f} RPM","derived_engine_rpm":"—" if c["derived_engine_rpm"] is None else f"{c['derived_engine_rpm']:,.0f} RPM","rpm_disagreement":"—" if c["rpm_source_difference"] is None else f"{c['rpm_source_difference']:,.0f} RPM","pump_rpm":f"{v.pump_rpm:,.0f} RPM","wheel_rpm":"—" if c["wheel_rpm"] is None else f"{c['wheel_rpm']:,.1f} RPM","cvt_ratio":"—" if c["cvt_ratio"] is None else f"{c['cvt_ratio']:.3f}:1","torque":f"{v.torque_nm:.2f} N·m","power":f"{v.power_kw:.3f} kW","flow_duty":f"{v.flow_valve_duty_pct:.1f}% raw","flow_voltage":f"{nominal*v.flow_valve_duty_pct/100:.2f} V estimated","pressure_duty":f"{v.pressure_valve_duty_pct:.1f}% raw","pressure_voltage":f"{nominal*v.pressure_valve_duty_pct/100:.2f} V estimated","brake_load":f"{c['pressure_command_pct']:.1f}%","throttle":f"{v.throttle_pct:.1f}%","servo_pulse":f"{pulse:.0f} µs","servo_duty":f"{duty:.2f}%","active_target":f"{c['target_value']:.1f}","control_error":"—" if c["target_error"] is None else f"{c['target_error']:+.1f}","controller":bank})

    def _review_test(self,params):
        if self.active_test_id is not None:return self._error("A test is already running")
        if self.engineering_page.is_live:return self._error("Disable live manual control first")
        if not self.controller.connected or not self.controller.telemetry:return self._error("Connect and receive telemetry first")
        if self.controller.telemetry.state!=SystemState.DISARMED:return self._error("Dyno must report DISARMED")
        profile=self._current_profile()
        try:self._preflight(params,profile)
        except ValueError as exc:return self._error(str(exc))
        summary=f"Test: {params['name']}\nSetup: {profile.name} ({profile.configuration})\nActuator: {params['actuator']} — locked for entire run\nTarget: {params['target_type']}\nDuration: {params.get('duration_s',params.get('overall_timeout_s',0)):.1f} s\n\nThe active setup will be written and acknowledged immediately before arming. Confirm the test area is clear and the physical E-stop is accessible."
        if QMessageBox.question(self,"Confirm automatic test",summary,QMessageBox.StandardButton.Yes|QMessageBox.StandardButton.Cancel,QMessageBox.StandardButton.Cancel)==QMessageBox.StandardButton.Yes:
            self.pending_start=dict(params); self.progress_label.setText("Writing active setup before arm…"); self.controller.apply_config(Config.active_setup(profile))

    def _preflight(self,p,profile):
        if not self.settings_store.preferences.simulator_enabled and self.controller.telemetry.engine_rpm < profile.idle_rpm:
            raise ValueError(f"Engine is not at a stable operating speed (requires at least {profile.idle_rpm:.0f} RPM for this setup)")
        if p["test_type"]=="cvt_characterization" and profile.configuration!="cvt":raise ValueError("CVT characterization needs a setup whose Configuration is ‘Engine through CVT’. Use Manage → Create/Edit Setup and select it first.")
        if p["test_type"]=="peak_power":
            if not self.settings_store.machine.pid_banks["flow_engine_rpm"].enabled:raise ValueError("Enable and validate Flow → Engine RPM in Machine Configuration before Peak Power")
            if p["auto_max"] and not self.settings_store.machine.pump_limit_enabled:raise ValueError("Auto-max requires the global pump-RPM safety ceiling to be enabled in Machine Configuration")
        else:
            target="pump_rpm" if p["target_type"]=="wheel_rpm" else p["target_type"]; key=f"{p['actuator']}_{target}"
            if not self.settings_store.machine.pid_banks[key].enabled:raise ValueError(f"Enable and validate {key.replace('_',' → ',1).replace('_',' ')} in Machine Configuration")
        if p.get("engine_map_csv"):EnginePowerMap.from_csv(p["engine_map_csv"])
        if p["target_type"]=="wheel_rpm" and (profile.configuration!="cvt" or not profile.vehicle_data_enabled):raise ValueError("Wheel-RPM targets require a CVT setup with vehicle/wheel calculations enabled")

    def _config_changed(self,c):
        if c.kind==ConfigKind.ACTIVE_SETUP and self.pending_start is not None:
            params=self.pending_start; self.pending_start=None; self._begin_test(params)
        elif c.kind==ConfigKind.ACTIVE_SETUP and self.pending_manual is not None:
            values=self.pending_manual; self.pending_manual=None; self._send_manual(values); self.engineering_page.set_live_enabled(True); self.controller.arm()
        self.progress_label.setText(f"Configuration acknowledged: {c.kind.name}")

    def _begin_test(self,p):
        profile=replace(self._current_profile()); self.active_profile=profile; serial={k:v for k,v in p.items() if k!="profile"}; self.active_test_id=self.database.start_test(self.project_id,p["name"],p["test_type"],asdict(profile),serial); self.active_parameters=p; self.active_engine_map=EnginePowerMap.from_csv(p["engine_map_csv"]) if p.get("engine_map_csv") else None; self.active_plan=[]
        if p["test_type"]=="peak_power" and not p["auto_max"]:
            self.active_plan=peak_power_plan(p["start_rpm"],p["end_rpm"],p["sweep_duration_s"],p["direction"],profile.configuration=="cvt",p["throttle_pct"]); p["duration_s"]=total_duration(self.active_plan)
        elif p["test_type"]=="cvt_characterization":
            stages=list(p["profile"].stages); first=stages[0]; last=stages[-1]; self.active_plan=[CharacterizationStage("Automatic startup ramp",0,2,1,0,2,"automatic",0,False)]+stages+[CharacterizationStage("Automatic shutdown ramp",0,2,1,0,2,"automatic",0,False)]; p["duration_s"]=sum(s.transition_s+s.hold_s for s in self.active_plan)
        self.active_stage_index=0; self.test_started=self.stage_started=time.monotonic(); self.sample_index=0; self.saturation_started=self.plateau_started=self.last_auto_rpm=self.last_auto_time=None; self.auto_endpoint=None; self.profile_combo.setEnabled(False); self.manage.setEnabled(False); self.abort.setEnabled(True); self.plot.clear(); self._advance_test(); self.controller.arm(); self.run_timer.start()

    def _advance_test(self):
        if not self.active_parameters:return
        p=self.active_parameters; now=time.monotonic(); elapsed=now-self.test_started
        if elapsed>=float(p.get("overall_timeout_s",1e12)) and p.get("auto_max"):self.auto_endpoint="overall_timeout"; return self._finish_test("completed")
        if p["test_type"]=="track_simulation":
            if elapsed>=p["duration_s"]:return self._finish_test("completed")
            point=p["profile"].target_at(elapsed); self._command_target(p["target_type"],point.target,point.throttle_pct,p["actuator"]); self.progress_label.setText(f"Track · {elapsed:.1f}/{p['duration_s']:.1f} s"); return
        if p["test_type"]=="peak_power" and p["auto_max"]:
            target=p["start_rpm"]+elapsed*200; self._command_target("engine_rpm",target,p["throttle_pct"],"flow"); v=self.controller.telemetry
            if v:
                flow=self._calculated(v,self.active_profile)["flow_command_pct"]
                if self.last_auto_time is not None:
                    rise=(v.engine_rpm-self.last_auto_rpm)/max(.001,now-self.last_auto_time)
                    if flow>=99 and rise<=p["rpm_rise_threshold"]:self.plateau_started=self.plateau_started or now
                    else:self.plateau_started=None
                    if self.plateau_started and now-self.plateau_started>=p["plateau_dwell_s"]:
                        endpoint=v.engine_rpm; self.auto_endpoint=f"rpm_plateau_at_{endpoint:.0f}_engine_rpm"
                        if self.active_profile.configuration=="cvt" and p["direction"] in {"down","both"}:
                            duration=max(1.0,float(p.get("sweep_duration_s",30.0))); settle=max(2.0,min(8.0,duration*.2)); self.active_plan=[TestStage("Auto-max high-speed stabilization",endpoint,endpoint,settle,p["throttle_pct"],"hold",False),TestStage("CVT downshift peak-power sweep",endpoint,p["start_rpm"],duration,p["throttle_pct"],"downshift",True)]; self.active_stage_index=0; self.stage_started=now; p["auto_max"]=False; return
                        return self._finish_test("completed")
                self.last_auto_rpm=v.engine_rpm; self.last_auto_time=now
            self.progress_label.setText(f"Auto-max · {elapsed:.1f}/{p['overall_timeout_s']:.1f} s"); return
        if self.active_stage_index>=len(self.active_plan):return self._finish_test("completed")
        stage=self.active_plan[self.active_stage_index]; stage_elapsed=now-self.stage_started; duration=stage.duration_s if isinstance(stage,TestStage) else stage.transition_s+stage.hold_s
        if stage_elapsed>=duration:
            self.active_stage_index+=1; self.stage_started=now
            if self.active_stage_index>=len(self.active_plan):return self._finish_test("completed")
            stage=self.active_plan[self.active_stage_index]; stage_elapsed=0
        if isinstance(stage,TestStage):
            fraction=min(1,stage_elapsed/stage.duration_s); target=stage.start_rpm+(stage.end_rpm-stage.start_rpm)*fraction; self._command_target("engine_rpm",target,stage.throttle_pct,"flow")
        else:
            previous=self.active_plan[self.active_stage_index-1] if self.active_stage_index else None; start_target=previous.target if isinstance(previous,CharacterizationStage) else 0; start_throttle=previous.throttle_pct if isinstance(previous,CharacterizationStage) else 0; tf=1 if stage.transition_s<=0 else min(1,stage_elapsed/stage.transition_s); thf=1 if stage.throttle_transition_s<=0 else min(1,stage_elapsed/stage.throttle_transition_s); self._command_target("torque",start_target+(stage.target-start_target)*tf,start_throttle+(stage.throttle_pct-start_throttle)*thf,"pressure")
        self.progress_label.setText(f"Stage {self.active_stage_index+1}/{len(self.active_plan)} · {stage.name if isinstance(stage,TestStage) else stage.stage}")

    def _command_target(self,target_type,target,throttle,actuator):
        args={"mode":DynoMode.MANUAL,"pressure_actuator":actuator=="pressure","manual_throttle_override":True,"manual_throttle_pct":round(throttle),"manual_flow_pct":0,"manual_pressure_pct":0,"target_torque_nm":0.,"target_pump_rpm":0.,"target_engine_rpm":0.}
        if target_type=="torque":args.update(mode=DynoMode.TORQUE,target_torque_nm=target)
        elif target_type=="engine_rpm":args.update(mode=DynoMode.ENGINE_RPM,target_engine_rpm=target)
        else:
            if target_type=="wheel_rpm":target=target*self.active_profile.final_drive_ratio*self.active_profile.cvt_output_to_pump_ratio
            args.update(mode=DynoMode.RPM,target_pump_rpm=target)
        self.controller.update_command(**args)

    def _check_saturation(self,v,c):
        p=self.active_parameters
        if not p or not p.get("saturation_enabled"):return
        output=c["pressure_command_pct"] if p["actuator"]=="pressure" else c["flow_command_pct"]; saturated=output<=.5 or output>=99.5; outside=abs(c["target_error"] or 0)>p["saturation_tolerance"]
        if saturated and outside:self.saturation_started=self.saturation_started or time.monotonic()
        else:self.saturation_started=None
        if self.saturation_started and time.monotonic()-self.saturation_started>=p["saturation_timeout_s"]:
            self.controller.report_saturation_fault(); self._finish_test("target_saturation_fault")

    def _stage_metadata(self):
        p=self.active_parameters or {}; stage=self.active_plan[self.active_stage_index] if self.active_plan and self.active_stage_index<len(self.active_plan) else None
        if isinstance(stage,TestStage):name,direction,record=stage.name,stage.direction,stage.record
        elif isinstance(stage,CharacterizationStage):name,direction,record=stage.stage,stage.group,stage.record
        elif p.get("test_type")=="peak_power" and p.get("auto_max"):name,direction,record="Auto-max upshift","upshift",p.get("direction")!="down"
        else:name,direction,record="Track profile","profile",True
        return {"index":self.active_stage_index,"name":name,"direction":direction,"record":record,"actuator":p.get("actuator",""),"target_type":p.get("target_type","")}

    def _finish_test(self,status):
        if self.active_test_id is None:return
        test_id=self.active_test_id; self.run_timer.stop(); self.controller.disarm(); self.database.connection.commit(); rows=self.database.samples(test_id); kind=self.active_parameters["test_type"]
        results=behavior_summary(rows) if kind=="cvt_characterization" else track_summary(rows) if kind=="track_simulation" else common_summary(rows); results["stop_reason"]=self.auto_endpoint or status; results["test_throttle_pct"]=self.active_parameters.get("throttle_pct"); results["actuator_locked_for_run"]=self.active_parameters.get("actuator"); results["target_type"]=self.active_parameters.get("target_type")
        if kind=="track_simulation" and self.active_profile.vehicle_data_enabled and "wheel_revolutions" in results:results["distance_km"]=results["wheel_revolutions"]*math.pi*self.active_profile.tire_diameter_in*.0254/1000
        self.database.finish_test(test_id,status,results); self.active_test_id=None; self.active_parameters=None; self.active_profile=None; self.active_plan=[]; self.active_engine_map=None; self.profile_combo.setEnabled(self.temporary_profile is None); self.manage.setEnabled(True); self.abort.setEnabled(False); self.progress_label.setText(f"Last test: {status}"); self._refresh_results()

    def _manual_enable(self,enabled,values):
        if not enabled:return self._disable_manual()
        if self.active_test_id is not None or not self.controller.connected or not self.controller.telemetry or self.controller.telemetry.state!=SystemState.DISARMED:self.engineering_page.set_live_enabled(False); return self._error("Connect with the dyno DISARMED and no test running")
        if QMessageBox.question(self,"Enable live manual control","Changes to sliders and numeric fields will apply immediately. Continue?",QMessageBox.StandardButton.Yes|QMessageBox.StandardButton.Cancel)==QMessageBox.StandardButton.Yes:self.pending_manual=values; self.controller.apply_config(Config.active_setup(self._current_profile()))
        else:self.engineering_page.set_live_enabled(False)
    def _manual_command(self,values):
        if self.engineering_page.is_live:self._send_manual(values)
    def _send_manual(self,v):
        mode=v["control_mode"]; args={"pressure_actuator":v["actuator"]=="pressure","manual_flow_pct":0,"manual_pressure_pct":0,"manual_throttle_pct":v["closed_throttle"],"manual_throttle_override":True,"target_torque_nm":0.,"target_pump_rpm":0.,"target_engine_rpm":0.}
        if mode=="direct":args.update(mode=DynoMode.MANUAL,manual_flow_pct=v["flow"],manual_pressure_pct=v["brake"],manual_throttle_pct=v["throttle"],manual_throttle_override=False)
        elif mode=="engine_rpm":args.update(mode=DynoMode.ENGINE_RPM,target_engine_rpm=v["target"])
        elif mode=="pump_rpm":args.update(mode=DynoMode.RPM,target_pump_rpm=v["target"])
        else:args.update(mode=DynoMode.TORQUE,target_torque_nm=v["target"])
        self.controller.update_command(**args)
    def _disable_manual(self):self.engineering_page.set_live_enabled(False); self.controller.safe_manual()
    def _disarm(self):
        self._disable_manual()
        if self.active_test_id is not None:self._finish_test("disarmed")
        else:self.controller.disarm()
    def _reset_latch(self):
        if QMessageBox.question(self,"Reset / clear latch","The firmware will clear only if the physical E-stop is released, outputs are zero, shafts and torque are below safe thresholds, communications are valid, and the original fault condition is gone. It always returns DISARMED and never rearms.")==QMessageBox.StandardButton.Yes:self.controller.clear_latched_fault()

    def _show_machine(self):
        dialog=MachineParametersDialog(self.settings_store.machine,self)
        if dialog.exec()!=QDialog.DialogCode.Accepted:return
        try:machine=dialog.result_machine()
        except ValueError as exc:return self._error(str(exc))
        self.settings_store.machine=machine; self.settings_store.save(); self.peak_page.set_configuration(self._current_profile().configuration,machine.pump_max_rpm,machine.pump_limit_enabled)
        if self.controller.connected and self.controller.telemetry and self.controller.telemetry.state==SystemState.DISARMED:
            self.controller.apply_config(Config.machine(machine));
            for index,key in enumerate(PID_BANK_NAMES):self.controller.apply_config(Config.pid(index,machine.pid_banks[key]))
        else:QMessageBox.information(self,"Saved on desktop","Machine parameters were saved. Connect while DISARMED and reopen Machine Configuration to write them to ESP32 nonvolatile storage.")
    def _show_load_cell(self):
        dialog=LoadCellCalibrationDialog(self.settings_store.machine,self)
        if dialog.exec()!=QDialog.DialogCode.Accepted:return
        machine=dialog.result_machine(); machine.validate(); self.settings_store.machine=machine; self.settings_store.save()
        if not self.controller.connected or not self.controller.telemetry or self.controller.telemetry.state!=SystemState.DISARMED:return self._error("Connect with controller DISARMED to apply calibration or tare")
        config=Config.machine(machine); config=replace(config,action_flags=config.action_flags|(ConfigFlag.TARE_LOAD_CELL if dialog.action=="tare" else ConfigFlag(0))); self.controller.apply_config(config)
    def _show_options(self):
        dialog=OptionsDialog(self.settings_store.preferences,self)
        if dialog.exec()!=QDialog.DialogCode.Accepted:return
        old=self.settings_store.preferences.simulator_enabled; self.settings_store.preferences=dialog.result_preferences(); self.settings_store.save(); QApplication.instance().setStyleSheet(build_theme(self.settings_store.preferences)); self._update_units()
        if old!=self.settings_store.preferences.simulator_enabled:QMessageBox.information(self,"Restart required","Restart to switch between Bluetooth and the offline simulator.")

    def _current_profile(self):return self.temporary_profile or self.settings_store.profile(str(self.profile_combo.currentData() or self.settings_store.preferences.last_setup_id))
    def _refresh_profiles(self):
        current=self.settings_store.preferences.last_setup_id; self.profile_combo.blockSignals(True); self.profile_combo.clear()
        for p in self.settings_store.profiles:self.profile_combo.addItem(p.name,p.profile_id)
        self.profile_combo.setCurrentIndex(max(0,self.profile_combo.findData(current))); self.profile_combo.blockSignals(False); self._profile_selected()
    def _profile_selected(self,*_):
        if self.temporary_profile:return
        p=self._current_profile(); self.settings_store.preferences.last_setup_id=p.profile_id; self.settings_store.save(); self.setup_status.setText("CVT setup ✓" if p.configuration=="cvt" else "DIRECT ENGINE"); m=self.settings_store.machine; self.peak_page.set_configuration(p.configuration,m.pump_max_rpm,m.pump_limit_enabled)
    def _new_setup(self):
        d=SetupProfileDialog(replace(self._current_profile().clone(),name="New setup"),self,title="Create New Setup")
        if d.exec()==QDialog.DialogCode.Accepted:
            try:p=self.settings_store.create_profile(d.result_profile()); self.settings_store.preferences.last_setup_id=p.profile_id; self.settings_store.save(); self._refresh_profiles()
            except ValueError as exc:self._error(str(exc))
    def _edit_setup(self):
        if self.temporary_profile:return self._error("Clear the temporary override first")
        original=self._current_profile(); d=SetupProfileDialog(original,self,title="Edit Selected Setup")
        if d.exec()==QDialog.DialogCode.Accepted:
            try:self.settings_store.update_profile(original.profile_id,d.result_profile()); self._refresh_profiles()
            except ValueError as exc:self._error(str(exc))
    def _override_setup(self):
        if self.temporary_profile:
            self.temporary_profile=None; self.profile_combo.setEnabled(True); self.override_action.setText("Create Temporary Override…"); self.setup_status.setText(""); return self._profile_selected()
        d=SetupProfileDialog(self._current_profile().clone(temporary=True),self,title="Temporary Override — Not Saved")
        if d.exec()==QDialog.DialogCode.Accepted:self.temporary_profile=d.result_profile(); self.profile_combo.setEnabled(False); self.override_action.setText("Clear Temporary Override"); self.setup_status.setText("TEMPORARY")
    def _delete_setup(self):
        if self.temporary_profile:return self._error("Clear the temporary override first")
        p=self._current_profile()
        if QMessageBox.question(self,"Delete setup",f'Delete “{p.name}”?')==QMessageBox.StandardButton.Yes:
            try:self.settings_store.delete_profile(p.profile_id); self._refresh_profiles()
            except (ValueError,KeyError) as exc:self._error(str(exc))

    def _refresh_results(self):self.results_page.set_tests(self.database.list_tests())
    def _show_result(self,test_id):
        test=self.database.test(test_id); rows=self.database.samples(test_id); recorded=[r for r in rows if r["record_enabled"]]; x=[(r["engine_rpm"],r["power_kw"]) for r in recorded]; torque=[(r["engine_rpm"],r["torque_nm"]) for r in recorded]; self.results_page.plot.set_curves(test["name"],"Engine RPM","Power / torque",[("Power kW","#a77bed",x),("Torque N·m","#36c5d7",torque)]); ratio=[(r["elapsed_s"],r["cvt_ratio"]) for r in recorded if r["cvt_ratio"] is not None]; error=[(r["elapsed_s"],r["target_error"]) for r in recorded if r["target_error"] is not None]; self.results_page.plot_secondary.set_curves("Behavior / tracking","Time (s)","Ratio / error",[("CVT ratio","#f0b44d",ratio),("Target error","#d64a57",error)])
    def _export_test(self,test_id,kind):
        directory=Path(self.settings_store.preferences.export_directory); directory.mkdir(parents=True,exist_ok=True); test=self.database.test(test_id); stem=f"test_{test_id}_{test['test_type']}"
        paths=[]
        if kind in {"telemetry","both"}:paths.append(self.database.export_telemetry_csv(test_id,directory/f"{stem}_telemetry.csv"))
        if kind in {"summary","both"}:paths.append(self.database.export_summary_csv(test_id,directory/f"{stem}_summary.csv"))
        QMessageBox.information(self,"Export complete","\n".join(str(p) for p in paths))
    def _update_units(self):
        units=self.settings_store.preferences.unit_system; self.torque_metric.set_unit("N·m / lb·ft" if units=="both" else "lb·ft" if units=="imperial" else "N·m"); self.power_metric.set_unit("kW / hp" if units=="both" else "hp" if units=="imperial" else "kW")
    def _error(self,message):QMessageBox.warning(self,"Western Dynamometer",str(message))
    def closeEvent(self,event):
        self.run_timer.stop(); self.controller.shutdown(); self.database.close(); event.accept()
