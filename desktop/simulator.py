"""Safe offline plant simulator implementing protocol v5 semantics."""
from __future__ import annotations
from dataclasses import replace
import math, random, struct, time, zlib
from PySide6.QtCore import QObject,QTimer,Signal
from protocol import PACKET_MAGIC,PACKET_VERSION,TELEMETRY_FORMAT_NO_CRC,Command,CommandFlag,Config,ConfigFlag,ConfigKind,DynoMode,Fault,SystemState,TelemetryFlag


def _approach(value,target,dt,response): return value+(target-value)*min(1.0,dt*response)


class DynoSimulator(QObject):
    connection_changed=Signal(bool,str); telemetry_received=Signal(bytes); error=Signal(str); config_received=Signal(bytes)
    def __init__(self,parent=None):
        super().__init__(parent); self.timer=QTimer(self); self.timer.setInterval(50); self.timer.timeout.connect(self._step); self.command=Command(); self.state=SystemState.DISARMED; self.faults=Fault.NONE; self.sequence=0; self.config=Config(); self.setup_valid=False; self.is_cvt=False; self.engine_sensor_source=True; self.engine_to_pump=.7; self.output_to_pump=.7; self.closed_us=500; self.full_us=1200; self.flow_min=0; self.flow_max=98; self.press_min=0; self.press_max=98; self.engine=0.; self.pump=0.; self.torque=0.; self.throttle=0.; self.flow_raw=0.; self.pressure_raw=0.; self._last=time.monotonic(); self._phase=0.
    def connect_dyno(self): self._last=time.monotonic(); self.timer.start(); self.connection_changed.emit(True,"Offline simulator connected"); self.config_received.emit(self.config.pack())
    def disconnect_dyno(self): self.command=replace(self.command,arm=False); self.state=SystemState.DISARMED; self.setup_valid=False; self.timer.stop(); self.connection_changed.emit(False,"Simulator disconnected")
    def stop_worker(self): self.disconnect_dyno()
    def set_command(self,command):
        self.sequence=(self.sequence+1)&0xffffffff or 1; self.command=replace(command,sequence=self.sequence)
        if command.emergency: self.state=SystemState.ESTOP
        elif command.saturation_fault: self.faults|=Fault.TARGET_SATURATION; self.state=SystemState.FAULT
        elif command.clear_latch and self.engine<50 and self.pump<50 and abs(self.torque)<2: self.faults=Fault.NONE; self.state=SystemState.DISARMED
        elif not command.arm and self.state not in {SystemState.ESTOP,SystemState.FAULT}: self.state=SystemState.DISARMED
        elif command.arm and self.state==SystemState.DISARMED:
            if self.setup_valid:self.state=SystemState.ARMED
            else:self.faults|=Fault.SETUP_REQUIRED; self.state=SystemState.FAULT
    def set_config(self,config):
        if self.state!=SystemState.DISARMED: self.error.emit("Simulator rejected configuration because it is not DISARMED"); return
        self.config=config
        if config.kind==ConfigKind.MACHINE:
            self.flow_min,self.flow_max,self.press_min,self.press_max=config.values[:4]
        elif config.kind==ConfigKind.ACTIVE_SETUP:
            self.engine_to_pump,self.output_to_pump=config.values[:2]; self.closed_us,self.full_us=config.values[3],config.values[4]; self.is_cvt=bool(config.action_flags&ConfigFlag.SETUP_CVT); self.engine_sensor_source=bool(config.action_flags&ConfigFlag.ENGINE_SENSOR_SOURCE); self.setup_valid=True
        self.config_received.emit(config.pack())
    def _map_flow(self,pct): return self.flow_min+(self.flow_max-self.flow_min)*pct/100
    def _map_pressure(self,pct): return self.press_max-(self.press_max-self.press_min)*pct/100
    def _step(self):
        now=time.monotonic(); dt=min(max(now-self._last,.001),.2); self._last=now; self._phase+=dt; c=self.command
        de=dp=dtq=dth=flow=pressure=0.
        if self.state==SystemState.ARMED:
            dth=float(c.manual_throttle_pct) if c.manual_throttle_override or c.mode==DynoMode.MANUAL else 40.
            pressure_selected=c.pressure_actuator
            if c.mode==DynoMode.MANUAL:
                flow=self._map_flow(c.manual_flow_pct); pressure=self._map_pressure(c.manual_pressure_pct); de=800+dth*32; dp=de*(self.output_to_pump/(2.6 if self.is_cvt else 1/self.engine_to_pump))*(.2+.8*c.manual_flow_pct/100); dtq=.65*c.manual_pressure_pct
            else:
                target={DynoMode.TORQUE:c.target_torque_nm,DynoMode.RPM:c.target_pump_rpm,DynoMode.ENGINE_RPM:c.target_engine_rpm}.get(c.mode,0)
                if c.mode==DynoMode.TORQUE: dtq=target; de=900+dth*32
                elif c.mode==DynoMode.RPM: dp=target; de=max(900,dp/max(self.engine_to_pump,.001))
                else: de=target
                if self.is_cvt:
                    ratio=3.9-min(1,max(0,(de-1800)/1800))*3; dp=de/ratio*self.output_to_pump
                elif c.mode!=DynoMode.RPM: dp=de*self.engine_to_pump
                if c.mode!=DynoMode.TORQUE: dtq=12+dth*.4
                if pressure_selected: pressure=self._map_pressure(min(100,max(0,dtq/0.7))); flow=0
                else: flow=self._map_flow(min(100,max(0,(target/4000)*100))); pressure=0
        self.engine=_approach(self.engine,de,dt,3); self.pump=_approach(self.pump,dp,dt,4); self.torque=_approach(self.torque,dtq,dt,5); self.throttle=_approach(self.throttle,dth,dt,5); self.flow_raw=_approach(self.flow_raw,flow,dt,8); self.pressure_raw=_approach(self.pressure_raw,pressure,dt,8)
        if self.state!=SystemState.ARMED:
            self.engine=self.pump=self.torque=self.throttle=self.flow_raw=self.pressure_raw=0.; noise=0.
        else: noise=math.sin(self._phase*7)*1.5+random.uniform(-.5,.5)
        physical=max(0,self.engine+noise) if self.engine_sensor_source else 0.; derived=max(0,(self.pump+noise*.7)/max(self.engine_to_pump,.001)); selected=physical if self.engine_sensor_source else derived; pump=max(0,self.pump+noise*.7); torque=self.torque+noise*.02
        flags=TelemetryFlag.BLE_CONNECTED|TelemetryFlag.SCALE_CONNECTED|TelemetryFlag.PUMP_RPM_VALID|TelemetryFlag.TORQUE_VALID
        if self.engine_sensor_source:flags|=TelemetryFlag.ENGINE_RPM_VALID
        if self.setup_valid:flags|=TelemetryFlag.ACTIVE_SETUP_VALID
        if self.state==SystemState.ARMED:flags|=TelemetryFlag.ARMED
        if self.state==SystemState.ESTOP:flags|=TelemetryFlag.ESTOP
        if c.pressure_actuator:flags|=TelemetryFlag.PRESSURE_ACTUATOR
        body=struct.pack(TELEMETRY_FORMAT_NO_CRC,PACKET_MAGIC,PACKET_VERSION,int(self.state),int(c.mode),0,int(flags),0,int(self.faults),self.sequence,physical,selected,c.target_engine_rpm,pump,c.target_pump_rpm,torque,c.target_torque_nm,self.flow_raw,self.pressure_raw,self.throttle)
        self.telemetry_received.emit(body+struct.pack("<I",zlib.crc32(body)&0xffffffff))
