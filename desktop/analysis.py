"""Post-test behavior and real-world summary calculations."""
from __future__ import annotations
import math, statistics


def _values(rows,key,recorded=True):
    return [float(r[key]) for r in rows if r[key] is not None and (not recorded or bool(r["record_enabled"]))]


def _trapz(rows,key):
    total=0.
    for a,b in zip(rows,rows[1:]): total+=(float(a[key])+float(b[key]))*.5*(float(b["elapsed_s"])-float(a["elapsed_s"]))
    return total


def common_summary(rows):
    if not rows:return {"samples":0}
    power=_values(rows,"power_kw"); torque=_values(rows,"torque_nm"); engine=_values(rows,"engine_rpm"); pump=_values(rows,"pump_rpm")
    result={"samples":len(rows),"duration_s":float(rows[-1]["elapsed_s"]),"peak_output_power_kw":max(power,default=0),"peak_torque_nm":max(torque,default=0),"peak_engine_rpm":max(engine,default=0),"peak_pump_rpm":max(pump,default=0),"fault_sample_count":sum(bool(r["fault_flags"]) for r in rows),"anti_stall_interventions":sum(bool(r["anti_stall_active"]) for r in rows)}
    ratios=_values(rows,"cvt_ratio")
    if ratios:result.update(observed_cvt_ratio_min=min(ratios),observed_cvt_ratio_max=max(ratios))
    efficiencies=_values(rows,"estimated_efficiency_pct")
    if efficiencies:result["estimated_efficiency_pct"]={"mean":statistics.fmean(efficiencies),"peak":max(efficiencies),"label":"estimated, not true measured efficiency"}
    return result


def behavior_summary(rows):
    result=common_summary(rows); valid=[r for r in rows if r["cvt_ratio"] is not None and bool(r["record_enabled"])]
    rates=[]
    for a,b in zip(valid,valid[1:]):
        dt=float(b["elapsed_s"])-float(a["elapsed_s"])
        if dt>0:rates.append((float(b["elapsed_s"]),(float(b["cvt_ratio"])-float(a["cvt_ratio"]))/dt))
    result["cvt_ratio_rate_per_s"]={"maximum_up":max((x[1] for x in rates),default=0),"maximum_down":min((x[1] for x in rates),default=0)}
    transitions=[]
    for stage_index,name in sorted({(int(r["stage_index"]),r["stage_name"]) for r in valid if r["stage_name"]}):
        group=[r for r in valid if int(r["stage_index"])==stage_index]
        if len(group)<2:continue
        start,end=group[0],group[-1]; duration=float(end["elapsed_s"])-float(start["elapsed_s"]); target=float(end["target_value"] or 0); measured=float(end["torque_nm"]); errors=[abs(float(r["target_error"] or 0)) for r in group]
        torque0=float(start["torque_nm"]); delta=target-torque0; threshold=max(abs(delta)*.05,1.0)
        onset=next((float(r["elapsed_s"])-float(start["elapsed_s"]) for r in group if abs(float(r["torque_nm"])-torque0)>=abs(delta)*.1),None)
        completion=next((float(r["elapsed_s"])-float(start["elapsed_s"]) for r in group if abs(float(r["torque_nm"])-target)<=threshold),None)
        slew=[]
        for a,b in zip(group,group[1:]):
            dt=float(b["elapsed_s"])-float(a["elapsed_s"])
            if dt>0:slew.append((float(b["torque_nm"])-float(a["torque_nm"]))/dt)
        ratios=[float(r["cvt_ratio"]) for r in group]
        ratio_delta=ratios[-1]-ratios[0]
        ratio_onset=next((float(r["elapsed_s"])-float(start["elapsed_s"]) for r in group if abs(float(r["cvt_ratio"])-ratios[0])>=abs(ratio_delta)*.1),None)
        ratio_complete=next((float(r["elapsed_s"])-float(start["elapsed_s"]) for r in group if abs(float(r["cvt_ratio"])-ratios[0])>=abs(ratio_delta)*.9),None)
        overshoot=max((float(r["torque_nm"])-target for r in group),default=0) if delta>=0 else max((target-float(r["torque_nm"]) for r in group),default=0)
        transitions.append({"stage":name,"repeat":int(start["stage_index"]),"duration_s":duration,"response_direction":"load" if delta>=0 else "release","torque_response_delay_s":onset,"torque_completion_s":completion,"torque_overshoot_nm":max(0,overshoot),"maximum_torque_rise_nm_s":max(slew,default=0),"maximum_torque_release_nm_s":min(slew,default=0),"ratio_shift_onset_s":ratio_onset,"ratio_shift_completion_s":ratio_complete,"ratio_shift_duration_s":None if ratio_onset is None or ratio_complete is None else ratio_complete-ratio_onset,"ratio_start":ratios[0],"ratio_end":ratios[-1],"ratio_change":ratio_delta,"torque_target_nm":target,"torque_final_nm":measured,"peak_abs_target_error":max(errors,default=0),"engine_rpm_droop":max(0,float(start["engine_rpm"])-min(float(r["engine_rpm"]) for r in group)),"engine_rpm_recovery":float(end["engine_rpm"])-min(float(r["engine_rpm"]) for r in group),"pump_rpm_change":float(end["pump_rpm"])-float(start["pump_rpm"])})
    result["stage_behavior"]=transitions
    result["repeatability"]=_repeatability(valid)
    result["ratio_hysteresis_by_engine_rpm_bin"]=_hysteresis(valid)
    result["interpretation"]="Behavior metrics retain every recorded transition; power is instantaneous and is not reduced to an average-power score."
    return result


def _repeatability(rows):
    groups={}
    for r in rows:groups.setdefault(r["stage_name"],[]).append(float(r["cvt_ratio"]))
    values=[]
    for name,ratios in groups.items():
        if len(ratios)>1:values.append({"stage":name,"ratio_standard_deviation":statistics.pstdev(ratios)})
    return values


def _hysteresis(rows):
    bins={}
    for r in rows:
        direction=(str(r["sweep_direction"])+" "+str(r["stage_name"])).lower()
        key="up" if "up" in direction else "down" if "down" in direction or "back" in direction or "release" in direction else ""
        if not key:continue
        rpm_bin=round(float(r["engine_rpm"])/100)*100
        bins.setdefault(rpm_bin,{"up":[],"down":[]})[key].append(float(r["cvt_ratio"]))
    return [{"engine_rpm":rpm,"upshift_ratio":statistics.fmean(values["up"]),"downshift_ratio":statistics.fmean(values["down"]),"ratio_hysteresis":statistics.fmean(values["down"])-statistics.fmean(values["up"])} for rpm,values in sorted(bins.items()) if values["up"] and values["down"]]


def track_summary(rows):
    result=common_summary(rows); recorded=[r for r in rows if bool(r["record_enabled"])]
    power=_values(recorded,"power_kw",False); torque=_values(recorded,"torque_nm",False); errors=_values(recorded,"target_error",False)
    result.update({"average_output_power_kw":statistics.fmean(power) if power else 0,"average_torque_nm":statistics.fmean(torque) if torque else 0,"output_energy_kj":_trapz(recorded,"power_kw") if len(recorded)>1 else 0,"tracking_mae":statistics.fmean(abs(x) for x in errors) if errors else 0,"tracking_rmse":math.sqrt(statistics.fmean(x*x for x in errors)) if errors else 0,"maximum_absolute_tracking_error":max((abs(x) for x in errors),default=0),"power_distribution_kw":_distribution(power),"torque_distribution_nm":_distribution(torque)})
    tolerance=next((float(r["target_value"])*.05 for r in recorded if r["target_value"]),0)
    result["time_within_5_percent_target_pct"]=100*sum(abs(float(r["target_error"] or 0))<=tolerance for r in recorded)/len(recorded) if recorded else 0
    wheel=_values(recorded,"wheel_rpm",False)
    if wheel and len(recorded)>1:
        # Distance needs tire circumference; the main window adds it when available.
        result["wheel_revolutions"]=_trapz(recorded,"wheel_rpm")/60
    result["estimated_tracking_lag_s"]=_tracking_lag(recorded)
    result["interpretation"]="Track metrics emphasize sustained real-world power, energy, and target tracking over the full replay."
    return result


def _distribution(values):
    if not values:return {}
    ordered=sorted(values)
    def percentile(p):return ordered[min(len(ordered)-1,round((len(ordered)-1)*p))]
    return {"minimum":ordered[0],"p10":percentile(.1),"median":percentile(.5),"p90":percentile(.9),"maximum":ordered[-1]}


def _tracking_lag(rows):
    if len(rows)<5:return None
    targets=[float(r["target_value"] or 0) for r in rows]; measured=[targets[i]-float(rows[i]["target_error"] or 0) for i in range(len(rows))]; mean_t=statistics.fmean(targets); mean_m=statistics.fmean(measured); best=(None,float("-inf"))
    for shift in range(-20,21):
        pairs=[(targets[i],measured[i+shift]) for i in range(len(rows)) if 0<=i+shift<len(rows)]
        score=sum((a-mean_t)*(b-mean_m) for a,b in pairs)
        if score>best[1]:best=(shift,score)
    dt=statistics.median(float(b["elapsed_s"])-float(a["elapsed_s"]) for a,b in zip(rows,rows[1:]))
    return best[0]*dt
