# Western Dynamometer desktop v0.3

## Run on Windows

```powershell
cd desktop
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python main.py
```

Use **Settings → User Preferences** to enable the offline simulator. Restart
after changing the connection backend.

## Configuration model

- User Preferences stores units, export location, simulator selection, nominal
  valve voltage, and RGB palette.
- Machine Configuration stores valve active-PWM ranges, pump pulses/rev, the
  optional hard pump limit, load-cell calibration/filtering, and six global PID
  banks. Saved machine values are also sent to ESP32 NVS while connected and
  DISARMED.
- Named setups store per-engine/CVT ratios, RPM-source choice, decimal engine
  pulses/rev, throttle-servo endpoints/frequency, overspeed and anti-stall
  thresholds, optional vehicle calculations, and documentation-only component
  fields. Endpoints automatically determine servo direction.
- The selected/temporary setup is written to ESP32 RAM and acknowledged before
  every arm. Individual setups are never retained in controller NVS.

## Tests

- Peak Power always uses **flow → engine RPM**. Throttle is adjustable. Direct
  engine and CVT output labels/directions differ. Auto-max detects a plateau at
  normalized 100% flow and uses the enabled Machine Configuration pump-RPM
  limit as its hard ceiling.
- CVT Characterization is fully CSV-driven and always pressure → torque. The
  same actuator remains selected for startup, recorded stages, and shutdown.
- Track Simulation accepts pressure or flow with torque, engine-RPM, pump-RPM,
  or wheel-RPM targets. The actuator and target are fixed by metadata for the
  entire replay; throttle is prescribed by the CSV.
- Engineering / Manual applies slider or numeric changes immediately after one
  enable confirmation. Direct PWM plus all six actuator/target combinations are
  available.

## CSV profile metadata

Every new profile begins with:

```csv
# profile_name=Descriptive Name
# actuator=pressure
# target=torque
# units=metric
```

Characterization columns:

```csv
stage,target,transition_s,hold_s,throttle_pct,throttle_transition_s,group,group_repeats,record,notes
```

A `transition_s` of `0.10` creates a rapid torque application or release. Rows
may be edited outside the application, imported into the saved dropdown,
exported, or deleted. Bundled presets are protected. The executor adds
unrecorded startup/shutdown ramps.

Track columns:

```csv
time_s,target,throttle_pct,any_reference_columns...
```

Reference columns are retained on import. Track profiles support the same saved
dropdown/import/export/delete workflow.

## Data and results

Data streams to `%USERPROFILE%\.western_dyno\western_dyno.sqlite3`. Results are
read-only and show local timestamps. Export Telemetry writes each raw and
calculated sample; Export Summary writes behavior/real-world metrics; Export
Both writes both files.

Characterization summaries retain shift timing, ratio/rate/hysteresis,
load/release delay and slew, overshoot, droop/recovery, repeatability,
interventions, and estimated efficiency where a measured engine map exists.
Track summaries add average/peak output, energy, distributions, tracking error,
time in tolerance, estimated lag, wheel revolutions/distance, and faults.

## Safety

Both valves and throttle are raw 0 with enable pins disabled when disarmed,
unused, E-stopped, or faulted. Active valve UI 0–100% is mapped only inside its
configured 0–99% raw range; raw 100% is prohibited. Pressure UI 100% means
maximum braking and maps inversely to its raw PWM. Automatic tests never switch
actuators and never run two hydraulic PIDs simultaneously.

Reset/Clear succeeds only after the physical E-stop is released, outputs are
zero, shafts/torque are below reset thresholds, communications are valid, and
the original fault is gone. It returns DISARMED and never rearms.
