# DynoController replacement firmware

This package replaces the original ESP32 dyno firmware while preserving its
general Arduino/C++ structure. The touchscreen/controller-specific packet
assumptions have been removed. BLE remains available for a future Python
desktop application.

## Intentional fail-safe behavior

Every disarm, E-stop, BLE disconnect, command timeout, unsupported mode,
overspeed fault, or required-sensor fault commands the system's intended
maximum-braking state:

- engine throttle pulse: minimum;
- flow-valve PWM: zero and enable low (valve closed);
- pressure-valve PWM: zero and enable low (maximum relief setting/pressure).

The physical engine ignition kill remains an independent safety layer.

## Important configuration checks

Before loaded operation, verify the constants near the top of
`DynoController.cpp`, especially:

- throttle pulse range and 150 Hz compatibility;
- load-cell scale factor and torque sign;
- engine and pump pulse edges/magnet counts;
- 70 N-m maximum target and 90 N-m torque trip;
- 2,850/3,000/3,200 RPM pump limiter thresholds;
- 4,100 RPM engine trip;
- valve enable-pin polarity.

The PID gains are copied from the old firmware only as starting values. They
must be retuned because the new loop runs at a deterministic 50 Hz and the
pressure-valve direction is explicitly inverted.

Normal valve and throttle changes are slew-limited. E-stop, fault, disconnect,
watchdog, and overspeed actions bypass the normal ramp and apply braking
immediately.

## Modes

- Manual: direct flow, braking-pressure percentage, and throttle commands.
- Torque: flow valve closed; pressure-valve current regulates measured torque.
- RPM: pressure valve de-energized at maximum braking; flow valve regulates
  pump RPM directly.
- CVT: reserved in the protocol but rejected by firmware until defined.

`manualPressurePct` is an operator-facing braking command:

- 0% = minimum braking (maximum pressure-valve duty);
- 100% = maximum braking (zero pressure-valve duty).

Telemetry reports raw pressure-valve duty, so its direction is inverse to
braking percentage.

## BLE protocol version 2

The command and telemetry structures are packed little-endian binary packets:

- command packet: 36 bytes;
- telemetry packet: 60 bytes;
- both include a standard CRC-32;
- commands include a monotonic 32-bit sequence number;
- a valid command must arrive at least every 500 ms while armed;
- reset after E-stop/fault returns only to DISARMED;
- a new explicit arm request is required to resume.

The future Python application can use `struct.pack`/`struct.unpack` with an
explicit `<` little-endian prefix and `zlib.crc32` for the packet CRC.

- command format: `<IBBBBIHHHHfffI`;
- telemetry format: `<IBBBBIIffffffffffI`.

The CRC is calculated over every byte preceding the final CRC field.

Because telemetry is 60 bytes, the BLE connection needs an ATT MTU of at least
63 bytes. The firmware requests 517; the computer application must verify that
notifications arrive at the full expected length before allowing arming.

## Integration

Place `DynoController.cpp` and `dyno_protocol.h` in the existing firmware
project. This code intentionally retains the Arduino-ESP32 2.x LEDC API used by
the original code (`ledcSetup`, `ledcAttachPin`, and channel-based `ledcWrite`).
If the project is upgraded to Arduino-ESP32 3.x, migrate the LEDC calls to that
core's pin-based API.

The HX711 `begin()` call also retains the four-argument form used by the
original project. Confirm that the installed HX711 library exposes that exact
overload.

## Bench validation sequence

1. Power the controller with the engine unable to start.
2. Confirm both valve outputs and enable pins enter the intended fail-safe
   state during boot, disconnect, E-stop, and watchdog timeout.
3. Confirm throttle minimum physically closes the linkage.
4. Validate RPM polarity/scaling with an independent tachometer.
5. Validate torque sign and calibration with known loads.
6. Confirm manual pressure percentage is inverse to pressure-valve duty.
7. Verify that approaching 3,000 pump RPM progressively closes flow, increases
   braking pressure, and closes throttle.
8. Inject BLE loss and sensor disconnections while unloaded.
9. Retune one controller at a time before a loaded combined-system test.
