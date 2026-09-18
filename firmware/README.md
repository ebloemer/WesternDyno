# Protocol-v5 ESP32 firmware

The controller is already arranged as `src/main.cpp` with
`include/dyno_protocol.h`. Copy your existing `platformio.ini` and any required
project libraries into this `firmware/` folder. The desktop and firmware must
both use protocol version 5.

## Packet contract

- Command: 36 bytes, `<IBBBBIHHHHfffI`
- Telemetry: 64 bytes, `<IBBBBHHIIffffffffffI`
- Configuration: 60 bytes, `<IBBHIB3xffffffffffI`
- CRC-32 covers every byte before the CRC.
- A new command sequence is required at least every 500 ms while armed.
- BLE ATT MTU must accommodate a 64-byte value plus ATT overhead.

Machine parameters and all six PID banks are CRC/schema-validated in NVS.
Active engine/CVT setups remain RAM-only and are invalidated at boot and BLE
disconnect. Firmware rejects arming until an active setup is acknowledged and
the required PID bank is enabled.

## Actuator rules

The command selects pressure or flow once per automatic run. Firmware enables
only that valve and forces the unused valve to raw 0. Manual direct-output mode
may commission both independently. UI normalized commands map to the configured
active raw ranges; the allowed maximum is 99%. Pressure mapping is inverted.

Direct-engine anti-stall retains the selected actuator: flow control moves to
maximum-flow/minimum-load, while pressure control moves to zero normalized
braking. It resets integral state, logs its telemetry flag, and faults after the
setup-specific recovery timeout.

## Required unloaded validation

1. Confirm both enable pins and all three PWM outputs are raw 0 at boot,
   disarm, BLE loss, E-stop, watchdog, and fault.
2. Verify pressure raw 0 produces the intended safe maximum-pressure relief.
3. Measure and enter physically safe active PWM min/max for each valve; prove
   that flow maximum is the real maximum-flow position.
4. Verify closed/full throttle endpoints and automatic direction on every
   engine setup before allowing ignition.
5. Verify pump and engine PPR against independent tachometers; validate both
   direct-engine RPM sources.
6. Calibrate/tare torque with known loads and confirm sign.
7. Enable and tune only one PID bank at a time. Test pressure→torque,
   pressure→engine RPM, pressure→pump RPM, flow→torque, flow→engine RPM, and
   flow→pump RPM separately.
8. Prove pump/engine overspeed, anti-stall, target-saturation, reset-button,
   UI-reset, sensor-loss, and communications faults without a loaded engine.

The supplied firmware uses the Arduino-ESP32 2.x channel-based LEDC API. Port
the calls if the project uses Arduino-ESP32 3.x.
