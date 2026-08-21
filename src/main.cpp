#include <Arduino.h>
#include <HX711.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "dyno_protocol.h"

// ==========================================================================
// Hardware configuration
// ==========================================================================

// LED
static const uint8_t LED_PIN = 48;
static const uint8_t LED_COUNT = 1;

// Physical safety inputs (normally open switches to ground)
static const uint8_t EMERGENCY_PIN = 1;
static const uint8_t RESET_PIN = 42;

// HX711
static const uint8_t LOAD_CELL_DOUT_PIN = 4;
static const uint8_t LOAD_CELL_CLK_PIN = 5;

// RPM sensors
static const uint8_t ENGINE_PULSE_SENSOR_PIN = 6;
static const uint8_t PUMP_PULSE_SENSOR_PIN = 9;

// Engine throttle
static const uint8_t ENGINE_THROTTLE_PIN = 7;

// Hydraulic valves
static const uint8_t FLOW_VALVE_PIN = 10;
static const uint8_t FLOW_VALVE_ENABLE_PIN = 11;
static const uint8_t PRESSURE_VALVE_PIN = 12;
static const uint8_t PRESSURE_VALVE_ENABLE_PIN = 13;

// ESP32 LEDC channels (Arduino-ESP32 2.x API, matching the original project)
static const uint8_t FLOW_VALVE_CHANNEL = 1;
static const uint8_t PRESSURE_VALVE_CHANNEL = 2;
static const uint8_t ENGINE_THROTTLE_CHANNEL = 3;

// ==========================================================================
// Safety and control configuration
// ==========================================================================

static const uint8_t PWM_RESOLUTION_BITS = 12;
static const uint16_t PWM_MAX_RAW = (1U << PWM_RESOLUTION_BITS) - 1U;
static const uint16_t VALVE_MAX_DUTY = (uint16_t)(PWM_MAX_RAW * 0.98f);

static const uint16_t FLOW_VALVE_FREQUENCY_HZ = 250;
static const uint16_t PRESSURE_VALVE_FREQUENCY_HZ = 250;
static const uint16_t ENGINE_THROTTLE_FREQUENCY_HZ = 150;

// Servo pulse range retained from the original system. Verify against the
// exact servo data and mechanical linkage before loaded operation.
static const uint16_t ENGINE_THROTTLE_MIN_US = 500;
static const uint16_t ENGINE_THROTTLE_MAX_US = 1200;

static const uint8_t ENGINE_MAGNETS = 1;
static const uint8_t PUMP_MAGNETS = 4;

static const float MAX_TARGET_ENGINE_RPM = 3800.0f;
static const float ENGINE_HARD_TRIP_RPM = 4100.0f;

static const float MAX_TARGET_PUMP_RPM = 3000.0f;
static const float PUMP_SOFT_LIMIT_START_RPM = 2850.0f;
static const float PUMP_SOFT_LIMIT_RPM = 3000.0f;
static const float PUMP_HARD_TRIP_RPM = 3200.0f;
static const uint32_t PUMP_OVERSPEED_LATCH_MS = 300;

// These torque limits must be checked against the load cell, torque arm,
// frame, couplings, and fasteners before operation.
static const float MAX_TARGET_TORQUE_NM = 70.0f;
static const float TORQUE_HARD_TRIP_NM = 90.0f;

static const uint32_t COMMAND_TIMEOUT_MS = 500;
static const uint32_t SENSOR_STARTUP_GRACE_MS = 3000;
static const uint32_t RPM_SIGNAL_TIMEOUT_US = 500000;
static const uint32_t TORQUE_SIGNAL_TIMEOUT_MS = 250;

static const uint32_t CONTROL_PERIOD_US = 20000; // 50 Hz deterministic control
static const uint32_t BLE_TELEMETRY_PERIOD_MS = 50;
static const uint32_t STATUS_PERIOD_MS = 500;

// Normal commands are rate-limited. Safety actions bypass these limits and
// move immediately to maximum braking/minimum throttle.
static const float VALVE_DUTY_SLEW_PER_SECOND = 6000.0f;
static const float THROTTLE_SLEW_US_PER_SECOND = 350.0f;

static const float LOAD_CELL_SCALE_FACTOR = 2280.0f;
static const float TORQUE_FILTER_ALPHA = 0.20f;

// Initial gains only. The corrected fixed-rate loop and inverted pressure
// response mean all three controllers must be tuned again on the real system.
static float pressureP = 1.0f;
static float pressureI = 0.2f;
static float pressureD = 0.05f;

static float flowP = 0.25f;
static float flowI = 0.0625f;
static float flowD = 0.025f;

static float engineP = 0.1f;
static float engineI = 0.02f;
static float engineD = 0.005f;

// ==========================================================================
// Objects and shared state
// ==========================================================================

enum LedColor : uint8_t { LED_OFF, LED_RED, LED_YELLOW, LED_GREEN, LED_BLUE };

Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
HX711 scale;

BLEServer* g_server = nullptr;
BLECharacteristic* g_commandCharacteristic = nullptr;
BLECharacteristic* g_telemetryCharacteristic = nullptr;

volatile bool g_bleClientConnected = false;
volatile bool g_bleDisconnectRequested = false;
volatile bool g_newBleSessionRequested = false;
volatile bool g_remoteEmergencyRequested = false;
volatile bool g_invalidCommandReceived = false;

portMUX_TYPE g_commandMux = portMUX_INITIALIZER_UNLOCKED;
CommandPacket g_pendingCommand{};
volatile bool g_pendingCommandAvailable = false;

portMUX_TYPE g_pulseMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t g_enginePulseNewestUs = 0;
volatile uint32_t g_enginePulsePreviousUs = 0;
volatile uint32_t g_enginePulseCount = 0;
volatile uint32_t g_pumpPulseNewestUs = 0;
volatile uint32_t g_pumpPulsePreviousUs = 0;
volatile uint32_t g_pumpPulseCount = 0;
volatile bool g_estopIsrRequested = false;

DynoSystemState systemState = STATE_BOOT;
DynoMode mode = MODE_MANUAL;
uint32_t faultFlags = FAULT_NONE;

bool haveCommandSequence = false;
uint32_t lastCommandSequence = 0;
uint32_t lastValidCommandMs = 0;
uint32_t sensorGraceUntilMs = 0;
uint32_t overspeedStartedMs = 0;

bool scaleConnected = false;
bool torqueValid = false;
bool engineRpmValid = false;
bool pumpRpmValid = false;
bool torqueFilterInitialized = false;
bool softOverspeedActive = false;

uint32_t lastTorqueSampleMs = 0;
uint32_t lastControlUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastLedMs = 0;

LedColor currentLedColor = LED_OFF;

// Commands
uint16_t manualFlowPct = 0;
uint16_t manualPressurePct = 0;
uint16_t manualThrottlePct = 0;
float targetTorqueNm = 60.0f;
float targetPumpRpm = 2500.0f;
float targetEngineRpm = 3600.0f;

// Measurements
float engineRpm = 0.0f;
float pumpRpm = 0.0f;
float torqueNm = 0.0f;

// Actuator commands
float flowValveValue = 0.0f;
float pressureValveValue = 0.0f;
float engineThrottlePulseUs = ENGINE_THROTTLE_MIN_US;
bool flowValveEnabled = false;
bool pressureValveEnabled = false;

// Incremental PID state
float pressurePreviousError = 0.0f;
float pressurePreviousPreviousError = 0.0f;
float flowPreviousError = 0.0f;
float flowPreviousPreviousError = 0.0f;
float enginePreviousError = 0.0f;
float enginePreviousPreviousError = 0.0f;
bool pressurePidInitialized = false;
bool flowPidInitialized = false;
bool enginePidInitialized = false;

// ==========================================================================
// Forward declarations
// ==========================================================================

void setupBle();
void sendTelemetry();
void printStatus();
void updateLed();
void updateSensors();
void getTorque();
void getEngineRpm();
void getPumpRpm();
void processSafetyInputs();
void processPendingCommand();
void evaluateSafety();
void runControlLoop(float dtSeconds);
void pumpControl(float dtSeconds);
void engineRpmControl(float dtSeconds);
void applyOverspeedOverride();
void applyFailSafeOutputs();
void writeActuatorOutputs();
void resetPidState();
void enterDisarmed();
void enterEstop();
void enterFault(uint32_t fault);
bool commandValuesValid(const CommandPacket& pkt);
bool sequenceIsNewer(uint32_t incoming, uint32_t previous);
uint32_t engineDutyFromPulse(uint32_t pulseUs);
float dutyToPercent(float duty);
float percentToDuty(uint16_t percent);
void deltaPidControl(float error,
                     float& previousError,
                     float& previousPreviousError,
                     float P,
                     float I,
                     float D,
                     float& output,
                     float minimum,
                     float maximum,
                     float dtSeconds);

// ==========================================================================
// Interrupt handlers
// ==========================================================================

void IRAM_ATTR enginePulseIsr() {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&g_pulseMux);
  g_enginePulsePreviousUs = g_enginePulseNewestUs;
  g_enginePulseNewestUs = now;
  ++g_enginePulseCount;
  portEXIT_CRITICAL_ISR(&g_pulseMux);
}

void IRAM_ATTR pumpPulseIsr() {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&g_pulseMux);
  g_pumpPulsePreviousUs = g_pumpPulseNewestUs;
  g_pumpPulseNewestUs = now;
  ++g_pumpPulseCount;
  portEXIT_CRITICAL_ISR(&g_pulseMux);
}

void IRAM_ATTR emergencyIsr() {
  g_estopIsrRequested = true;
}

// ==========================================================================
// Basic helpers
// ==========================================================================

void setLedColor(LedColor color) {
  switch (color) {
    case LED_RED:    led.setPixelColor(0, led.Color(255, 0, 0)); break;
    case LED_YELLOW: led.setPixelColor(0, led.Color(255, 255, 0)); break;
    case LED_GREEN:  led.setPixelColor(0, led.Color(0, 255, 0)); break;
    case LED_BLUE:   led.setPixelColor(0, led.Color(0, 0, 255)); break;
    default:         led.setPixelColor(0, led.Color(0, 0, 0)); break;
  }

  currentLedColor = color;
  led.show();
}

float percentToDuty(uint16_t percent) {
  const float limited = constrain((float)percent, 0.0f, 100.0f);
  return (limited / 100.0f) * VALVE_MAX_DUTY;
}

float dutyToPercent(float duty) {
  return constrain((duty / (float)VALVE_MAX_DUTY) * 100.0f, 0.0f, 100.0f);
}

uint32_t engineDutyFromPulse(uint32_t pulseUs) {
  const uint32_t periodUs = 1000000UL / ENGINE_THROTTLE_FREQUENCY_HZ;
  pulseUs = constrain(pulseUs,
                      (uint32_t)ENGINE_THROTTLE_MIN_US,
                      (uint32_t)ENGINE_THROTTLE_MAX_US);
  return (pulseUs * PWM_MAX_RAW) / periodUs;
}

bool sequenceIsNewer(uint32_t incoming, uint32_t previous) {
  return (int32_t)(incoming - previous) > 0;
}

// ==========================================================================
// Sensors
// ==========================================================================

void getEngineRpm() {
  uint32_t newest;
  uint32_t previous;
  uint32_t count;

  portENTER_CRITICAL(&g_pulseMux);
  newest = g_enginePulseNewestUs;
  previous = g_enginePulsePreviousUs;
  count = g_enginePulseCount;
  portEXIT_CRITICAL(&g_pulseMux);

  const uint32_t now = micros();
  const uint32_t interval = newest - previous;

  if (count < 2 || newest == 0 || interval == 0 ||
      (uint32_t)(now - newest) > RPM_SIGNAL_TIMEOUT_US) {
    engineRpm = 0.0f;
    engineRpmValid = false;
    return;
  }

  const float calculated = 60000000.0f / ((float)interval * ENGINE_MAGNETS);
  if (!isfinite(calculated) || calculated < 0.0f || calculated > 10000.0f) {
    engineRpm = 0.0f;
    engineRpmValid = false;
    return;
  }

  engineRpm = calculated;
  engineRpmValid = true;
}

void getPumpRpm() {
  uint32_t newest;
  uint32_t previous;
  uint32_t count;

  portENTER_CRITICAL(&g_pulseMux);
  newest = g_pumpPulseNewestUs;
  previous = g_pumpPulsePreviousUs;
  count = g_pumpPulseCount;
  portEXIT_CRITICAL(&g_pulseMux);

  const uint32_t now = micros();
  const uint32_t interval = newest - previous;

  if (count < 2 || newest == 0 || interval == 0 ||
      (uint32_t)(now - newest) > RPM_SIGNAL_TIMEOUT_US) {
    pumpRpm = 0.0f;
    pumpRpmValid = false;
    return;
  }

  const float calculated = 60000000.0f / ((float)interval * PUMP_MAGNETS);
  if (!isfinite(calculated) || calculated < 0.0f || calculated > 10000.0f) {
    pumpRpm = 0.0f;
    pumpRpmValid = false;
    return;
  }

  pumpRpm = calculated;
  pumpRpmValid = true;
}

void getTorque() {
  if (!scaleConnected) {
    torqueValid = false;
    return;
  }

  if (scale.is_ready()) {
    const float sample = scale.get_units(1);
    if (isfinite(sample)) {
      if (!torqueFilterInitialized) {
        torqueNm = sample;
        torqueFilterInitialized = true;
      } else {
        torqueNm += TORQUE_FILTER_ALPHA * (sample - torqueNm);
      }

      lastTorqueSampleMs = millis();
      torqueValid = true;
    } else {
      torqueValid = false;
    }
  }

  if ((uint32_t)(millis() - lastTorqueSampleMs) > TORQUE_SIGNAL_TIMEOUT_MS) {
    torqueValid = false;
  }
}

void updateSensors() {
  getEngineRpm();
  getPumpRpm();
  getTorque();
}

// ==========================================================================
// State and safety
// ==========================================================================

void resetPidState() {
  pressurePreviousError = 0.0f;
  pressurePreviousPreviousError = 0.0f;
  flowPreviousError = 0.0f;
  flowPreviousPreviousError = 0.0f;
  enginePreviousError = 0.0f;
  enginePreviousPreviousError = 0.0f;
  pressurePidInitialized = false;
  flowPidInitialized = false;
  enginePidInitialized = false;
}

void applyFailSafeOutputs() {
  // Intentional system fail-safe: maximum hydraulic braking.
  // - FAP161CN flow valve: de-energized/closed.
  // - AP04G2YR21CN pressure valve: de-energized/maximum pressure.
  // - Engine servo: minimum commanded throttle.
  flowValveValue = 0.0f;
  pressureValveValue = 0.0f;
  engineThrottlePulseUs = ENGINE_THROTTLE_MIN_US;
  flowValveEnabled = false;
  pressureValveEnabled = false;
  writeActuatorOutputs();
}

void enterDisarmed() {
  systemState = STATE_DISARMED;
  resetPidState();
  overspeedStartedMs = 0;
  softOverspeedActive = false;
  applyFailSafeOutputs();
}

void enterEstop() {
  if (systemState != STATE_ESTOP) {
    Serial.println("[SAFETY] Emergency stop latched");
  }

  systemState = STATE_ESTOP;
  resetPidState();
  overspeedStartedMs = 0;
  softOverspeedActive = false;
  applyFailSafeOutputs();
}

void enterFault(uint32_t fault) {
  faultFlags |= fault;
  if (systemState == STATE_ESTOP) {
    // An E-stop remains the dominant latched state until physical reset.
    applyFailSafeOutputs();
    return;
  }

  if (systemState != STATE_FAULT) {
    Serial.print("[SAFETY] Fault latched: 0x");
    Serial.println(faultFlags, HEX);
  }

  systemState = STATE_FAULT;
  resetPidState();
  overspeedStartedMs = 0;
  softOverspeedActive = false;
  applyFailSafeOutputs();
}

void processSafetyInputs() {
  if (g_estopIsrRequested || digitalRead(EMERGENCY_PIN) == LOW) {
    g_estopIsrRequested = false;
    enterEstop();
  }

  if (g_remoteEmergencyRequested) {
    g_remoteEmergencyRequested = false;
    enterEstop();
  }

  // A physical reset clears ESTOP/FAULT only to DISARMED. It never resumes a test.
  static bool previousResetPressed = false;
  static bool resetConsumed = false;
  static uint32_t resetChangedMs = 0;
  const bool resetPressed = digitalRead(RESET_PIN) == LOW;

  if (resetPressed != previousResetPressed) {
    resetChangedMs = millis();
    previousResetPressed = resetPressed;
    if (!resetPressed) resetConsumed = false;
  }

  if (resetPressed && !resetConsumed &&
      (uint32_t)(millis() - resetChangedMs) >= 50) {
    resetConsumed = true;

    if (digitalRead(EMERGENCY_PIN) == HIGH &&
        (systemState == STATE_ESTOP || systemState == STATE_FAULT)) {
      faultFlags = FAULT_NONE;
      haveCommandSequence = false;
      lastValidCommandMs = 0;
      manualFlowPct = 0;
      manualPressurePct = 0;
      manualThrottlePct = 0;
      enterDisarmed();
      Serial.println("[SAFETY] Latched state cleared; system remains DISARMED");
    }
  }
}

bool commandValuesValid(const CommandPacket& pkt) {
  if (pkt.emergency > 1 || pkt.arm > 1) return false;
  if (pkt.mode > MODE_RPM) return false; // CVT is deliberately unavailable.
  if (pkt.reserved0 != 0) return false;
  if (pkt.manualFlowPct > 100) return false;
  if (pkt.manualPressurePct > 100) return false;
  if (pkt.manualThrottlePct > 100) return false;
  if (!isfinite(pkt.targetTorqueNm) ||
      !isfinite(pkt.targetPumpRpm) ||
      !isfinite(pkt.targetEngineRpm)) return false;
  if (pkt.targetTorqueNm < 0.0f || pkt.targetTorqueNm > MAX_TARGET_TORQUE_NM) return false;
  if (pkt.targetPumpRpm < 0.0f || pkt.targetPumpRpm > MAX_TARGET_PUMP_RPM) return false;
  if (pkt.targetEngineRpm < 0.0f || pkt.targetEngineRpm > MAX_TARGET_ENGINE_RPM) return false;
  return true;
}

void processPendingCommand() {
  if (g_newBleSessionRequested) {
    g_newBleSessionRequested = false;
    haveCommandSequence = false;
  }

  if (g_bleDisconnectRequested) {
    g_bleDisconnectRequested = false;
    if (systemState == STATE_ARMED) {
      enterFault(FAULT_BLE_DISCONNECTED);
    }
  }

  if (g_invalidCommandReceived) {
    g_invalidCommandReceived = false;
    if (systemState == STATE_ARMED) {
      enterFault(FAULT_INVALID_COMMAND);
    }
  }

  CommandPacket pkt{};
  bool available = false;

  portENTER_CRITICAL(&g_commandMux);
  if (g_pendingCommandAvailable) {
    pkt = g_pendingCommand;
    g_pendingCommandAvailable = false;
    available = true;
  }
  portEXIT_CRITICAL(&g_commandMux);

  if (!available) return;

  if (haveCommandSequence && !sequenceIsNewer(pkt.sequence, lastCommandSequence)) {
    return;
  }

  if (!commandValuesValid(pkt)) {
    if (pkt.mode == MODE_CVT) enterFault(FAULT_UNSUPPORTED_MODE);
    else if (systemState == STATE_ARMED) enterFault(FAULT_INVALID_COMMAND);
    return;
  }

  haveCommandSequence = true;
  lastCommandSequence = pkt.sequence;
  lastValidCommandMs = millis();

  const DynoMode requestedMode = (DynoMode)pkt.mode;
  const bool modeChanged = requestedMode != mode;

  mode = requestedMode;
  manualFlowPct = pkt.manualFlowPct;
  manualPressurePct = pkt.manualPressurePct;
  manualThrottlePct = pkt.manualThrottlePct;
  targetTorqueNm = pkt.targetTorqueNm;
  targetPumpRpm = pkt.targetPumpRpm;
  targetEngineRpm = pkt.targetEngineRpm;

  if (modeChanged) {
    resetPidState();
    if (systemState == STATE_ARMED) {
      // Every live mode transition passes through maximum braking. The next
      // scheduled controller update deliberately leaves that state.
      applyFailSafeOutputs();
    }
  }

  if (pkt.emergency != 0) {
    enterEstop();
    return;
  }

  if (pkt.arm == 0) {
    if (systemState == STATE_ARMED) enterDisarmed();
    return;
  }

  if (systemState != STATE_DISARMED) return;
  if (!g_bleClientConnected) return;
  if (digitalRead(EMERGENCY_PIN) == LOW) return;

  if (mode == MODE_TORQUE && (!scaleConnected || !torqueValid)) {
    enterFault(scaleConnected ? FAULT_TORQUE_SENSOR : FAULT_SCALE_UNAVAILABLE);
    return;
  }

  systemState = STATE_ARMED;
  sensorGraceUntilMs = millis() + SENSOR_STARTUP_GRACE_MS;
  resetPidState();
  Serial.println("[SAFETY] System ARMED");
}

void evaluateSafety() {
  if (systemState != STATE_ARMED) return;

  const uint32_t now = millis();

  if (!g_bleClientConnected) {
    enterFault(FAULT_BLE_DISCONNECTED);
    return;
  }

  if ((uint32_t)(now - lastValidCommandMs) > COMMAND_TIMEOUT_MS) {
    enterFault(FAULT_COMMAND_TIMEOUT);
    return;
  }

  if (pumpRpmValid && pumpRpm >= PUMP_HARD_TRIP_RPM) {
    enterFault(FAULT_PUMP_OVERSPEED);
    return;
  }

  if (engineRpmValid && engineRpm >= ENGINE_HARD_TRIP_RPM) {
    enterFault(FAULT_ENGINE_OVERSPEED);
    return;
  }

  if (torqueValid && fabsf(torqueNm) >= TORQUE_HARD_TRIP_NM) {
    enterFault(FAULT_TORQUE_OVERRANGE);
    return;
  }

  if (pumpRpmValid && pumpRpm >= PUMP_SOFT_LIMIT_RPM) {
    if (overspeedStartedMs == 0) overspeedStartedMs = now;
    if ((uint32_t)(now - overspeedStartedMs) >= PUMP_OVERSPEED_LATCH_MS) {
      enterFault(FAULT_PUMP_OVERSPEED);
      return;
    }
  } else {
    overspeedStartedMs = 0;
  }

  const bool graceExpired = (int32_t)(now - sensorGraceUntilMs) >= 0;
  const bool motionRequested =
      (mode == MODE_MANUAL && manualThrottlePct > 15) ||
      (mode != MODE_MANUAL && targetEngineRpm > 1200.0f);

  if (graceExpired && motionRequested) {
    if (!engineRpmValid) {
      enterFault(FAULT_ENGINE_RPM_SENSOR);
      return;
    }
    if (!pumpRpmValid) {
      enterFault(FAULT_PUMP_RPM_SENSOR);
      return;
    }
  }

  if (mode == MODE_TORQUE && !torqueValid) {
    enterFault(FAULT_TORQUE_SENSOR);
  }
}

// ==========================================================================
// Control
// ==========================================================================

void deltaPidControl(float error,
                     float& previousError,
                     float& previousPreviousError,
                     float P,
                     float I,
                     float D,
                     float& output,
                     float minimum,
                     float maximum,
                     float dtSeconds) {
  if (dtSeconds <= 0.0f) return;

  const float deltaP = P * (error - previousError);
  const float deltaI = I * error * dtSeconds;
  const float deltaD = D *
      ((error - 2.0f * previousError + previousPreviousError) / dtSeconds);

  output = constrain(output + deltaP + deltaI + deltaD, minimum, maximum);
  previousPreviousError = previousError;
  previousError = error;
}

void pumpControl(float dtSeconds) {
  switch (mode) {
    case MODE_MANUAL:
      flowValveEnabled = true;
      pressureValveEnabled = true;
      flowValveValue = percentToDuty(manualFlowPct);

      // AP04G2YR is inverse: zero current is maximum pressure.
      pressureValveValue = percentToDuty(100U - manualPressurePct);
      break;

    case MODE_TORQUE: {
      flowValveEnabled = false;
      pressureValveEnabled = true;
      flowValveValue = 0.0f;

      // Increasing pressure-valve current lowers pressure and torque. Using
      // measured-target error therefore raises duty when torque is too high
      // and lowers duty toward maximum braking when torque is too low.
      const float error = torqueNm - targetTorqueNm;
      if (!pressurePidInitialized) {
        pressurePreviousError = error;
        pressurePreviousPreviousError = error;
        pressurePidInitialized = true;
      }

      deltaPidControl(error,
                      pressurePreviousError,
                      pressurePreviousPreviousError,
                      pressureP,
                      pressureI,
                      pressureD,
                      pressureValveValue,
                      0.0f,
                      (float)VALVE_MAX_DUTY,
                      dtSeconds);
      break;
    }

    case MODE_RPM: {
      pressureValveEnabled = false;
      flowValveEnabled = true;
      pressureValveValue = 0.0f; // Maximum braking pressure.

      const float error = targetPumpRpm - pumpRpm;
      if (!flowPidInitialized) {
        flowPreviousError = error;
        flowPreviousPreviousError = error;
        flowPidInitialized = true;
      }

      deltaPidControl(error,
                      flowPreviousError,
                      flowPreviousPreviousError,
                      flowP,
                      flowI,
                      flowD,
                      flowValveValue,
                      0.0f,
                      (float)VALVE_MAX_DUTY,
                      dtSeconds);
      break;
    }

    default:
      enterFault(FAULT_UNSUPPORTED_MODE);
      break;
  }
}

void engineRpmControl(float dtSeconds) {
  if (mode == MODE_MANUAL) {
    engineThrottlePulseUs = ENGINE_THROTTLE_MIN_US +
        ((ENGINE_THROTTLE_MAX_US - ENGINE_THROTTLE_MIN_US) *
         constrain((float)manualThrottlePct, 0.0f, 100.0f) / 100.0f);
    return;
  }

  const float error = targetEngineRpm - engineRpm;
  if (!enginePidInitialized) {
    enginePreviousError = error;
    enginePreviousPreviousError = error;
    enginePidInitialized = true;
  }

  deltaPidControl(error,
                  enginePreviousError,
                  enginePreviousPreviousError,
                  engineP,
                  engineI,
                  engineD,
                  engineThrottlePulseUs,
                  (float)ENGINE_THROTTLE_MIN_US,
                  (float)ENGINE_THROTTLE_MAX_US,
                  dtSeconds);
}

void applyOverspeedOverride() {
  softOverspeedActive = false;
  if (!pumpRpmValid || pumpRpm < PUMP_SOFT_LIMIT_START_RPM) return;

  softOverspeedActive = true;
  const float range = PUMP_SOFT_LIMIT_RPM - PUMP_SOFT_LIMIT_START_RPM;
  const float factor = constrain(
      (PUMP_SOFT_LIMIT_RPM - pumpRpm) / range, 0.0f, 1.0f);

  // Move continuously toward the system's maximum-braking state.
  flowValveValue *= factor;
  pressureValveValue *= factor;
  engineThrottlePulseUs = ENGINE_THROTTLE_MIN_US +
      (engineThrottlePulseUs - ENGINE_THROTTLE_MIN_US) * factor;
}

void runControlLoop(float dtSeconds) {
  if (systemState != STATE_ARMED) {
    applyFailSafeOutputs();
    return;
  }

  const float previousFlowValue = flowValveValue;
  const float previousPressureValue = pressureValveValue;
  const float previousThrottlePulseUs = engineThrottlePulseUs;

  pumpControl(dtSeconds);
  if (systemState != STATE_ARMED) return;

  engineRpmControl(dtSeconds);

  const float valveStep = VALVE_DUTY_SLEW_PER_SECOND * dtSeconds;
  const float throttleStep = THROTTLE_SLEW_US_PER_SECOND * dtSeconds;
  flowValveValue = constrain(flowValveValue,
                             previousFlowValue - valveStep,
                             previousFlowValue + valveStep);
  pressureValveValue = constrain(pressureValveValue,
                                 previousPressureValue - valveStep,
                                 previousPressureValue + valveStep);
  engineThrottlePulseUs = constrain(engineThrottlePulseUs,
                                    previousThrottlePulseUs - throttleStep,
                                    previousThrottlePulseUs + throttleStep);

  applyOverspeedOverride();
  writeActuatorOutputs();
}

void writeActuatorOutputs() {
  flowValveValue = constrain(flowValveValue, 0.0f, (float)VALVE_MAX_DUTY);
  pressureValveValue = constrain(pressureValveValue, 0.0f, (float)VALVE_MAX_DUTY);
  engineThrottlePulseUs = constrain(engineThrottlePulseUs,
                                    (float)ENGINE_THROTTLE_MIN_US,
                                    (float)ENGINE_THROTTLE_MAX_US);

  digitalWrite(FLOW_VALVE_ENABLE_PIN, flowValveEnabled ? HIGH : LOW);
  digitalWrite(PRESSURE_VALVE_ENABLE_PIN, pressureValveEnabled ? HIGH : LOW);

  ledcWrite(FLOW_VALVE_CHANNEL, (uint32_t)lroundf(flowValveValue));
  ledcWrite(PRESSURE_VALVE_CHANNEL, (uint32_t)lroundf(pressureValveValue));
  ledcWrite(ENGINE_THROTTLE_CHANNEL,
            engineDutyFromPulse((uint32_t)lroundf(engineThrottlePulseUs)));
}

// ==========================================================================
// BLE
// ==========================================================================

class DynoServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    (void)server;
    g_bleClientConnected = true;
    g_newBleSessionRequested = true;
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(BLEServer* server) override {
    g_bleClientConnected = false;
    g_bleDisconnectRequested = true;
    Serial.println("[BLE] Client disconnected");

    BLEAdvertising* advertising = server->getAdvertising();
    advertising->start();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    const size_t length = characteristic->getLength();
    uint8_t* data = characteristic->getData();

    if (data == nullptr || length != sizeof(CommandPacket)) {
      g_invalidCommandReceived = true;
      return;
    }

    CommandPacket pkt{};
    memcpy(&pkt, data, sizeof(pkt));

    if (!dynoCommandPacketValid(pkt)) {
      g_invalidCommandReceived = true;
      return;
    }

    // Emergency is honored as soon as a structurally valid packet arrives.
    if (pkt.emergency != 0) g_remoteEmergencyRequested = true;

    portENTER_CRITICAL(&g_commandMux);
    g_pendingCommand = pkt;
    g_pendingCommandAvailable = true;
    portEXIT_CRITICAL(&g_commandMux);
  }
};

void setupBle() {
  BLEDevice::init(DYNO_BLE_DEVICE_NAME);
  BLEDevice::setMTU(517);

  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new DynoServerCallbacks());

  BLEService* service = g_server->createService(DYNO_SERVICE_UUID);

  g_commandCharacteristic = service->createCharacteristic(
      DYNO_CMD_UUID,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR);
  g_commandCharacteristic->setCallbacks(new CommandCallbacks());

  g_telemetryCharacteristic = service->createCharacteristic(
      DYNO_TEL_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY);
  g_telemetryCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* advertising = g_server->getAdvertising();
  advertising->addServiceUUID(DYNO_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("[BLE] DynoController advertising");
}

void sendTelemetry() {
  TelemetryPacket pkt{};
  pkt.magic = DYNO_PACKET_MAGIC;
  pkt.version = DYNO_PACKET_VERSION;
  pkt.state = (uint8_t)systemState;
  pkt.mode = (uint8_t)mode;
  pkt.faultFlags = faultFlags;
  pkt.lastCommandSequence = lastCommandSequence;

  if (systemState == STATE_ARMED) pkt.flags |= TEL_FLAG_ARMED;
  if (systemState == STATE_ESTOP) pkt.flags |= TEL_FLAG_ESTOP;
  if (g_bleClientConnected) pkt.flags |= TEL_FLAG_BLE_CONNECTED;
  if (scaleConnected) pkt.flags |= TEL_FLAG_SCALE_CONNECTED;
  if (engineRpmValid) pkt.flags |= TEL_FLAG_ENGINE_RPM_VALID;
  if (pumpRpmValid) pkt.flags |= TEL_FLAG_PUMP_RPM_VALID;
  if (torqueValid) pkt.flags |= TEL_FLAG_TORQUE_VALID;
  if (softOverspeedActive) pkt.flags |= TEL_FLAG_SOFT_OVERSPEED;

  pkt.engineRpm = engineRpm;
  pkt.targetEngineRpm = targetEngineRpm;
  pkt.pumpRpm = pumpRpm;
  pkt.targetPumpRpm = targetPumpRpm;
  pkt.torqueNm = torqueNm;
  pkt.targetTorqueNm = targetTorqueNm;
  pkt.flowValveDutyPct = dutyToPercent(flowValveValue);
  pkt.pressureValveDutyPct = dutyToPercent(pressureValveValue);
  pkt.throttlePct = constrain(
      (engineThrottlePulseUs - ENGINE_THROTTLE_MIN_US) * 100.0f /
      (ENGINE_THROTTLE_MAX_US - ENGINE_THROTTLE_MIN_US),
      0.0f,
      100.0f);
  pkt.powerKw = pumpRpm * torqueNm / 9549.2966f;

  dynoFinalizeTelemetryPacket(pkt);

  if (g_telemetryCharacteristic != nullptr) {
    g_telemetryCharacteristic->setValue((uint8_t*)&pkt, sizeof(pkt));
    if (g_bleClientConnected) g_telemetryCharacteristic->notify();
  }
}

// ==========================================================================
// Status indication
// ==========================================================================

void updateLed() {
  const uint32_t now = millis();

  if (systemState == STATE_ESTOP || systemState == STATE_FAULT) {
    if ((uint32_t)(now - lastLedMs) >= 250) {
      lastLedMs = now;
      setLedColor(currentLedColor == LED_RED ? LED_OFF : LED_RED);
    }
    return;
  }

  if (systemState == STATE_ARMED) {
    if (currentLedColor != LED_GREEN) setLedColor(LED_GREEN);
  } else if (g_bleClientConnected) {
    if (currentLedColor != LED_BLUE) setLedColor(LED_BLUE);
  } else {
    if (currentLedColor != LED_YELLOW) setLedColor(LED_YELLOW);
  }
}

void printStatus() {
  Serial.print("State: ");
  Serial.print((int)systemState);
  Serial.print(" | Mode: ");
  Serial.print((int)mode);
  Serial.print(" | Engine RPM: ");
  Serial.print(engineRpm, 0);
  Serial.print(engineRpmValid ? " valid" : " invalid");
  Serial.print(" | Pump RPM: ");
  Serial.print(pumpRpm, 0);
  Serial.print(pumpRpmValid ? " valid" : " invalid");
  Serial.print(" | Torque: ");
  Serial.print(torqueNm, 1);
  Serial.print(torqueValid ? " valid" : " invalid");
  Serial.print(" | Flow duty: ");
  Serial.print(dutyToPercent(flowValveValue), 1);
  Serial.print("% | Pressure duty: ");
  Serial.print(dutyToPercent(pressureValveValue), 1);
  Serial.print("% | Throttle: ");
  Serial.print((engineThrottlePulseUs - ENGINE_THROTTLE_MIN_US) * 100.0f /
               (ENGINE_THROTTLE_MAX_US - ENGINE_THROTTLE_MIN_US), 1);
  Serial.print("% | Faults: 0x");
  Serial.println(faultFlags, HEX);
}

// ==========================================================================
// Arduino setup and loop
// ==========================================================================

void setup() {
  Serial.begin(115200);

  led.begin();
  led.setBrightness(100);
  setLedColor(LED_RED);

  // Configure only pins owned by this firmware.
  pinMode(EMERGENCY_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(ENGINE_PULSE_SENSOR_PIN, INPUT_PULLUP);
  pinMode(PUMP_PULSE_SENSOR_PIN, INPUT_PULLUP);

  pinMode(FLOW_VALVE_PIN, OUTPUT);
  pinMode(FLOW_VALVE_ENABLE_PIN, OUTPUT);
  pinMode(PRESSURE_VALVE_PIN, OUTPUT);
  pinMode(PRESSURE_VALVE_ENABLE_PIN, OUTPUT);
  pinMode(ENGINE_THROTTLE_PIN, OUTPUT);

  digitalWrite(FLOW_VALVE_ENABLE_PIN, LOW);
  digitalWrite(PRESSURE_VALVE_ENABLE_PIN, LOW);

  ledcSetup(FLOW_VALVE_CHANNEL, FLOW_VALVE_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(PRESSURE_VALVE_CHANNEL, PRESSURE_VALVE_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(ENGINE_THROTTLE_CHANNEL, ENGINE_THROTTLE_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(FLOW_VALVE_PIN, FLOW_VALVE_CHANNEL);
  ledcAttachPin(PRESSURE_VALVE_PIN, PRESSURE_VALVE_CHANNEL);
  ledcAttachPin(ENGINE_THROTTLE_PIN, ENGINE_THROTTLE_CHANNEL);

  applyFailSafeOutputs();

  attachInterrupt(digitalPinToInterrupt(ENGINE_PULSE_SENSOR_PIN), enginePulseIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(PUMP_PULSE_SENSOR_PIN), pumpPulseIsr, FALLING);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_PIN), emergencyIsr, FALLING);

  Serial.println("Dynamometer initializing in maximum-braking state...");

  // Observe long enough to detect a rotating shaft before taring.
  delay(1100);
  updateSensors();
  while ((engineRpmValid && engineRpm > 0.0f) ||
         (pumpRpmValid && pumpRpm > 0.0f)) {
    applyFailSafeOutputs();
    setLedColor(LED_RED);
    Serial.println("Waiting for engine and pump to stop before load-cell tare...");
    delay(100);
    updateSensors();
  }

  Serial.print("Initializing HX711... ");
  scale.begin(LOAD_CELL_DOUT_PIN, LOAD_CELL_CLK_PIN, true, false);
  if (scale.wait_ready_timeout(1000)) {
    scaleConnected = true;
    scale.set_scale(LOAD_CELL_SCALE_FACTOR);
    scale.tare(50);
    lastTorqueSampleMs = millis();
    getTorque();
    Serial.println("detected and tared");
  } else {
    scaleConnected = false;
    torqueValid = false;
    Serial.println("not detected; torque mode unavailable");
  }

  setupBle();

  faultFlags = FAULT_NONE;
  enterDisarmed();
  lastControlUs = micros();
  Serial.println("Dynamometer initialized and DISARMED");
}

void loop() {
  processSafetyInputs();
  processPendingCommand();
  updateSensors();
  evaluateSafety();

  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
    // Advance by one nominal interval. If badly delayed, resynchronize rather
    // than executing a burst of stale controller updates.
    lastControlUs += CONTROL_PERIOD_US;
    if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
      lastControlUs = nowUs;
    }

    runControlLoop(CONTROL_PERIOD_US / 1000000.0f);
  }

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastTelemetryMs) >= BLE_TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    sendTelemetry();
  }

  if ((uint32_t)(nowMs - lastStatusMs) >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    printStatus();
  }

  updateLed();
}
