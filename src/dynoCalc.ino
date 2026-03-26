#include <Arduino.h>
#include <HX711.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "dyno_protocol.h"

// ============================
// Hardware pins
// ============================
#define LED_PIN 48
#define LED_COUNT 1

#define emergencyPin 14
#define resetPin 15

#define DOUT 4
#define CLK 5

#define pumpPulseSensor 9
#define flowValvePin 10
#define flowValveEnablePin 11
#define pressureValvePin 12
#define pressureValveEnablePin 13

#define enginePulseSensor 6
#define engineThrottlePin 7

// ============================
// PWM / actuator config
// ============================
int flowValveFrequency = 250;
int pressureValveFrequency = 250;
int pwmResolution = 12;

// ============================
// Runtime objects
// ============================
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
HX711 scale;
Servo engineThrottle;

BLEServer* g_server = nullptr;
BLECharacteristic* g_cmdChar = nullptr;
BLECharacteristic* g_telChar = nullptr;
volatile bool g_bleClientConnected = false;

// ============================
// LED state
// ============================
enum ledColor { OFF, RED, YELLOW, GREEN, BLUE };
ledColor currentColor = OFF;

// ============================
// Command/state variables
// ============================
bool emergency = false;
bool scaleConnected = false;
int mode = MODE_RPM;
int g_lastCommandedMode = MODE_RPM;

int manualPressure = 0;
int manualFlow = 0;

int minPressureValve = 0;
int minFlowValve = 0;
int maxPressureValve = 0;
int maxFlowValve = 0;

int primarySprocket = 28;
int secondarySprocket = 40;

float targetTorque = 60.0f;
float targetRpm = 4200.0f;
float targetEngineRpm = 3800.0f;

// ============================
// Valve / throttle outputs
// ============================
float flowValveValue = 0.0f;
float pressureValveValue = 0.0f;

int engineThrottleMin = 500;
int engineThrottleMax = 2500;
float engineThrottleValue = 500.0f;
int servoMinAngle = 20;
int servoMaxAngle = 110;

// ============================
// Sensor config / telemetry
// ============================
float scaleFactor = 2280.0f;
float engineRpm = 0.0f;
float pumpRpm = 0.0f;
float targetPumpRpm = 0.0f;
float torque = 0.0f;
float powerKw = 0.0f;

volatile unsigned long enginePulse = 0;
volatile unsigned long prevEnginePulse = 0;
volatile unsigned long pumpPulse = 0;
volatile unsigned long prevPumpPulse = 0;

int engineMagnets = 1;
int pumpMagnets = 3;
const int maxPumpRpm = 3000;
int rpmLimiterDeadband = 300;

// ============================
// PID variables
// ============================
float pressureP = 1.0f;
float pressureI = 0.2f;
float pressureD = 0.05f;
float pressureIntegral = 0.0f;
float pressurePreviousError = 0.0f;
float pressurePreviousPreviousError = 0.0f;
unsigned long pressurePreviousTime = 0;

float flowP = 0.1f;
float flowI = 0.025f;
float flowD = 0.01f;
float flowIntegral = 0.0f;
float flowPreviousError = 0.0f;
float flowPreviousPreviousError = 0.0f;
unsigned long flowPreviousTime = 0;

float engineP = 0.1f;
float engineI = 0.02f;
float engineD = 0.005f;
float engineIntegral = 0.0f;
float enginePreviousError = 0.0f;
float enginePreviousPreviousError = 0.0f;
unsigned long enginePreviousTime = 0;

// ============================
// Timing
// ============================
unsigned long statusPreviousMillis = 0;
unsigned long sensorPreviousMillis = 0;
unsigned long blePreviousMillis = 0;
unsigned long g_lastBleDebugMs = 0;

const unsigned long SENSOR_PERIOD_MS = 20;
const unsigned long STATUS_PERIOD_MS = 250;
const unsigned long BLE_TELEMETRY_PERIOD_MS = 50;

#define FAKE_TELEMETRY_TEST 1

#if FAKE_TELEMETRY_TEST
unsigned long fakePreviousMs = 0;
#endif

// ============================
// Helpers
// ============================
static float pctToPwm(int pct, int maxPwm) {
  pct = constrain(pct, 0, 100);
  return (maxPwm * pct) / 100.0f;
}

static float pwmToPct(float pwm, int maxPwm) {
  if (maxPwm <= 0) return 0.0f;
  return constrain((pwm * 100.0f) / maxPwm, 0.0f, 100.0f);
}

static float throttleToPct(float us) {
  const float span = float(engineThrottleMax - engineThrottleMin);
  if (span <= 0.0f) return 0.0f;
  return constrain((us - engineThrottleMin) * 100.0f / span, 0.0f, 100.0f);
}

static const char* modeToStr(uint8_t m) {
  switch (m) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_TORQUE: return "TORQUE";
    case MODE_RPM:    return "RPM";
    case MODE_CVT:    return "CVT";
    default:          return "UNKNOWN";
  }
}

static void printCommandPacket(const CommandPacket& pkt) {
  Serial.println("---- CMD RX ----");
  Serial.print("magic: 0x"); Serial.println(pkt.magic, HEX);
  Serial.print("version: "); Serial.println(pkt.version);
  Serial.print("emergency: "); Serial.println(pkt.emergency);
  Serial.print("mode: "); Serial.print(pkt.mode); Serial.print(" ("); Serial.print(modeToStr(pkt.mode)); Serial.println(")");
  Serial.print("manualFlowPct: "); Serial.println(pkt.manualFlowPct);
  Serial.print("manualPressurePct: "); Serial.println(pkt.manualPressurePct);
  Serial.print("targetTorque: "); Serial.println(pkt.targetTorque);
  Serial.print("targetRpm: "); Serial.println(pkt.targetRpm);
  Serial.print("targetEngineRpm: "); Serial.println(pkt.targetEngineRpm);
  Serial.println("----------------");
}

static float fakeFirstOrder(float current, float target, float ratePerSec, float dt) {
  float alpha = constrain(ratePerSec * dt, 0.0f, 1.0f);
  return current + (target - current) * alpha;
}

void colorSet(ledColor color) {
  if (color == RED) {
    led.setPixelColor(0, led.Color(255, 0, 0));
  } else if (color == YELLOW) {
    led.setPixelColor(0, led.Color(255, 255, 0));
  } else if (color == GREEN) {
    led.setPixelColor(0, led.Color(0, 255, 0));
  } else if (color == BLUE) {
    led.setPixelColor(0, led.Color(0, 0, 255));
  } else {
    led.setPixelColor(0, led.Color(0, 0, 0));
  }
  currentColor = color;
  led.show();
}

void engineMagRead() {
  prevEnginePulse = enginePulse;
  enginePulse = micros();
}

void pumpMagRead() {
  prevPumpPulse = pumpPulse;
  pumpPulse = micros();
}

void getEngineRpm() {
  const unsigned long pulse = enginePulse;
  const unsigned long prevPulse = prevEnginePulse;

  if (micros() - pulse > 1000000UL) {
    engineRpm = 0.0f;
  } else if (pulse > prevPulse && (pulse - prevPulse) > 0) {
    const double magFreq = 60000000.0 / double(pulse - prevPulse);
    engineRpm = magFreq / engineMagnets;
  }
}

void getPumpRpm() {
  const unsigned long pulse = pumpPulse;
  const unsigned long prevPulse = prevPumpPulse;

  if (micros() - pulse > 1000000UL) {
    pumpRpm = 0.0f;
  } else if (pulse > prevPulse && (pulse - prevPulse) > 0) {
    const double magFreq = 60000000.0 / double(pulse - prevPulse);
    pumpRpm = magFreq / pumpMagnets;
  }
}

void getTorque() {
  if (scaleConnected && scale.is_ready()) {
    torque = scale.get_units(3);
  }
}

void deltaPIDControl(float error, float &previousError, float &previousPreviousError,
                     float P, float I, float D, float &outputValue,
                     int minVal, int maxVal, unsigned long &previousTime) {
  const unsigned long now = micros();
  float dt = (now - previousTime) / 1000000.0f;
  if (dt <= 0.0f) dt = 0.000001f;
  previousTime = now;

  const float deltaP = P * (error - previousError);
  const float deltaI = I * error * dt;
  const float deltaD = D * ((error - 2.0f * previousError + previousPreviousError) / dt);

  outputValue += (deltaP + deltaI + deltaD);
  outputValue = constrain(outputValue, float(minVal), float(maxVal));

  previousPreviousError = previousError;
  previousError = error;
}

void torqueControl() {
  getTorque();
  const float error = targetTorque - torque;

  if (pressurePreviousTime == 0 || micros() > pressurePreviousTime + 1000000UL) {
    pressurePreviousTime = micros();
    pressureIntegral = 0.0f;
    pressurePreviousError = error;
    pressurePreviousPreviousError = error;
  }

  deltaPIDControl(error, pressurePreviousError, pressurePreviousPreviousError,
                  pressureP, pressureI, pressureD, pressureValveValue,
                  minPressureValve, maxPressureValve, pressurePreviousTime);

  getPumpRpm();
  float rpmLimiter = 1.0f;
  if (pumpRpm > maxPumpRpm - rpmLimiterDeadband) {
    rpmLimiter = 1.0f - ((pumpRpm - (maxPumpRpm - rpmLimiterDeadband)) / (rpmLimiterDeadband * 2.0f));
    rpmLimiter = constrain(rpmLimiter, 0.0f, 1.0f);
  }
  pressureValveValue *= rpmLimiter;
}

void pumpRpmControl() {
  getPumpRpm();

  targetPumpRpm = (targetRpm * primarySprocket) / float(secondarySprocket);
  targetPumpRpm = constrain(targetPumpRpm, 0.0f, float(maxPumpRpm));

  const float error = targetPumpRpm - pumpRpm;

  if (flowPreviousTime == 0 || micros() > flowPreviousTime + 1000000UL) {
    flowPreviousTime = micros();
    flowIntegral = 0.0f;
    flowPreviousError = error;
    flowPreviousPreviousError = error;
  }

  deltaPIDControl(error, flowPreviousError, flowPreviousPreviousError,
                  flowP, flowI, flowD, flowValveValue,
                  minFlowValve, maxFlowValve, flowPreviousTime);
}

void engineRpmControl() {
  getEngineRpm();
  const float error = targetEngineRpm - engineRpm;

  if (enginePreviousTime == 0 || micros() > enginePreviousTime + 1000000UL) {
    enginePreviousTime = micros();
    engineIntegral = 0.0f;
    enginePreviousError = error;
    enginePreviousPreviousError = error;
  }

  deltaPIDControl(error, enginePreviousError, enginePreviousPreviousError,
                  engineP, engineI, engineD, engineThrottleValue,
                  engineThrottleMin, engineThrottleMax, enginePreviousTime);

  engineThrottle.writeMicroseconds((int)engineThrottleValue);
}

void pumpControl() {
  if (mode == MODE_MANUAL) {
    digitalWrite(flowValveEnablePin, HIGH);
    digitalWrite(pressureValveEnablePin, HIGH);
    flowValveValue = pctToPwm(manualFlow, maxFlowValve);
    pressureValveValue = pctToPwm(manualPressure, maxPressureValve);
  } else if (mode == MODE_TORQUE && scaleConnected) {
    flowValveValue = 0.0f;
    digitalWrite(flowValveEnablePin, LOW);
    digitalWrite(pressureValveEnablePin, HIGH);
    torqueControl();
  } else if (mode == MODE_RPM || mode == MODE_CVT) {
    pressureValveValue = 0.0f;
    digitalWrite(pressureValveEnablePin, LOW);
    digitalWrite(flowValveEnablePin, HIGH);
    pumpRpmControl();
  } else {
    flowValveValue = 0.0f;
    pressureValveValue = 0.0f;
    digitalWrite(flowValveEnablePin, LOW);
    digitalWrite(pressureValveEnablePin, LOW);
  }

  ledcWrite(flowValvePin, (uint32_t)flowValveValue);
  ledcWrite(pressureValvePin, (uint32_t)pressureValveValue);
}

void updateDerivedValues() {
  powerKw = (torque * pumpRpm * 2.0f * PI / 60.0f) / 1000.0f;
}

#if FAKE_TELEMETRY_TEST
void updateFakeTelemetry(float dt) {
  scaleConnected = true;

  const float throttlePct = throttleToPct(engineThrottleValue) / 100.0f;
  const float flowPct = pwmToPct(flowValveValue, maxFlowValve) / 100.0f;
  const float pressurePct = pwmToPct(pressureValveValue, maxPressureValve) / 100.0f;

  float desiredEngine = engineRpm;
  float desiredPump = pumpRpm;
  float desiredTorque = torque;

  switch (mode) {
    case MODE_MANUAL:
      desiredEngine = 900.0f + throttlePct * 5200.0f;
      desiredPump = flowPct * float(maxPumpRpm);
      desiredTorque = 4.0f + pressurePct * 90.0f + flowPct * 8.0f;
      break;

    case MODE_TORQUE:
      desiredEngine = 1000.0f + throttlePct * 5000.0f;
      desiredPump = 300.0f + pressurePct * float(maxPumpRpm - 300);
      desiredTorque = 8.0f + pressurePct * 72.0f;
      break;

    case MODE_RPM:
      desiredEngine = 900.0f + throttlePct * 5200.0f;
      desiredPump = flowPct * float(maxPumpRpm);
      desiredTorque = 6.0f + 0.018f * desiredPump + 0.010f * pressurePct * 100.0f;
      break;

    case MODE_CVT:
      desiredEngine = 900.0f + throttlePct * 5200.0f;
      desiredPump = flowPct * float(maxPumpRpm);
      desiredTorque = 8.0f + 0.020f * desiredPump + pressurePct * 20.0f;
      break;

    default:
      break;
  }

  if (emergency) {
    desiredEngine = 0.0f;
    desiredPump = 0.0f;
    desiredTorque = 0.0f;
  }

  engineRpm = fakeFirstOrder(engineRpm, constrain(desiredEngine, 0.0f, 6500.0f), 3.0f, dt);
  pumpRpm = fakeFirstOrder(pumpRpm, constrain(desiredPump, 0.0f, float(maxPumpRpm)), 4.0f, dt);
  torque = fakeFirstOrder(torque, constrain(desiredTorque, 0.0f, 120.0f), 3.5f, dt);

  if (mode == MODE_CVT) {
    targetPumpRpm = constrain(targetPumpRpm, 0.0f, float(maxPumpRpm));
  } else {
    targetPumpRpm = (targetRpm * primarySprocket) / float(secondarySprocket);
    targetPumpRpm = constrain(targetPumpRpm, 0.0f, float(maxPumpRpm));
  }

  updateDerivedValues();
}
#endif

void resetControllers() {
  flowIntegral = pressureIntegral = engineIntegral = 0.0f;
  flowPreviousError = pressurePreviousError = enginePreviousError = 0.0f;
  flowPreviousPreviousError = pressurePreviousPreviousError = enginePreviousPreviousError = 0.0f;
  flowPreviousTime = pressurePreviousTime = enginePreviousTime = 0;
}

void applyCommandPacket(const CommandPacket& pkt) {
  if (!dynoCommandPacketValid(pkt)) return;

  emergency = pkt.emergency != 0;

  int newMode = pkt.mode;
  bool modeChanged = (newMode != mode);
  mode = newMode;
  g_lastCommandedMode = newMode;

  manualFlow = constrain((int)pkt.manualFlowPct, 0, 100);
  manualPressure = constrain((int)pkt.manualPressurePct, 0, 100);
  targetTorque = pkt.targetTorque;
  targetRpm = pkt.targetRpm;
  targetEngineRpm = pkt.targetEngineRpm;

  if (modeChanged) {
    resetControllers();
  }

  Serial.print("[BLE][CTRL] Applied command | mode=");
  Serial.print(modeToStr(mode));
  Serial.print(" | targetRpm=");
  Serial.print(targetRpm);
  Serial.print(" | targetEngineRpm=");
  Serial.print(targetEngineRpm);
  Serial.print(" | targetTorque=");
  Serial.print(targetTorque);
  Serial.print(" | manualFlow=");
  Serial.print(manualFlow);
  Serial.print(" | manualPressure=");
  Serial.print(manualPressure);
  Serial.print(" | emergency=");
  Serial.println(emergency ? 1 : 0);
}

class DynoServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    g_bleClientConnected = true;
    Serial.println("[BLE][CTRL] Client connected");
    colorSet(BLUE);
  }

  void onDisconnect(BLEServer* pServer) override {
    g_bleClientConnected = false;
    Serial.println("[BLE][CTRL] Client disconnected");

    BLEAdvertising* adv = pServer->getAdvertising();
    adv->addServiceUUID(DYNO_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->start();

    Serial.println("[BLE][CTRL] Advertising restarted");
    colorSet(scaleConnected ? GREEN : YELLOW);
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String value = characteristic->getValue();
    Serial.print("[BLE][CTRL] Write received, len = ");
    Serial.println(value.length());

    if (value.length() != sizeof(CommandPacket)) {
      Serial.print("[BLE][CTRL] Bad packet size. Expected ");
      Serial.print(sizeof(CommandPacket));
      Serial.print(", got ");
      Serial.println(value.length());
      return;
    }

    CommandPacket pkt;
    memcpy(&pkt, value.c_str(), sizeof(pkt));

    if (!dynoCommandPacketValid(pkt)) {
      Serial.println("[BLE][CTRL] Invalid command packet magic/version");
      return;
    }

    printCommandPacket(pkt);
    applyCommandPacket(pkt);
  }
};

void setupBle() {
  Serial.println("[BLE][CTRL] BLE init starting...");
  Serial.print("[BLE][CTRL] Device name: ");
  Serial.println(DYNO_BLE_DEVICE_NAME);
  Serial.print("[BLE][CTRL] Service UUID: ");
  Serial.println(DYNO_SERVICE_UUID);
  Serial.print("[BLE][CTRL] Cmd UUID: ");
  Serial.println(DYNO_CMD_UUID);
  Serial.print("[BLE][CTRL] Tel UUID: ");
  Serial.println(DYNO_TEL_UUID);

  BLEDevice::init(DYNO_BLE_DEVICE_NAME);
  BLEDevice::setMTU(517);
  Serial.println("[BLE][CTRL] Preferred MTU set to 517");

  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new DynoServerCallbacks());

  BLEService* service = g_server->createService(DYNO_SERVICE_UUID);
  Serial.println("[BLE][CTRL] Service created");

  g_cmdChar = service->createCharacteristic(
      DYNO_CMD_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  g_cmdChar->setCallbacks(new CommandCallbacks());
  Serial.println("[BLE][CTRL] Command characteristic created");

  g_telChar = service->createCharacteristic(
      DYNO_TEL_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  g_telChar->addDescriptor(new BLE2902());
  Serial.println("[BLE][CTRL] Telemetry characteristic created");

  service->start();
  Serial.println("[BLE][CTRL] Service started");

  BLEAdvertising* adv = g_server->getAdvertising();
  adv->addServiceUUID(DYNO_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();
  Serial.println("[BLE][CTRL] Advertising started");
}

void sendTelemetry() {
  TelemetryPacket pkt{};
  pkt.magic = DYNO_PACKET_MAGIC;
  pkt.version = DYNO_PACKET_VERSION;
  pkt.emergency = emergency ? 1 : 0;
  pkt.scaleConnected = scaleConnected ? 1 : 0;
  pkt.mode = mode;
  pkt.engineRpm = engineRpm;
  pkt.targetEngineRpm = targetEngineRpm;
  pkt.pumpRpm = pumpRpm;
  pkt.targetPumpRpm = targetPumpRpm;
  pkt.torque = torque;
  pkt.targetTorque = targetTorque;
  pkt.flowValvePercent = pwmToPct(flowValveValue, maxFlowValve);
  pkt.pressureValvePercent = pwmToPct(pressureValveValue, maxPressureValve);
  pkt.throttlePercent = throttleToPct(engineThrottleValue);
  pkt.powerKw = powerKw;

  if (g_telChar != nullptr) {
    g_telChar->setValue((uint8_t*)&pkt, sizeof(pkt));
    if (g_bleClientConnected) {
      g_telChar->notify();
    }
  }

  if (millis() - g_lastBleDebugMs > 1000) {
    g_lastBleDebugMs = millis();
    Serial.print("[BLE][CTRL] Telemetry sent | client=");
    Serial.print(g_bleClientConnected ? "YES" : "NO");
    Serial.print(" | mode=");
    Serial.print(modeToStr(mode));
    Serial.print(" | engine=");
    Serial.print(engineRpm, 0);
    Serial.print(" | pump=");
    Serial.print(pumpRpm, 0);
    Serial.print(" | tq=");
    Serial.print(torque, 2);
    Serial.print(" | emergency=");
    Serial.println(emergency ? "1" : "0");
  }
}

void printStatus() {
  Serial.print("Mode: ");
  Serial.print(modeToStr(mode));
  Serial.print(" | BLE: "); Serial.print(g_bleClientConnected ? "CONNECTED" : "WAITING");
  Serial.print(" | E-RPM: "); Serial.print(engineRpm, 0);
  Serial.print("/"); Serial.print(targetEngineRpm, 0);
  Serial.print(" | P-RPM: "); Serial.print(pumpRpm, 0);
  Serial.print("/"); Serial.print(targetPumpRpm, 0);
  Serial.print(" | TQ: "); Serial.print(torque, 2);
  Serial.print("/"); Serial.print(targetTorque, 2);
  Serial.print(" | F%: "); Serial.print(pwmToPct(flowValveValue, maxFlowValve), 0);
  Serial.print(" | P%: "); Serial.print(pwmToPct(pressureValveValue, maxPressureValve), 0);
  Serial.print(" | THR%: "); Serial.print(throttleToPct(engineThrottleValue), 0);
  Serial.print(" | kW: "); Serial.println(powerKw, 2);
}

void emergencyStop() {
  while (digitalRead(emergencyPin) == LOW || emergency) {
    emergency = true;

    flowValveValue = 0.0f;
    pressureValveValue = 0.0f;
    engineThrottleValue = engineThrottleMin;

    ledcWrite(flowValvePin, 0);
    ledcWrite(pressureValvePin, 0);
    engineThrottle.writeMicroseconds(engineThrottleMin);

    digitalWrite(flowValveEnablePin, LOW);
    digitalWrite(pressureValveEnablePin, LOW);

    resetControllers();

    colorSet(currentColor == RED ? OFF : RED);
    sendTelemetry();
    delay(200);

    if (digitalRead(emergencyPin) == HIGH && digitalRead(resetPin) == HIGH) {
      emergency = false;
      colorSet(scaleConnected ? GREEN : YELLOW);
      Serial.println("[CTRL] Emergency cleared");
    }
  }
}

void setup() {
  led.begin();
  led.setBrightness(100);
  colorSet(RED);

  Serial.begin(115200);
  delay(300);
  Serial.println("Dyno controller starting...");

  Serial.print("sizeof(CommandPacket) = ");
  Serial.println(sizeof(CommandPacket));
  Serial.print("sizeof(TelemetryPacket) = ");
  Serial.println(sizeof(TelemetryPacket));

  maxPressureValve = int(0.98f * ((1 << pwmResolution) - 1));
  maxFlowValve = int(0.98f * ((1 << pwmResolution) - 1));
  engineThrottleValue = engineThrottleMin;

  pinMode(emergencyPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLDOWN);
  pinMode(flowValvePin, OUTPUT);
  pinMode(pressureValvePin, OUTPUT);
  pinMode(flowValveEnablePin, OUTPUT);
  pinMode(pressureValveEnablePin, OUTPUT);
  pinMode(enginePulseSensor, INPUT_PULLDOWN);
  pinMode(pumpPulseSensor, INPUT_PULLDOWN);
  pinMode(engineThrottlePin, OUTPUT);

  digitalWrite(flowValveEnablePin, LOW);
  digitalWrite(pressureValveEnablePin, LOW);

  ledcAttach(flowValvePin, flowValveFrequency, pwmResolution);
  ledcAttach(pressureValvePin, pressureValveFrequency, pwmResolution);
  ledcWrite(flowValvePin, 0);
  ledcWrite(pressureValvePin, 0);

  engineThrottle.attach(engineThrottlePin, engineThrottleMin, engineThrottleMax);
  engineThrottle.writeMicroseconds(engineThrottleMin);

  attachInterrupt(digitalPinToInterrupt(enginePulseSensor), engineMagRead, RISING);
  attachInterrupt(digitalPinToInterrupt(pumpPulseSensor), pumpMagRead, RISING);

  scale.begin(DOUT, CLK);
  bool hxReady = scale.is_ready();
  if (hxReady) {
    scale.set_scale(scaleFactor);
    scale.tare();
    Serial.println("HX711 ready.");
  } else {
    Serial.println("HX711 not detected - using fake torque if test mode is enabled.");
  }

#if FAKE_TELEMETRY_TEST
  scaleConnected = true;
  fakePreviousMs = millis();
  Serial.println("[TEST] Fake telemetry mode ENABLED");
#else
  scaleConnected = hxReady;
#endif

  setupBle();
  colorSet(scaleConnected ? GREEN : YELLOW);
}

void loop() {
  if (digitalRead(emergencyPin) == LOW || emergency) {
    emergencyStop();
  }

  const unsigned long now = millis();

#if FAKE_TELEMETRY_TEST
  if (now - sensorPreviousMillis >= SENSOR_PERIOD_MS) {
    float dt = (now - fakePreviousMs) / 1000.0f;
    if (dt <= 0.0f) dt = SENSOR_PERIOD_MS / 1000.0f;
    fakePreviousMs = now;
    sensorPreviousMillis = now;

    if (mode != MODE_MANUAL) {
      engineRpmControl();
    }
    pumpControl();
    updateFakeTelemetry(dt);

    digitalWrite(flowValveEnablePin, (mode == MODE_MANUAL || mode == MODE_TORQUE || mode == MODE_RPM || mode == MODE_CVT) ? HIGH : LOW);
    digitalWrite(pressureValveEnablePin, (mode == MODE_MANUAL || mode == MODE_TORQUE) ? HIGH : LOW);
    ledcWrite(flowValvePin, (uint32_t)flowValveValue);
    ledcWrite(pressureValvePin, (uint32_t)pressureValveValue);
    engineThrottle.writeMicroseconds((int)engineThrottleValue);
  }
#else
  if (now - sensorPreviousMillis >= SENSOR_PERIOD_MS) {
    sensorPreviousMillis = now;
    getEngineRpm();
    getPumpRpm();
    getTorque();
    updateDerivedValues();
  }

  if (mode != MODE_MANUAL) {
    engineRpmControl();
  }
  pumpControl();
  updateDerivedValues();
#endif

  if (now - blePreviousMillis >= BLE_TELEMETRY_PERIOD_MS) {
    blePreviousMillis = now;
    sendTelemetry();
  }

  if (now - statusPreviousMillis >= STATUS_PERIOD_MS) {
    statusPreviousMillis = now;
    printStatus();
  }
}

// FAKE VALUES GENERATED WHEN FAKE_TELEMETRY_TEST == 1
