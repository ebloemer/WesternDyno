#include <Arduino.h>
#include <HX711.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <HardwareSerial.h>

#include "dyno_protocol.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =====================================================
// LED
// =====================================================
#define LED_PIN   48
#define LED_COUNT 1

Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

enum ledColor { OFF, RED, YELLOW, GREEN, BLUE };
ledColor currentColor = OFF;

// =====================================================
// BLE globals
// =====================================================
BLEServer* g_server = nullptr;
BLECharacteristic* g_cmdChar = nullptr;
BLECharacteristic* g_telChar = nullptr;

volatile bool g_bleClientConnected = false;
unsigned long g_lastBleDebugMs = 0;
unsigned long blePreviousMillis = 0;

const unsigned long BLE_TELEMETRY_PERIOD_MS = 50;

// =====================================================
// Runtime / modes
// =====================================================
enum Mode { MANUAL, TORQUE, RPM, CVT };

#define emergencyPin 1   // normally-open switch to GND
#define resetPin     2   // normally-open switch to GND

bool emergency = false;   // FIX: do not start latched in emergency
unsigned long ledPreviousMillis = 0;

// =====================================================
// HX711
// =====================================================
HX711 scale;

#define DOUT 4
#define CLK  5

bool scaleConnected = false;
float scaleFactor = 2280.0f;

// =====================================================
// Pump pins
// =====================================================
#define pumpPulseSensor        9
#define flowValvePin           10
#define flowValveEnablePin     11
#define pressureValvePin       12
#define pressureValveEnablePin 13

#define flowValveChannel       1
#define pressureValveChannel   2
#define engineThrottleChannel  3

// =====================================================
// Engine pins
// =====================================================
#define enginePulseSensor  6
#define engineThrottlePin  7

// =====================================================
// PWM config
// =====================================================
int pwmResolution = 12;

float flowValveValue = 0.0f;
int flowValveFrequency = 250;

float pressureValveValue = 0.0f;
int pressureValveFrequency = 250;

int engineThrottleFrequency = 150;

// =====================================================
// Tunable parameters
// =====================================================
int mode = MANUAL;
int manualPressure = 0;
int manualFlow = 0;
int manualEngineThrottle = 0;   // retained in case you later add GUI throttle

const int PWM_MAX = (1 << 12) - 1;

int minPressureValve = 0;
int maxPressureValve = (int)(0.98f * PWM_MAX);

int minFlowValve = 0;
int maxFlowValve = (int)(0.98f * PWM_MAX);

int primarySprocket = 28;
int secondarySprocket = 40;

// =====================================================
// Pressure PID
// =====================================================
float pressureP = 1.0f;
float pressureI = 0.2f;
float pressureD = 0.05f;

float pressureIntegral = 0.0f;
float pressurePreviousError = 0.0f;
float pressurePreviousPreviousError = 0.0f;
unsigned long pressurePreviousTime = 0;

// =====================================================
// Flow PID
// =====================================================
float flowP = 0.25f;
float flowI = 0.0625f;
float flowD = 0.025f;

float flowIntegral = 0.0f;
float flowPreviousError = 0.0f;
float flowPreviousPreviousError = 0.0f;
unsigned long flowPreviousTime = 0;

// =====================================================
// Engine RPM control
// =====================================================
int engineThrottleMin = 500;   // pulse width us
int engineThrottleMax = 1200;  // pulse width us
float engineThrottleValue = 500.0f;

int servoMinAngle = 20;
int servoMaxAngle = 110;

volatile unsigned long enginePulse = 0;
volatile unsigned long prevEnginePulse = 0;
int engineMagnets = 1;

float engineP = 0.1f;
float engineI = 0.02f;
float engineD = 0.005f;

float engineIntegral = 0.0f;
float enginePreviousError = 0.0f;
float enginePreviousPreviousError = 0.0f;
unsigned long enginePreviousTime = 0;

// =====================================================
// Pump RPM control
// =====================================================
float targetPumpRpm = 0.0f;
const int maxPumpRpm = 3000;

volatile unsigned long pumpPulse = 0;
volatile unsigned long prevPumpPulse = 0;
int pumpMagnets = 3;

int rpmLimiterDeadband = 300;

// =====================================================
// Targets
// =====================================================
float targetTorque = 60.0f;
float targetRpm = 4200.0f;
float targetEngineRpm = 3800.0f;

// =====================================================
// Measured values
// =====================================================
float engineRpm = 0.0f;
float pumpRpm = 0.0f;
float secondaryRpm = 0.0f;
float torque = 0.0f;
float power = 0.0f;

// =====================================================
// Helpers
// =====================================================
void colorSet(ledColor color) {
  if (color == RED) {
    led.setPixelColor(0, led.Color(255, 0, 0));
    currentColor = RED;
  } else if (color == YELLOW) {
    led.setPixelColor(0, led.Color(255, 255, 0));
    currentColor = YELLOW;
  } else if (color == GREEN) {
    led.setPixelColor(0, led.Color(0, 255, 0));
    currentColor = GREEN;
  } else if (color == BLUE) {
    led.setPixelColor(0, led.Color(0, 0, 255));
    currentColor = BLUE;
  } else {
    led.setPixelColor(0, led.Color(0, 0, 0));
    currentColor = OFF;
  }
  led.show();
}

float percentFromPwm(float value, int minVal, int maxVal) {
  if (maxVal <= minVal) return 0.0f;
  float pct = ((value - minVal) * 100.0f) / (float)(maxVal - minVal);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

uint32_t engineDuty(uint32_t pulse_us) {
  uint32_t maxDuty = (1 << pwmResolution) - 1;
  uint32_t period_us = 1000000UL / engineThrottleFrequency;
  return (pulse_us * maxDuty) / period_us;
}

// =====================================================
// RPM ISRs
// =====================================================
void IRAM_ATTR engineMagRead() {
  prevEnginePulse = enginePulse;
  enginePulse = micros();
}

void IRAM_ATTR pumpMagRead() {
  prevPumpPulse = pumpPulse;
  pumpPulse = micros();
}

// =====================================================
// RPM calculation
// =====================================================
void getEngineRpm() {
  unsigned long nowPulse = enginePulse;
  unsigned long lastPulse = prevEnginePulse;

  if (micros() - nowPulse > 1000000UL) {
    engineRpm = 0.0f;
  } else if (nowPulse > lastPulse) {
    double magFreq = 60000000.0 / (double)(nowPulse - lastPulse);
    engineRpm = magFreq / engineMagnets;
  }
}

void getPumpRpm() {
  unsigned long nowPulse = pumpPulse;
  unsigned long lastPulse = prevPumpPulse;

  if (micros() - nowPulse > 1000000UL) {
    pumpRpm = 0.0f;
  } else if (nowPulse > lastPulse) {
    double magFreq = 60000000.0 / (double)(nowPulse - lastPulse);
    pumpRpm = magFreq / pumpMagnets;
  }
}

// =====================================================
// Torque read
// =====================================================
void getTorque() {
  if (scaleConnected && scale.is_ready()) {
    torque = scale.get_units(10);
  }
}

// =====================================================
// PID helpers
// =====================================================
void absPIDControl(float error, float& integral, float& previousError,
                   float P, float I, float D, float& valveValue,
                   int minVal, int maxVal, unsigned long& previousTime) {
  unsigned long now = micros();
  float dt = (now - previousTime) / 1000000.0f;
  if (dt <= 0.0f) dt = 0.000001f;
  previousTime = now;

  float proportional = P * error;

  integral += error * I * dt;
  integral = constrain(integral, (float)minVal, (float)maxVal);

  float derivative = D * ((error - previousError) / dt);
  float output = proportional + integral + derivative;

  valveValue = constrain(output, (float)minVal, (float)maxVal);
  previousError = error;
}

void deltaPIDControl(float error, float& previousError, float& previousPreviousError,
                     float P, float I, float D, float& valveValue,
                     int minVal, int maxVal, unsigned long& previousTime) {
  unsigned long now = micros();
  float dt = (now - previousTime) / 1000000.0f;
  if (dt <= 0.0f) dt = 0.000001f;
  previousTime = now;

  float deltaP = P * (error - previousError);
  float deltaI = I * error * dt;
  float deltaD = D * ((error - 2.0f * previousError + previousPreviousError) / dt);

  float deltaOutput = deltaP + deltaI + deltaD;
  valveValue += deltaOutput;
  valveValue = constrain(valveValue, (float)minVal, (float)maxVal);

  previousPreviousError = previousError;
  previousError = error;
}

// =====================================================
// Control loops
// =====================================================
void torqueControl() {
  getTorque();
  float error = targetTorque - torque;

  if (pressurePreviousTime == 0) {
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

  targetPumpRpm = (targetRpm * primarySprocket) / (float)secondarySprocket;
  targetPumpRpm = constrain(targetPumpRpm, 0.0f, (float)maxPumpRpm);

  float error = targetPumpRpm - pumpRpm;

  if (flowPreviousTime == 0) {
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

  if (mode == MANUAL) {
    // Manual throttle retained only if you later add a GUI throttle field.
    // Right now your GUI protocol does not send manual throttle, so this stays at its last/default value.
    engineThrottleValue = map(manualEngineThrottle, 0, 100, engineThrottleMin, engineThrottleMax);
    ledcWrite(engineThrottleChannel, engineDuty((uint32_t)engineThrottleValue));
    return;
  }

  float error = targetEngineRpm - engineRpm;

  if (enginePreviousTime == 0) {
    enginePreviousTime = micros();
    engineIntegral = 0.0f;
    enginePreviousError = error;
    enginePreviousPreviousError = error;
  }

  deltaPIDControl(error, enginePreviousError, enginePreviousPreviousError,
                  engineP, engineI, engineD, engineThrottleValue,
                  engineThrottleMin, engineThrottleMax, enginePreviousTime);

  ledcWrite(engineThrottleChannel, engineDuty((uint32_t)engineThrottleValue));
}

void pumpControl() {
  if (mode == MANUAL) {
    digitalWrite(flowValveEnablePin, HIGH);
    digitalWrite(pressureValveEnablePin, HIGH);

    flowValveValue = map(manualFlow, 0, 100, minFlowValve, maxFlowValve);
    pressureValveValue = map(manualPressure, 0, 100, minPressureValve, maxPressureValve);
  }
  else if (mode == TORQUE && scaleConnected) {
    flowValveValue = 0.0f;
    digitalWrite(flowValveEnablePin, LOW);
    digitalWrite(pressureValveEnablePin, HIGH);
    torqueControl();
  }
  else if (mode == RPM || mode == CVT) {
    pressureValveValue = 0.0f;
    digitalWrite(pressureValveEnablePin, LOW);
    digitalWrite(flowValveEnablePin, HIGH);
    pumpRpmControl();
  }

  ledcWrite(flowValveChannel, (uint32_t)flowValveValue);
  ledcWrite(pressureValveChannel, (uint32_t)pressureValveValue);
}

// =====================================================
// Serial status
// =====================================================
void printStatus() {
  Serial.print("Mode: ");
  Serial.print(mode == MANUAL ? "MANUAL" :
               mode == TORQUE ? "TORQUE" :
               mode == RPM    ? "RPM" :
               mode == CVT    ? "CVT" : "UNKNOWN");

  Serial.print(" || Engine RPM: ");
  Serial.print(engineRpm);
  Serial.print(" - ");
  Serial.print(targetEngineRpm);

  if (mode == TORQUE) {
    Serial.print(" || Torque: ");
    Serial.print(torque);
    Serial.print(" - ");
    Serial.print(targetTorque);
    Serial.print(" || Pressure Valve: ");
    Serial.print(percentFromPwm(pressureValveValue, minPressureValve, maxPressureValve));
  }

  if (mode == RPM || mode == CVT) {
    Serial.print(" || Pump RPM: ");
    Serial.print(pumpRpm);
    Serial.print(" - ");
    Serial.print(targetPumpRpm);
    Serial.print(" || Flow Valve: ");
    Serial.print(percentFromPwm(flowValveValue, minFlowValve, maxFlowValve));
  }

  Serial.print("% || Engine Throttle: ");
  Serial.print(percentFromPwm(engineThrottleValue, engineThrottleMin, engineThrottleMax));
  Serial.println("%");
}

// =====================================================
// BLE command handling
// =====================================================
void applyCommandPacket(const CommandPacket& pkt) {
  if (!dynoCommandPacketValid(pkt)) return;

  emergency = pkt.emergency != 0;

  switch (pkt.mode) {
    case MODE_MANUAL: mode = MANUAL; break;
    case MODE_TORQUE: mode = TORQUE; break;
    case MODE_RPM:    mode = RPM;    break;
    case MODE_CVT:    mode = CVT;    break;
    default:          mode = MANUAL; break;
  }

  manualFlow = constrain((int)pkt.manualFlowPct, 0, 100);
  manualPressure = constrain((int)pkt.manualPressurePct, 0, 100);

  // FIX: this was wrongly being written into manualEngineThrottle before
  targetTorque = pkt.targetTorque;
  targetRpm = pkt.targetRpm;
  targetEngineRpm = pkt.targetEngineRpm;

  Serial.print("[BLE][CTRL] CMD | mode=");
  Serial.print((int)pkt.mode);
  Serial.print(" | estop=");
  Serial.print((int)pkt.emergency);
  Serial.print(" | flow=");
  Serial.print(manualFlow);
  Serial.print(" | pressure=");
  Serial.print(manualPressure);
  Serial.print(" | targetTorque=");
  Serial.print(targetTorque);
  Serial.print(" | targetRpm=");
  Serial.print(targetRpm);
  Serial.print(" | targetEngineRpm=");
  Serial.println(targetEngineRpm);
}

class DynoServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    g_bleClientConnected = true;
    Serial.println("[BLE][CTRL] Client connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    g_bleClientConnected = false;
    Serial.println("[BLE][CTRL] Client disconnected");

    BLEAdvertising* adv = pServer->getAdvertising();
    adv->addServiceUUID(DYNO_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->start();

    Serial.println("[BLE][CTRL] Advertising restarted");
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    size_t len = characteristic->getLength();
    uint8_t* data = characteristic->getData();

    Serial.print("[BLE][CTRL] Write received, len = ");
    Serial.println((int)len);

    if (len != sizeof(CommandPacket) || data == nullptr) {
      Serial.print("[BLE][CTRL] Bad packet size. Expected ");
      Serial.print(sizeof(CommandPacket));
      Serial.print(", got ");
      Serial.println((int)len);
      return;
    }

    CommandPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (!dynoCommandPacketValid(pkt)) {
      Serial.println("[BLE][CTRL] Invalid command packet magic/version");
      return;
    }

    applyCommandPacket(pkt);
  }
};

// =====================================================
// Telemetry
// =====================================================
void sendTelemetry() {
  getEngineRpm();
  getPumpRpm();
  if (scaleConnected) getTorque();

  TelemetryPacket pkt{};
  pkt.magic = DYNO_PACKET_MAGIC;
  pkt.version = DYNO_PACKET_VERSION;
  pkt.emergency = emergency ? 1 : 0;
  pkt.scaleConnected = scaleConnected ? 1 : 0;
  pkt.mode = (uint8_t)(
      mode == MANUAL ? MODE_MANUAL :
      mode == TORQUE ? MODE_TORQUE :
      mode == RPM    ? MODE_RPM :
                       MODE_CVT
  );

  pkt.engineRpm = engineRpm;
  pkt.targetEngineRpm = targetEngineRpm;
  pkt.pumpRpm = pumpRpm;
  pkt.targetPumpRpm = targetPumpRpm;
  pkt.torque = torque;
  pkt.targetTorque = targetTorque;
  pkt.flowValvePercent = percentFromPwm(flowValveValue, minFlowValve, maxFlowValve);
  pkt.pressureValvePercent = percentFromPwm(pressureValveValue, minPressureValve, maxPressureValve);
  pkt.throttlePercent = percentFromPwm(engineThrottleValue, engineThrottleMin, engineThrottleMax);
  pkt.powerKw = (pumpRpm * torque) / 7047.0f;

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
    Serial.print((int)pkt.mode);
    Serial.print(" | engine=");
    Serial.print(engineRpm, 1);
    Serial.print(" | pump=");
    Serial.print(pumpRpm, 1);
    Serial.print(" | tq=");
    Serial.print(torque, 2);
    Serial.print(" | emergency=");
    Serial.println(emergency ? "1" : "0");
  }
}

// =====================================================
// Non-blocking emergency handler
// =====================================================
void emergencyStop() {
  static unsigned long emergencyBlinkMs = 0;

  bool estopActive = (digitalRead(emergencyPin) == LOW) || emergency;
  if (!estopActive) return;

  emergency = true;

  flowValveValue = 0.0f;
  pressureValveValue = 0.0f;
  engineThrottleValue = engineThrottleMin;

  ledcWrite(flowValveChannel, 0);
  ledcWrite(pressureValveChannel, 0);
  ledcWrite(engineThrottleChannel, engineDuty((uint32_t)engineThrottleValue));

  flowIntegral = 0.0f;
  pressureIntegral = 0.0f;
  engineIntegral = 0.0f;

  flowPreviousError = 0.0f;
  pressurePreviousError = 0.0f;
  enginePreviousError = 0.0f;

  flowPreviousPreviousError = 0.0f;
  pressurePreviousPreviousError = 0.0f;
  enginePreviousPreviousError = 0.0f;

  flowPreviousTime = 0;
  pressurePreviousTime = 0;
  enginePreviousTime = 0;

  if (millis() - emergencyBlinkMs >= 200) {
    emergencyBlinkMs = millis();

    if (currentColor != RED) colorSet(RED);
    else colorSet(OFF);

    Serial.println("EMERGENCY STOP ENGAGED!");
    sendTelemetry();  // keep GUI alive even during e-stop
  }

  // reset button clears latched emergency
  if (digitalRead(emergencyPin) == HIGH && digitalRead(resetPin) == LOW) {
    emergency = false;
    colorSet(scaleConnected ? GREEN : YELLOW);
    Serial.println("EMERGENCY CLEARED");
  }
}

// =====================================================
// BLE setup
// =====================================================
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

// =====================================================
// Setup
// =====================================================
void setup() {
  led.begin();
  led.setBrightness(100);
  colorSet(RED);

  const int halt = 500;

  Serial.begin(115200);
  delay(200);

  setupBle();

  Serial.println("Dynamometer initializing...");
  delay(halt * 2);

  colorSet(YELLOW);

  Serial.println("Assigning pins...");

  int pins[] = {
    0,1,2,3,4,5,6,7,8,9,10,
    11,12,13,14,15,16,19,20,21,
    35,36,38,39,40,41,42,45,47,48
  };

  int numPins = sizeof(pins) / sizeof(pins[0]);

  for (int i = 0; i < numPins; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }

  pinMode(emergencyPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(flowValvePin, OUTPUT);
  pinMode(pressureValvePin, OUTPUT);
  pinMode(flowValveEnablePin, OUTPUT);
  pinMode(pressureValveEnablePin, OUTPUT);
  pinMode(enginePulseSensor, INPUT_PULLUP);
  pinMode(pumpPulseSensor, INPUT_PULLUP);
  pinMode(engineThrottlePin, OUTPUT);

  delay(halt);

  Serial.println("Setting up PWM channels...");
  ledcAttachChannel(flowValvePin, flowValveFrequency, pwmResolution, flowValveChannel);
  ledcAttachChannel(pressureValvePin, pressureValveFrequency, pwmResolution, pressureValveChannel);
  ledcAttachChannel(engineThrottlePin, engineThrottleFrequency, pwmResolution, engineThrottleChannel);

  delay(halt);

  Serial.println("Attaching interrupts...");
  attachInterrupt(digitalPinToInterrupt(enginePulseSensor), engineMagRead, RISING);
  attachInterrupt(digitalPinToInterrupt(pumpPulseSensor), pumpMagRead, RISING);

  delay(halt);

  Serial.print("Initializing HX711...");
  scale.begin(DOUT, CLK, true, false);

  getEngineRpm();
  getPumpRpm();

  while (engineRpm != 0.0f && pumpRpm != 0.0f) {
    colorSet(RED);
    Serial.println("Waiting for engine and pump to stop...");
    getEngineRpm();
    getPumpRpm();
    delay(100);
  }

  colorSet(YELLOW);

  if (scale.wait_ready_timeout(1000)) {
    Serial.println("HX711 detected.");
    scaleConnected = true;

    digitalWrite(flowValveEnablePin, HIGH);
    ledcWrite(flowValveChannel, maxFlowValve);
    delay(2500);

    scale.set_scale(scaleFactor);
    scale.tare(50);

    ledcWrite(flowValveChannel, 0);
    digitalWrite(flowValveEnablePin, LOW);
  } else {
    Serial.println("HX711 NOT detected. Torque control unavailable.");
    scaleConnected = false;
  }

  //dynoInitCommandPacket(*(CommandPacket*)nullptr); // no-op placeholder removed below
  // defaults already set by globals, but make them explicit:
  targetTorque = 60.0f;
  targetRpm = 4200.0f;
  targetEngineRpm = 3800.0f;

  Serial.println("Dynamometer initialized!");
  colorSet(scaleConnected ? GREEN : YELLOW);
  delay(halt);
}

// =====================================================
// Loop
// =====================================================
void loop() {
  emergencyStop();

  if (!emergency) {
    pumpControl();
    engineRpmControl();
  }

  if (millis() - blePreviousMillis >= BLE_TELEMETRY_PERIOD_MS) {
    blePreviousMillis = millis();
    sendTelemetry();
  }

  if (!emergency && millis() - ledPreviousMillis >= 500) {
    if (!scaleConnected) {
      if (currentColor != YELLOW) colorSet(YELLOW);
      else colorSet(GREEN);
    } else {
      colorSet(GREEN);
    }

    ledPreviousMillis = millis();
    printStatus();
  }
}
