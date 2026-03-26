#include <Arduino.h>
#include <HX711.h>
#include <ESP32Servo.h>
#include <Adafruit_Neopixel.h>
#include <HardwareSerial.h>
#include <dyno_protocol.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// LED Properties
#define LED_PIN 48
#define LED_COUNT 1

Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
HX711 scale;

// ============================
// BLE globals
// ============================
BLEServer* g_server = nullptr;
BLECharacteristic* g_cmdChar = nullptr;
BLECharacteristic* g_telChar = nullptr;
volatile bool g_bleClientConnected = false;
unsigned long g_lastBleDebugMs = 0;
unsigned long blePreviousMillis = 0;

const unsigned long BLE_TELEMETRY_PERIOD_MS = 50;

enum ledColor { OFF, RED, YELLOW, GREEN, BLUE }; // LED color states
ledColor currentColor = OFF;

enum Mode { MANUAL, TORQUE, RPM, CVT }; // Control modes

// Runtime
#define emergencyPin 1 // Pin to trigger emergency state (normally open switch to ground)
#define resetPin 2 // Pin to reset from emergency state (normally open switch to ground)
bool emergency = true; // Start in emergency state until reset is pressed
unsigned long ledPreviousMillis = 0;
unsigned long UIPreviousMillis = 0;
unsigned long PIDPreviousMillis = 0;

// HX711
#define DOUT 4  // Data pin
#define CLK 5   // Clock pin
bool scaleConnected = false;
float scaleFactor = 2280.f; // Calibration factor for the load cell (adjust as needed)

// Pump Pins
#define pumpPulseSensor 9 // Pump RPM sensor (magnetic pickup)
#define flowValvePin 10      // Flow solenoid PWM
#define flowValveEnablePin 11 // Flow solenoid enable pin (if needed, set HIGH to enable)
#define pressureValvePin 12  // Pressure solenoid PWM
#define pressureValveEnablePin 13 // Pressure solenoid enable pin (if needed, set HIGH to enable)


#define flowValveChannel 1      // PWM channel for flow valve
#define pressureValveChannel 2  // PWM channel for pressure valve
#define engineThrottleChannel 3    // PWM channel for engine throttle

// Engine Pins
#define enginePulseSensor 6
#define engineThrottlePin 7

// Solenoid PWM values
float flowValveValue = 0;      // Current flow valve setting (0-4095)
int flowValveFrequency = 250; // PWM frequency for flow valve
float pressureValveValue = 0; // Current pressure valve setting (0-4095)
int pressureValveFrequency = 250; // PWM frequency for pressure valve

int pwmResolution = 12; // PWM resolution (12 bits for 0-4095 range)

// Tunable parameters
int mode = MANUAL; // Start in manual mode
int g_lastCommandedMode = MANUAL; // Track last commanded mode for telemetry reporting
int manualPressure = 0; // Manual pressure valve setting (0-100%)
int manualFlow = 0; // Manual flow valve setting (0-100%)
int manualEngineThrottle = 0; // Manual engine throttle setting (0-100%)

int minPressureValve = 0; // Minimum pressure valve setting (0% duty cycle)
int maxPressureValve = 0.98 * (pow(2, pwmResolution) - 1); // Maximum pressure valve setting (98% duty cycle)

int minFlowValve = 0; // Minimum flow valve setting (0% duty cycle)
int maxFlowValve = 0.98 * (pow(2, pwmResolution) - 1); // Maximum flow valve setting (98% duty cycle)

int primarySprocket = 28; // Teeth on primary sprocket
int secondarySprocket = 40; // Teeth on secondary sprocket

float pressureP = 1.0; // Proportional gain for pressure control
float pressureI = 0.2; // Integral gain for pressure control
float pressureD = 0.05; // Derivative gain for pressure control
float pressureIntegral = 0; // Integral term for pressure control
float pressurePreviousError = 0; // Previous error for pressure control
float pressurePreviousPreviousError = 0; // Previous previous error for delta PID
unsigned long pressurePreviousTime = 0; // Previous time for pressure control PID

float flowP = 0.25;     // Proportional gain for flow control
float flowI = 0.0625;     // Integral gain for flow control
float flowD = 0.025;    // Derivative gain for flow control
float flowIntegral = 0; // Integral term for flow control
float flowPreviousError = 0; // Previous error for flow control
float flowPreviousPreviousError = 0; // Previous previous error for delta PID
unsigned long flowPreviousTime = 0; // Previous time for flow control PID

// Engine RPM variables:
int engineThrottleMin = 500;       // minimum throttle command %
int engineThrottleMax = 1200;     // maximum throttle command %
float engineThrottleValue = engineThrottleMin;     // PID output, 0-100%

int engineThrottleFrequency = 150; // PWM frequency for engine throttle (servo control)

int servoMinAngle = 20;              // actual closed throttle position
int servoMaxAngle = 110;             // actual full throttle position

unsigned long enginePulse;
unsigned long prevEnginePulse;
int engineMagnets = 1;

float engineP = 0.1;     // Proportional gain for engine RPM control
float engineI = 0.02;     // Integral gain for engine RPM control
float engineD = 0.005;    // Derivative gain for engine RPM control
float engineIntegral = 0; // Integral term for engine RPM control
float enginePreviousError = 0; // Previous error for engine RPM control
float enginePreviousPreviousError = 0; // Previous previous error for delta PID
unsigned long enginePreviousTime = 0; // Previous time for engine RPM control PID

// Pump RPM variables:
float targetPumpRpm;   // Target pump RPM
const int maxPumpRpm = 3000; // Maximum pump RPM for safety
unsigned long pumpPulse;
unsigned long prevPumpPulse;
int pumpMagnets = 3;
int rpmLimiterDeadband = 300; // RPM where the RPM limiter starts to kick in (for torque control mode)

// Target values
float targetTorque = 60.0; // Target torque in Nm
float targetRpm = 4200.0;   // Target pump for AUX connected to pump
float targetEngineRpm = 3800.0; // Target engine RPM

// Active values:
float engineRpm = 0;
float pumpRpm = 0;
float secondaryRpm = 0;
float torque = 0;
float power = 0;

void colorSet(ledColor color){
  if(color == RED){
    led.setPixelColor(0, led.Color(255,0,0)); // RED
    currentColor = RED;
  } else if(color == YELLOW){
    led.setPixelColor(0, led.Color(255,255,0)); // YELLOW
    currentColor = YELLOW;
  } else if(color == GREEN){
    led.setPixelColor(0, led.Color(0,255,0)); // GREEN
    currentColor = GREEN;
  } else if(color == BLUE){
    led.setPixelColor(0, led.Color(0,0,255)); // BLUE
    currentColor = BLUE;
  } else if(color == OFF){
    led.setPixelColor(0, led.Color(0,0,0)); // OFF
    currentColor = OFF;
  }
  led.show();
}

// Function to calculate RPM from pulse time
void engineMagRead() {
  prevEnginePulse = enginePulse;
  enginePulse = micros();
}

// Function to calculate RPM from magnetic interval
void getEngineRpm() {
  // Convert difference into frequency (seconds domain)
  if (micros() - enginePulse > 1000000){ // If more than 1 second has passed, assume engine is stopped
    engineRpm = 0;
  } else if(enginePulse > prevEnginePulse){
    double magFreq = 60000000 / (enginePulse - prevEnginePulse);
    engineRpm = magFreq / engineMagnets;
  }
}

// Function to calculate RPM from pulse time
void pumpMagRead() {
  prevPumpPulse = pumpPulse;
  pumpPulse = micros();
}

// Function to calculate RPM from magnetic interval
void getPumpRpm() {
  // Convert difference into frequency (seconds domain)
  if (micros() - pumpPulse > 1000000){ // If more than 1 second has passed, assume pump is stopped
    pumpRpm = 0;
  } else if(pumpPulse > prevPumpPulse){
    double magFreq = 60000000 / (pumpPulse - prevPumpPulse);
    pumpRpm = magFreq / pumpMagnets;
  }
}

// Function to read torque from HX711
void getTorque(){
  if (scale.is_ready()) {
      torque = scale.get_units(10);
    }
}

// Function to control solenoid valves based on mode and RPM
void pumpControl() {
 
  // Set flow valve (0-4095 PWM)
  if (mode == MANUAL) { // Manual mode
    digitalWrite(flowValveEnablePin, HIGH); // Enable flow valve
    digitalWrite(pressureValveEnablePin, HIGH); // Enable pressure valve
    flowValveValue = map(manualFlow, 0, 100, 0, maxFlowValve);
    pressureValveValue = map(manualPressure, 0, 100, 0, maxPressureValve);
  }
  
  else if (mode == TORQUE && scaleConnected) { // Torque control mode
    flowValveValue = 0; // Start with flow valve closed
    digitalWrite(flowValveEnablePin, LOW); // Disable flow valve
    digitalWrite(pressureValveEnablePin, HIGH); // Enable pressure valve
    torqueControl();
  }
  
  else if (mode == RPM) { // RPM control mode
    pressureValveValue = 0; // Start with pressure valve closed
    digitalWrite(pressureValveEnablePin, LOW); // Disable pressure valve
    digitalWrite(flowValveEnablePin, HIGH); // Enable flow valve
    pumpRpmControl();
  }

  // Write PWM values to valves
  ledcWrite(flowValveChannel, flowValveValue);
  ledcWrite(pressureValveChannel, pressureValveValue);
}

// PID control function for torque control
void torqueControl() {
  getTorque(); // Update torque reading
  float error = targetTorque - torque;

  if(micros() > (pressurePreviousTime + 100000)) {
    pressurePreviousTime = micros(); // Initialize previous time on first run
    pressureIntegral = 0; // Reset integral term
    pressurePreviousError = error; // Initialize previous error
    pressurePreviousPreviousError = error; // Initialize previous previous error
  }

  /*absPIDControl(error, pressureIntegral, pressurePreviousError,
                  pressureP, pressureI, pressureD, pressureValveValue,
                  minPressureValve, maxPressureValve, pressurePreviousTime);*/
  
  deltaPIDControl(error, pressurePreviousError, pressurePreviousPreviousError,
                  pressureP, pressureI, pressureD, pressureValveValue,
                  minPressureValve, maxPressureValve, pressurePreviousTime);

  // RPM limiter (only near the top)
  getPumpRpm(); // Update pump RPM reading
  float rpmLimiter = 1.0;

  if (pumpRpm > maxPumpRpm - rpmLimiterDeadband) {  // start limiting near max
    rpmLimiter = 1 - ((pumpRpm - (maxPumpRpm - rpmLimiterDeadband)) / (rpmLimiterDeadband*2)); // ramp to zero at max
    rpmLimiter = constrain(rpmLimiter, 0, 1);
  }

  pressureValveValue *= rpmLimiter;
}

// PID control function for RPM control
void pumpRpmControl() {
  getPumpRpm(); // Update pump RPM reading
  targetPumpRpm = (targetRpm * primarySprocket) / secondarySprocket; // Calculate target pump RPM based on sprocket ratio
  targetPumpRpm = constrain(targetPumpRpm, 0, maxPumpRpm); // Ensure target pump RPM does not exceed target engine RPM
  float error = targetPumpRpm - pumpRpm;

  if(micros() > (flowPreviousTime + 100000)){
    flowPreviousTime = micros(); // Initialize previous time on first run
    flowIntegral = 0; // Reset integral term
    flowPreviousError = error; // Initialize previous error
    flowPreviousPreviousError = error; // Initialize previous previous error
  }

  /*absPIDControl(error, flowIntegral, flowPreviousError,
                  flowP, flowI, flowD, flowValveValue,
                  minFlowValve, maxFlowValve, flowPreviousTime);*/
  
  deltaPIDControl(error, flowPreviousError, flowPreviousPreviousError,
                  flowP, flowI, flowD, flowValveValue,
                  minFlowValve, maxFlowValve, flowPreviousTime);
}

uint32_t engineDuty(uint32_t pulse) {
  uint32_t maxDuty = (1 << pwmResolution) - 1;
  uint32_t period_us = 1000000UL / engineThrottleFrequency;
  
  return (pulse * maxDuty) / period_us;
}

// PID control function for engine RPM control
void engineRpmControl() {
  getEngineRpm(); // Update engine RPM reading

  if(mode == MANUAL) {
    engineThrottleValue = map(manualEngineThrottle, 0, 100, engineThrottleMin, engineThrottleMax);
    ledcWrite(engineThrottleChannel, engineDuty(engineThrottleValue)); // Map throttle value to 0-100% for servo write
    return;
  }

  else{
    float error = targetEngineRpm - engineRpm;

    if(micros() > (enginePreviousTime + 100000)) {
      enginePreviousTime = micros(); // Initialize previous time on first run
      engineIntegral = 0; // Reset integral term
      enginePreviousError = error; // Initialize previous error
      enginePreviousPreviousError = error; // Initialize previous previous error
    }

    /*absPIDControl(error, engineIntegral, enginePreviousError,
                    engineP, engineI, engineD, engineThrottleValue,
                    engineThrottleMin, engineThrottleMax, enginePreviousTime);*/
    
    deltaPIDControl(error, enginePreviousError, enginePreviousPreviousError,
                    engineP, engineI, engineD, engineThrottleValue,
                    engineThrottleMin, engineThrottleMax, enginePreviousTime);

    ledcWrite(engineThrottleChannel, engineDuty(engineThrottleValue)); // Map throttle value to 0-100% for servo write
  }
}

void absPIDControl(float error, float &integral, float &previousError, 
                    float P, float I, float D, float &valveValue,
                    int min, int max, unsigned long &previousTime) {
  unsigned long now = micros();
  float dt = (now - previousTime) / 1000000.0;  // seconds
  if (dt <= 0) dt = 0.000001;                   // safety
  previousTime = now;

  float proportional = P * error;

  integral += error * I * dt;
  integral = constrain(integral, min, max);   // anti-windup

  float derivative = D * ((error - previousError) / dt);

  float output = proportional + integral + derivative;

  valveValue = constrain(output, min, max);

  previousError = error;
}

void deltaPIDControl(float error, float &previousError, float &previousPreviousError,
                    float P, float I, float D, float &valveValue,
                    int min, int max, unsigned long &previousTime) {
  unsigned long now = micros();
  float dt = (now - previousTime) / 1000000.0;   // seconds
  if (dt <= 0) dt = 0.000001;                    // safety
  previousTime = now;

  // Incremental PID terms
  float deltaP = P * (error - previousError);
  float deltaI = I * error * dt;
  float deltaD = D * ((error - 2 * previousError + previousPreviousError) / dt);

  float deltaOutput = deltaP + deltaI + deltaD;

  // Add change to existing output
  valveValue += deltaOutput;

  valveValue = constrain(valveValue, min, max);

  // Shift stored errors
  previousPreviousError = previousError;
  previousError = error;
}

void printStatus() {
  Serial.print("Mode: ");
  Serial.print(mode == MANUAL ? "MANUAL" : mode == TORQUE ? "TORQUE" : mode == RPM ? "RPM" : mode == CVT ? "CVT" : "UNKNOWN");
  Serial.print(" || Engine RPM: ");
  Serial.print(engineRpm);
  Serial.print(" - ");
  Serial.print(targetEngineRpm);
  
  if(mode == TORQUE) {
    Serial.print(" || Torque: ");
    Serial.print(torque);
    Serial.print(" - ");
    Serial.print(targetTorque);
    Serial.print(" || Pressure Valve: ");
    Serial.print(map(pressureValveValue, minPressureValve, maxPressureValve, 0, 100));
  }

  if(mode == RPM || mode == CVT) {
    Serial.print(" || Pump RPM: ");
    Serial.print(pumpRpm);
    Serial.print(" - ");
    Serial.print(targetPumpRpm);
    Serial.print(" || Flow Valve: ");
    Serial.print(map(flowValveValue, minFlowValve, maxFlowValve, 0, 100));
  }


  Serial.print("% || Engine Throttle: ");
  Serial.print(map(engineThrottleValue, engineThrottleMin, engineThrottleMax, 0, 100));
  Serial.println("%");
}

void applyCommandPacket(const CommandPacket& pkt) {
  if (!dynoCommandPacketValid(pkt)) return;

  emergency = pkt.emergency != 0;
  switch(pkt.mode) {
    case 0:
      mode = MANUAL;
      break;
    case 1:
      mode = TORQUE;
      break;
    case 2:
      mode = RPM;
      break;
    case 3:
      mode = CVT;
      break;
    default:
      mode = MANUAL;
  }
  manualFlow = constrain((int)pkt.manualFlowPct, 0, 100);
  manualEngineThrottle = constrain((int)pkt.manualPressurePct, 0, 100);
  //manualEngineThrottle = constrain((int)pkt.targetTorque, 0, 100);
  targetRpm = pkt.targetRpm;
  targetEngineRpm = pkt.targetEngineRpm;
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
    String value = String(characteristic->getValue().c_str());
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

    applyCommandPacket(pkt);
  }
};

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
  pkt.flowValvePercent = map(flowValveValue, minFlowValve, maxFlowValve, 0, 100);
  pkt.pressureValvePercent = map(pressureValveValue, minPressureValve, maxPressureValve, 0, 100);
  pkt.throttlePercent = map(engineThrottleValue, engineThrottleMin, engineThrottleMax, 0, 100);
  pkt.powerKw = ((pumpRpm * torque) / 7047.0); // Convert RPM and Torque to Power in kW

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
    Serial.print(mode == MANUAL ? 0 : mode == TORQUE ? 1 : mode == RPM ? 2 : mode == CVT ? 3 : -1);
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

void emergencyStop() {
  while(digitalRead(emergencyPin) == LOW || emergency) {
    emergency = true; // Set emergency flag to stay in loop until reset

    Serial.println("EMERGENCY STOP ENGAGED!");

    flowValveValue = 0;
    pressureValveValue = 0;
    engineThrottleValue = engineThrottleMin;

    ledcWrite(flowValveChannel, flowValveValue);
    ledcWrite(pressureValveChannel, pressureValveValue);
    ledcWrite(engineThrottleChannel, engineDuty(engineThrottleValue));

    flowIntegral = 0;
    pressureIntegral = 0;
    engineIntegral = 0;
    flowPreviousError = 0;
    pressurePreviousError = 0;
    enginePreviousError = 0;
    flowPreviousPreviousError = 0;
    pressurePreviousPreviousError = 0;
    enginePreviousPreviousError = 0;
    
    if(currentColor != RED) {
      colorSet(RED); // RED for emergency
    } else {
      colorSet(OFF); // Blink RED for emergency
    }

    delay(500);

    if(digitalRead(emergencyPin) == HIGH && digitalRead(resetPin) == LOW) { // Check if emergency switch is still engaged
      emergency = false;
      colorSet(OFF); // Turn off LED when exiting emergency state
    }
  }
}

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

void setup() {
  led.begin();
  led.setBrightness(100);
  colorSet(RED); // RED to start
  
  int halt = 500; // Delay between setup steps for stability

  
  Serial.begin(115200);
  setupBle();

  Serial.println("Dynamometer initializing...");
  delay(halt*2);

  colorSet(YELLOW); // YELLOW during setup
  
  // Initialize pins
  Serial.println("Assigning pins...");
  int pins[] = {
    0,1,2,3,4,5,6,7,8,9,10,
    11,12,13,14,15,16,19,20,21,
    35,36,38,39,40,41,42,45,47,48
  };

  int numPins = sizeof(pins) / sizeof(pins[0]);

  for(int i = 0; i < numPins; i++) {
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

  // Set PWM frequency and resolution
  Serial.println("Setting up PWM channels...");
  ledcSetup(flowValveChannel, flowValveFrequency, pwmResolution);
  ledcSetup(pressureValveChannel, pressureValveFrequency, pwmResolution);
  ledcSetup(engineThrottleChannel, engineThrottleFrequency, pwmResolution); // 150 Hz for servo control
  ledcAttachPin(flowValvePin, flowValveChannel);
  ledcAttachPin(pressureValvePin, pressureValveChannel);
  ledcAttachPin(engineThrottlePin, engineThrottleChannel);

  delay(halt);

  // Attach interrupts for RPM sensors
  Serial.println("Attaching interrupts...");
  attachInterrupt(digitalPinToInterrupt(enginePulseSensor), engineMagRead, RISING);
  attachInterrupt(digitalPinToInterrupt(pumpPulseSensor), pumpMagRead, RISING);

  delay(halt);

  // Initialize HX711
  Serial.print("Initializing HX711...");
  scale.begin(DOUT, CLK, true, false); // fast read, no reset
  
  getEngineRpm(); // Update engine RPM reading before waiting for engine to stop
  getPumpRpm(); // Update pump RPM reading before waiting for pump to stop

  while(engineRpm != 0 && pumpRpm != 0) { // Wait for engine and pump to stop (RPM = 0) before proceeding with HX711 setup
    colorSet(RED); // RED while waiting for engine and pump to stop
    Serial.println("Waiting for engine and pump to stop...");
    getEngineRpm();
    getPumpRpm();
    delay(100);
  }

  colorSet(YELLOW); // YELLOW again
  
  if (scale.wait_ready_timeout(1000)) {
    Serial.println("HX711 detected.");
    scaleConnected = true;
    
    digitalWrite(flowValveEnablePin, HIGH); // Enable flow valve to relieve pressure for taring
    ledcWrite(flowValveChannel, maxFlowValve); // Relieve any pressure in the system for accurate taring
    delay(2500);

    scale.set_scale(scaleFactor);
    scale.tare(50); // Tare with 50 samples for better accuracy

    ledcWrite(flowValveChannel, flowValveValue); // Close flow valve after taring
    digitalWrite(flowValveEnablePin, LOW); // Disable flow valve after taring
  } else {
    Serial.println("HX711 NOT detected. Torque control unavailable.");
  }

  delay(halt);

  // Finalize setup
  Serial.println("Dynamometer initialized!");

  delay(halt);
}

void loop() {
  emergencyStop(); // Check for emergency stop condition every loop
  pumpControl();
  engineRpmControl();

  if (millis() - blePreviousMillis >= BLE_TELEMETRY_PERIOD_MS) {
    blePreviousMillis = millis();
    sendTelemetry();
  }

  if (millis() - ledPreviousMillis >= 500) {
    if(!scaleConnected) {
      if(currentColor != YELLOW) {
        colorSet(YELLOW); // YELLOW if no scale connected
      }else{
        colorSet(GREEN); // GREEN if no scale connected
      }
    } else {
      colorSet(GREEN); // GREEN for normal operation
    }
    ledPreviousMillis = millis();
    printStatus();
  }
}
