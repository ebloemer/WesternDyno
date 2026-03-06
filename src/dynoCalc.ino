#include <Arduino.h>
#include <HX711.h>
#include <ESP32Servo.h>
#include <Adafruit_Neopixel.h>

#define LED_PIN 48
#define LED_COUNT 1

Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
HX711 scale;
Servo engineThrottle; // Using Servo library for throttle control (0-180 degrees)

enum ledColor { OFF, RED, YELLOW, GREEN, BLUE }; // 0 = off, 1 = red, 2 = yellow, 3 = green, 4 = blue (no scale)
ledColor currentColor = OFF;

// Runtime
const int emergencyPin = 14; // Pin to trigger emergency state (normally open switch to ground)
const int resetPin = 13; // Pin to reset from emergency state (normally open switch to ground)
bool emergency = false;
unsigned long previousTime = 0;

// HX711
const int DOUT = 4;  // Data pin
const int CLK = 5;   // Clock pin
bool scaleConnected = false;
float scaleFactor = 2280.f; // Calibration factor for the load cell (adjust as needed)

// Pump Pins
const int pumpPulseSensor = 16;
const int flowValvePin = 17;      // Flow solenoid PWM
const int pressureValvePin = 18;  // Pressure solenoid PWM

const int flowValve = 1;      // PWM channel for flow valve
const int pressureValve = 2;  // PWM channel for pressure valve

// Engine Pins
const int enginePulseSensor = 10;
const int engineThrottlePin = 11;

// Solenoid PWM values
int flowValveValue = 0;      // Current flow valve setting (0-4095)
int flowValveFrequency = 250; // PWM frequency for flow valve
int pressureValveValue = 0; // Current pressure valve setting (0-4095)
int pressureValveFrequency = 250; // PWM frequency for pressure valve

int pwmResolution = 12; // PWM resolution (12 bits for 0-4095 range)

// Tunable parameters
int mode = 2; // 0 = manual, 1 = torque control, 2 = fixed ratio control, 3 = variable ratio control
int manualPressure = 0; // Manual pressure valve setting (0-100%)
int manualFlow = 0; // Manual flow valve setting (0-100%)

int minPressureValve = 0; // Minimum pressure valve setting (0% duty cycle)
int maxPressureValve = 0.98 * (pow(2, pwmResolution) - 1); // Maximum pressure valve setting (98% duty cycle)

int minFlowValve = 0; // Minimum flow valve setting (0% duty cycle)
int maxFlowValve = 0.98 * (pow(2, pwmResolution) - 1); // Maximum flow valve setting (98% duty cycle)

int primarySprocket = 28; // Teeth on primary sprocket
int secondarySprocket = 40; // Teeth on secondary sprocket

float pressureP = 5; // Proportional gain for pressure control
float pressureI = 1; // Integral gain for pressure control
float pressureD = .5; // Derivative gain for pressure control
float pressureIntegral = 0; // Integral term for pressure control
float pressurePreviousError = 0; // Previous error for pressure control
unsigned long pressurePreviousTime = 0; // Previous time for pressure control PID

float flowP = 0.1;     // Proportional gain for flow control
float flowI = 0.02;     // Integral gain for flow control
float flowD = 0.04;    // Derivative gain for flow control
float flowIntegral = 0; // Integral term for flow control
float flowPreviousError = 0; // Previous error for flow control
unsigned long flowPreviousTime = 0; // Previous time for flow control PID

// Engine RPM variables:
int engineThrottleValue = 0; // Current engine throttle setting (Min-Max degrees)
int engineThrottleMin = 0; // Minimum engine throttle setting (0 degrees)
int engineThrottleMax = 180; // Maximum engine throttle setting (180 degrees)

unsigned long enginePulse;
unsigned long prevEnginePulse;
int engineMagnets = 1;

float engineP = 0.005;     // Proportional gain for engine RPM control
float engineI = 0.001;     // Integral gain for engine RPM control
float engineD = 0.0005;    // Derivative gain for engine RPM control
float engineIntegral = 0; // Integral term for engine RPM control
float enginePreviousError = 0; // Previous error for engine RPM control
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
  if (mode == 0) { // Manual mode
    flowValveValue = map(manualFlow, 0, 100, 0, maxFlowValve);
    pressureValveValue = map(manualPressure, 0, 100, 0, maxPressureValve);
  }
  
  else if (mode == 1 && scaleConnected) { // Torque control mode
    flowValveValue = 0; // Start with flow valve closed
    torqueControl();
  }
  
  else if (mode == 2) { // RPM control mode
    pressureValveValue = 0; // Start with pressure valve closed
    pumpRpmControl();
  }

  // Write PWM values to valves
  ledcWrite(flowValve, flowValveValue);
  ledcWrite(pressureValve, pressureValveValue);
}

// PID control function for torque control
void torqueControl() {
  getTorque(); // Update torque reading
  float error = targetTorque - torque;

  PIDControl(error, pressureIntegral, pressurePreviousError, pressureP, pressureI, pressureD, pressureValveValue, minPressureValve, maxPressureValve, pressurePreviousTime);

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

  PIDControl(error, flowIntegral, flowPreviousError, flowP, flowI, flowD, flowValveValue, minFlowValve, maxFlowValve, flowPreviousTime);
}

// PID control function for engine RPM control
void engineRpmControl() {
  getEngineRpm(); // Update engine RPM reading
  float error = targetEngineRpm - engineRpm;

  PIDControl(error, engineIntegral, enginePreviousError, engineP, engineI, engineD, engineThrottleValue, engineThrottleMin, engineThrottleMax, enginePreviousTime);
}

void PIDControl(float error, float &integral, float &previousError, float P, float I, float D, int &valveValue, int min, int max, unsigned long &previousTime) {
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

void printStatus() {
  Serial.print("Engine RPM: ");
  Serial.print(engineRpm);
  Serial.print(" - ");
  Serial.print(targetEngineRpm);
  Serial.print(" || Pump RPM: ");
  Serial.print(pumpRpm);
  Serial.print(" - ");
  Serial.print(targetPumpRpm);
  Serial.print(" || Torque: ");
  Serial.print(torque);
  Serial.print(" - ");
  Serial.print(targetTorque);
  Serial.print(" || Flow Valve: ");
  Serial.print(map(flowValveValue, 0, maxFlowValve, 0, 100));
  Serial.print("% || Pressure Valve: ");
  Serial.print(map(pressureValveValue, 0, maxPressureValve, 0, 100));
  Serial.print("% || Engine Throttle: ");
  Serial.print(map(engineThrottleValue, engineThrottleMin, engineThrottleMax, 0, 100));
  Serial.println("%");
}

void emergencyStop() {
  while(digitalRead(emergencyPin) == HIGH || emergency) {
    emergency = true; // Set emergency flag to stay in loop until reset

    Serial.println("EMERGENCY STOP ENGAGED!");

    flowValveValue = 0;
    pressureValveValue = 0;
    engineThrottleValue = engineThrottleMin;

    ledcWrite(flowValve, flowValveValue);
    ledcWrite(pressureValve, pressureValveValue);
    engineThrottle.write(engineThrottleValue);

    flowIntegral = 0;
    pressureIntegral = 0;
    engineIntegral = 0;
    flowPreviousError = 0;
    pressurePreviousError = 0;
    enginePreviousError = 0;
    
    if(currentColor != RED) {
      colorSet(RED); // RED for emergency
    } else {
      colorSet(OFF); // Blink RED for emergency
    }

    delay(500);

    if(digitalRead(emergencyPin) == LOW && digitalRead(resetPin) == HIGH) { // Check if emergency switch is still engaged
      emergency = false;
      colorSet(OFF); // Turn off LED when exiting emergency state
    }
  }
}

void setup() {
  led.begin();
  led.setBrightness(100);
  colorSet(RED); // RED to start
  
  int halt = 1500; // Delay between setup steps for stability

  Serial.begin(115200);
  Serial.println("Dynamometer initializing...");
  delay(halt*2);

  colorSet(YELLOW); // YELLOW during setup
  
  // Initialize pins
  Serial.println("Assigning pins...");
  int pins[] = {
    0,1,2,3,4,5,6,7,8,9,10,
    11,12,13,14,15,16,17,18,19,20,21,
    35,36,38,39,40,41,42,45,47,48
  };

  int numPins = sizeof(pins) / sizeof(pins[0]);

  for(int i = 0; i < numPins; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
  
  pinMode(emergencyPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLDOWN);
  pinMode(flowValve, OUTPUT);
  pinMode(pressureValve, OUTPUT);
  pinMode(enginePulseSensor, INPUT_PULLDOWN);
  pinMode(pumpPulseSensor, INPUT_PULLDOWN);
  pinMode(engineThrottlePin, OUTPUT);

  engineThrottle.attach(engineThrottlePin, engineThrottleMin, engineThrottleMax);

  delay(halt);

  // Set PWM frequency and resolution
  Serial.println("Setting up PWM channels...");
  ledcSetup(flowValve, flowValveFrequency, pwmResolution);
  ledcSetup(pressureValve, pressureValveFrequency, pwmResolution);
  ledcAttachPin(flowValvePin, flowValve);
  ledcAttachPin(pressureValvePin, pressureValve);

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
    
    ledcWrite(flowValve, maxFlowValve); // Relieve any pressure in the system for accurate taring
    delay(2500);

    scale.set_scale(scaleFactor);
    scale.tare(50); // Tare with 50 samples for better accuracy

    ledcWrite(flowValve, flowValveValue); // Close flow valve after taring
  } else {
    Serial.println("HX711 NOT detected. Torque control unavailable.");
  }

  delay(halt);

  // Finalize setup
  Serial.println("Dynamometer initialized!");
  
  colorSet(GREEN); // GREEN for ready

  delay(halt);
}

void loop() {
  pumpControl();
  engineRpmControl();
  
  //emergencyStop(); // Check for emergency stop condition every loop

  if (millis() - previousTime >= 500) {
    if(!scaleConnected) {
      if(currentColor != YELLOW) {
        colorSet(YELLOW); // YELLOW if no scale connected
      }else{
        colorSet(GREEN); // GREEN if no scale connected
      }
    } else {
      colorSet(GREEN); // GREEN for normal operation
    }
    previousTime = millis();
    printStatus();
  }
}