#include <Arduino.h>
#include <HX711.h>
#include <ESP32Servo.h>
#include <Adafruit_Neopixel.h>
#include <HardwareSerial.h>

// Setting up serial to UI
HardwareSerial Interface(1); // Use UART1 (pins 17 and 18 for S3)
#define UI_BAUD_RATE 115200
#define UI_RX_PIN 17
#define UI_TX_PIN 18

// LED Properties
#define LED_PIN 48
#define LED_COUNT 1

Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
HX711 scale;
Servo engineThrottle; // Using Servo library for throttle control (0-180 degrees)

enum ledColor { OFF, RED, YELLOW, GREEN, BLUE }; // LED color states
ledColor currentColor = OFF;

// Runtime
#define emergencyPin 14 // Pin to trigger emergency state (normally open switch to ground)
#define resetPin 13 // Pin to reset from emergency state (normally open switch to ground)
bool emergency = false;
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
#define pressureValvePin 11  // Pressure solenoid PWM

#define flowValveChannel 1      // PWM channel for flow valve
#define pressureValveChannel 2  // PWM channel for pressure valve

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
int mode = 2; // 0 = manual, 1 = torque control, 2 = fixed ratio control, 3 = variable ratio control
int manualPressure = 0; // Manual pressure valve setting (0-100%)
int manualFlow = 0; // Manual flow valve setting (0-100%)

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

float flowP = 0.1;     // Proportional gain for flow control
float flowI = 0.025;     // Integral gain for flow control
float flowD = 0.01;    // Derivative gain for flow control
float flowIntegral = 0; // Integral term for flow control
float flowPreviousError = 0; // Previous error for flow control
float flowPreviousPreviousError = 0; // Previous previous error for delta PID
unsigned long flowPreviousTime = 0; // Previous time for flow control PID

// Engine RPM variables:
int engineThrottleMin = 500;       // minimum throttle command %
int engineThrottleMax = 2500;     // maximum throttle command %
float engineThrottleValue = engineThrottleMin;     // PID output, 0-100%

int servoMinAngle = 20;              // actual closed throttle position
int servoMaxAngle = 110;             // actual full throttle position

unsigned long enginePulse;
unsigned long prevEnginePulse;
int engineMagnets = 1;

float engineP = 0.001;     // Proportional gain for engine RPM control
float engineI = 0.0002;     // Integral gain for engine RPM control
float engineD = 0.00005;    // Derivative gain for engine RPM control
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

// PID control function for engine RPM control
void engineRpmControl() {
  getEngineRpm(); // Update engine RPM reading
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

  engineThrottle.writeMicroseconds((int)engineThrottleValue); // Map throttle value to 0-100% for servo write
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
  if(mode == 0) Serial.print("MANUAL");
  else if(mode == 1) Serial.print("TRQ CTRL");
  else if(mode == 2) Serial.print("RPM CTRL");
  else if(mode == 3) Serial.print("CVT CTRL");
  else Serial.print("N/A");
  Serial.print(" || Engine RPM: ");
  Serial.print(engineRpm);
  Serial.print(" - ");
  Serial.print(targetEngineRpm);
  
  if(mode == 1) {
    Serial.print(" || Torque: ");
    Serial.print(torque);
    Serial.print(" - ");
    Serial.print(targetTorque);
    Serial.print("% || Pressure Valve: ");
    Serial.print(map(pressureValveValue, minPressureValve, maxPressureValve, 0, 100));
  }

  if(mode == 2 || mode == 3) {
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

void sendUIData() {
  // Send data in a simple CSV format for easy parsing on the UI side
  Interface.print(emergency ? "1" : "0"); // Emergency status
  Interface.print(",");
  Interface.print(engineRpm);
  Interface.print(",");
  Interface.print(targetEngineRpm);
  Interface.print(",");
  Interface.print(pumpRpm);
  Interface.print(",");
  Interface.print(targetPumpRpm);
  Interface.print(",");
  Interface.print(torque);
  Interface.print(",");
  Interface.print(targetTorque);
  Interface.print(",");
  Interface.print(map(flowValveValue, 0, maxFlowValve, 0, 100));
  Interface.print(",");
  Interface.print(map(pressureValveValue, 0, maxPressureValve, 0, 100));
  Interface.print(",");
  Interface.println(map(engineThrottleValue, engineThrottleMin, engineThrottleMax, 0, 100));
}

void recieveUICommands() {
  // Check if data is available on the serial port
  if (Interface.available() > 0) {
    String command = Interface.readStringUntil('\n'); // Read until newline
    // Parse command and update parameters accordingly
    // Expected format: "EMERGENCY_STATUS,MODE,MANUAL_FLOW,MANUAL_PRESSURE,TARGET_TORQUE,TARGET_RPM,TARGET_ENGINE_RPM"
    int index = 0;
    String tokens[7];
    while (command.length() > 0 && index < 7) {
      int commaIndex = command.indexOf(',');
      if (commaIndex == -1) {
        tokens[index++] = command; // Last token
        break;
      } else {
        tokens[index++] = command.substring(0, commaIndex);
        command = command.substring(commaIndex + 1);
      }
    }
    
    if (index >= 1) emergency = tokens[0].toInt() == 1; // Set emergency flag based on first token
    if (index >= 2) mode = tokens[1].toInt();
    if (index >= 3) manualFlow = tokens[2].toInt();
    if (index >= 4) manualPressure = tokens[3].toInt();
    if (index >= 5) targetTorque = tokens[4].toFloat();
    if (index >= 6) targetRpm = tokens[5].toFloat();
    if (index >= 7) targetEngineRpm = tokens[6].toFloat();
  }
}

void emergencyStop() {
  while(digitalRead(emergencyPin) == HIGH || emergency) {
    emergency = true; // Set emergency flag to stay in loop until reset

    Serial.println("EMERGENCY STOP ENGAGED!");

    flowValveValue = 0;
    pressureValveValue = 0;
    engineThrottleValue = engineThrottleMin;

    ledcWrite(flowValveChannel, flowValveValue);
    ledcWrite(pressureValveChannel, pressureValveValue);
    engineThrottle.write(engineThrottleValue);

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
  
  int halt = 500; // Delay between setup steps for stability

  Serial.begin(115200);
  Interface.begin(UI_BAUD_RATE, SERIAL_8N1, UI_RX_PIN, UI_TX_PIN); // RX, TX pins for UART1

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
  pinMode(resetPin, INPUT_PULLDOWN);
  pinMode(flowValveChannel, OUTPUT);
  pinMode(pressureValveChannel, OUTPUT);
  pinMode(enginePulseSensor, INPUT_PULLDOWN);
  pinMode(pumpPulseSensor, INPUT_PULLDOWN);
  pinMode(engineThrottlePin, OUTPUT);

  engineThrottle.attach(engineThrottlePin, engineThrottleMin, engineThrottleMax);

  delay(halt);

  // Set PWM frequency and resolution
  Serial.println("Setting up PWM channels...");
  ledcSetup(flowValveChannel, flowValveFrequency, pwmResolution);
  ledcSetup(pressureValveChannel, pressureValveFrequency, pwmResolution);
  ledcAttachPin(flowValvePin, flowValveChannel);
  ledcAttachPin(pressureValvePin, pressureValveChannel);

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
    
    ledcWrite(flowValveChannel, maxFlowValve); // Relieve any pressure in the system for accurate taring
    delay(2500);

    scale.set_scale(scaleFactor);
    scale.tare(50); // Tare with 50 samples for better accuracy

    ledcWrite(flowValveChannel, flowValveValue); // Close flow valve after taring
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
  //emergencyStop(); // Check for emergency stop condition every loop
  pumpControl();
  //engineRpmControl();
  recieveUICommands();

  if (millis() - UIPreviousMillis >= 16) { // Send data to UI every 100ms
    sendUIData();
    UIPreviousMillis = millis();
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