/*
  Smart Soldering Iron Firmware
  MCU: ATmega328P
  Display: SSD1306 128x32
  Thermocouple: ADC
  Voltage negotiation: CH224K with 3 configuration pins
  Accelerometer: LIS3DHTR
  Buttons: OK, Forward, Backward
  Features:
    - PID temperature control
    - Voltage selection
    - Sleep/wake with accelerometer
    - No tip warning
    - Menu navigation using 3 buttons
*/

// Include libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>

// ---------- USER CONFIGURABLE PINS ----------
#define PIN_BUTTON_OK     2  // OK button
#define PIN_BUTTON_FWD    3  // Forward
#define PIN_BUTTON_BWD    4  // Backward

#define PIN_CH224K_CONF1  5  // CH224K configuration pin 1
#define PIN_CH224K_CONF2  6  // CH224K configuration pin 2
#define PIN_CH224K_CONF3  7  // CH224K configuration pin 3

#define PIN_THERMOCOUPLE  A0 // ADC for thermocouple
#define PIN_TIP_POWER     8  // MOSFET / relay for tip power

#define PIN_LIS3DH_INT    9  // Accelerometer interrupt pin

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// ---------- USER ADJUSTABLE PARAMETERS ----------
float setTemperature = 350.0;      // Default temperature in Celsius
float pidKp = 30.0;                 // PID constants
float pidKi = 0.2;
float pidKd = 80.0;

float inputVoltage = 5.0;           // Will read from CH224K (dummy ADC or resistor voltage divider)
float measuredTemp = 0.0;           // Thermocouple reading
bool tipConnected = false;
bool heatingEnabled = false;
bool sleepMode = false;

// ---------- PID CONTROL VARIABLES ----------
float pidOutput = 0;
float previousError = 0;
float integral = 0;

// ---------- Button State ----------
bool buttonOkPressed = false;
bool buttonFwdPressed = false;
bool buttonBwdPressed = false;

// ---------- Menu Variables ----------
enum MenuState { MAIN_SCREEN, SET_PID, SET_VOLTAGE };
MenuState menuState = MAIN_SCREEN;

// ---------- Timing ----------
unsigned long lastUpdateTime = 0;
unsigned long displayUpdateInterval = 200; // ms
unsigned long tipDetectTimeout = 1000;     // ms
unsigned long lastMovementTime = 0;
unsigned long sleepDelay = 300000;        // 5 minutes

// ---------- Helper Functions ----------

// Read thermocouple ADC and convert to temperature (dummy conversion)
float readThermocouple() {
  int adc = analogRead(PIN_THERMOCOUPLE);
  if(adc < 5) return NAN; // Assume tip disconnected if very low
  float voltage = adc * (5.0 / 1023.0);
  float tempC = voltage * 100; // Adjust according to thermocouple calibration
  return tempC;
}

// Read voltage from CH224K configuration (dummy example)
float readInputVoltage() {
  // In reality, read a pin or read resistor divider
  return inputVoltage;
}

// PID calculation
float computePID(float setpoint, float measured) {
  float error = setpoint - measured;
  integral += error * 0.2; // dt simplified
  float derivative = error - previousError;
  previousError = error;
  float output = pidKp*error + pidKi*integral + pidKd*derivative;
  output = constrain(output, 0, 255);
  return output;
}

// Update display
void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if(sleepMode) {
    display.setTextSize(2);
    display.setCursor(0,10);
    display.println("SLEEP");
    display.display();
    return;
  }

  if(menuState == MAIN_SCREEN) {
    display.setTextSize(2);
    display.setCursor(0,0);
    if(isnan(measuredTemp)) {
      display.println("NO TIP");
    } else {
      display.print(measuredTemp, 0);
      display.println((char)247); // degree symbol
      display.print("C");
    }

    display.setTextSize(1);
    display.setCursor(0,25);
    display.print(inputVoltage,1);
    display.println(" V");
  } else if(menuState == SET_PID) {
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Adjust PID");
    display.setCursor(0,12);
    display.print("Kp: "); display.println(pidKp);
    display.setCursor(0,20);
    display.print("Ki: "); display.println(pidKi);
    display.setCursor(0,28);
    display.print("Kd: "); display.println(pidKd);
  } else if(menuState == SET_VOLTAGE) {
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Voltage Select");
    display.setCursor(0,12);
    display.print("Conf1: "); display.println(digitalRead(PIN_CH224K_CONF1));
    display.setCursor(0,20);
    display.print("Conf2: "); display.println(digitalRead(PIN_CH224K_CONF2));
    display.setCursor(0,28);
    display.print("Conf3: "); display.println(digitalRead(PIN_CH224K_CONF3));
  }

  display.display();
}

// Read buttons with simple debouncing
bool readButton(int pin) {
  return digitalRead(pin) == LOW; // Assuming pull-up
}

// Go to sleep
void enterSleep() {
  digitalWrite(PIN_TIP_POWER, LOW);
  sleepMode = true;
  updateDisplay();
}

// Wake up
void wakeUp() {
  sleepMode = false;
}

// LIS3DH interrupt handler
void lis3dhISR() {
  wakeUp();
  lastMovementTime = millis();
}

// ---------- Setup ----------
void setup() {
  pinMode(PIN_BUTTON_OK, INPUT_PULLUP);
  pinMode(PIN_BUTTON_FWD, INPUT_PULLUP);
  pinMode(PIN_BUTTON_BWD, INPUT_PULLUP);

  pinMode(PIN_CH224K_CONF1, OUTPUT);
  pinMode(PIN_CH224K_CONF2, OUTPUT);
  pinMode(PIN_CH224K_CONF3, OUTPUT);

  pinMode(PIN_TIP_POWER, OUTPUT);
  digitalWrite(PIN_TIP_POWER, LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_LIS3DH_INT), lis3dhISR, RISING);

  Wire.begin();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // SSD1306 init failed
    while(1);
  }

  if(!lis.begin(0x18)) {  // I2C address
    while(1);
  }
  lis.setRange(LIS3DH_RANGE_2_G);
  lis.setDataRate(LIS3DH_DATARATE_50_HZ);
}

// ---------- Main Loop ----------
void loop() {
  // Handle sleep
  if(!sleepMode && (millis() - lastMovementTime > sleepDelay)) {
    enterSleep();
  }

  // Read thermocouple
  measuredTemp = readThermocouple();

  // Detect tip connection
  if(!tipConnected && !isnan(measuredTemp)) {
    tipConnected = true;
    // Wait for user to press OK to start heating
    if(readButton(PIN_BUTTON_OK)) {
      heatingEnabled = true;
      digitalWrite(PIN_TIP_POWER, HIGH);
    }
  }

  // Heating control
  if(heatingEnabled && !sleepMode && !isnan(measuredTemp)) {
    pidOutput = computePID(setTemperature, measuredTemp);
    analogWrite(PIN_TIP_POWER, (int)pidOutput); // Adjust with MOSFET or PWM circuit
  }

  // Menu navigation
  buttonOkPressed = readButton(PIN_BUTTON_OK);
  buttonFwdPressed = readButton(PIN_BUTTON_FWD);
  buttonBwdPressed = readButton(PIN_BUTTON_BWD);

  if(buttonOkPressed) {
    // Long press can open settings
    static unsigned long pressStart = 0;
    if(pressStart == 0) pressStart = millis();
    if(millis() - pressStart > 1500) { // 1.5 sec long press
      menuState = (menuState == MAIN_SCREEN) ? SET_PID : MAIN_SCREEN;
      pressStart = 0;
    }
  } else {
    pressStart = 0;
  }

  // Forward/backward adjust values in menu
  if(menuState == SET_PID) {
    if(buttonFwdPressed) pidKp += 1.0;
    if(buttonBwdPressed) pidKp -= 1.0;
  } else if(menuState == SET_VOLTAGE) {
    if(buttonFwdPressed) digitalWrite(PIN_CH224K_CONF1, !digitalRead(PIN_CH224K_CONF1));
    if(buttonBwdPressed) digitalWrite(PIN_CH224K_CONF2, !digitalRead(PIN_CH224K_CONF2));
  }

  // Update display periodically
  if(millis() - lastUpdateTime > displayUpdateInterval) {
    lastUpdateTime = millis();
    updateDisplay();
  }
}