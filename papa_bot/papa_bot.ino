/*
  =============================================================================
  PROJECT: PAPA BOT - 5-Sensor Line Follower, Maze Solver & Logic Tester
  BOARD:   Arduino Nano (ATmega328P)
  MOTOR DRIVER: TB6612FNG Dual H-Bridge Driver
  DISPLAY: 0.96" I2C OLED (SSD1306, 128x64)
  
  PIN CONFIGURATION (Strictly matching updated pin diagram):
  -----------------------------------------------------------------
  I2C Display:
    - SDA   -> A4
    - SCL   -> A5
  
  Motor Driver TB6612FNG:
    - PWMB  -> D9 (PWM Motor 1 / Right Motor)
    - BIN1  -> D8 (Motor 1 Direction 1 - Interchanged)
    - BIN2  -> D7 (Motor 1 Direction 2 - Interchanged)
    - PWMA  -> D6 (PWM Motor 2 / Left Motor)
    - AIN1  -> D5 (Motor 2 Direction 1 - Interchanged)
    - AIN2  -> D4 (Motor 2 Direction 2 - Interchanged)
    - STNBY -> D10 (Motor Standby, active HIGH)
    
  LEDs:
    - Red LED   -> D2 (Stop / Calibration / Standby status)
    - Green LED -> D3 (Running / Active status)
    
  Sensors (5-Array Analog Line Sensors):
    - S0 -> A0 (Far Left)
    - S1 -> A1 (Mid Left)
    - S2 -> A2 (Center)
    - S3 -> A3 (Mid Right)
    - S4 -> A6 (Far Right - Analog only on Nano)
    
  Buttons:
    - BNT1 -> D12 (Move selection / Next)
    - BNT2 -> D11 (Select / OK)
  =============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -----------------------------------------------------------------------------
// OLED DISPLAY CONFIGURATION
// -----------------------------------------------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledReady = false;

// -----------------------------------------------------------------------------
// PIN DEFINITIONS (Interchanged as requested)
// -----------------------------------------------------------------------------
// Motor 2 / Left Motor (Channel A)
const byte PWMA_PIN = 6;
const byte AIN1_PIN = 5; // Interchanged (was 4)
const byte AIN2_PIN = 4; // Interchanged (was 5)

// Motor 1 / Right Motor (Channel B)
const byte PWMB_PIN = 9;
const byte BIN1_PIN = 8; // Interchanged (was 7)
const byte BIN2_PIN = 7; // Interchanged (was 8)

// Driver Standby
const byte STBY_PIN = 10;

// Status LEDs
const byte RED_LED_PIN   = 2;
const byte GREEN_LED_PIN = 3;

// Pushbuttons (Active LOW with internal pull-ups)
const byte BNT1_PIN = 12; // Move selection
const byte BNT2_PIN = 11; // Select / OK

// Sensor array pins: S0 (Far Left) to S4 (Far Right)
const byte sensorPins[5] = {A0, A1, A2, A3, A6};

// -----------------------------------------------------------------------------
// USER CONFIGURATIONS & SPEED TUNING
// -----------------------------------------------------------------------------
const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;

// True: analog reading ABOVE threshold indicates black line
// False: analog reading BELOW threshold indicates black line
const bool BLACK_IS_HIGH = true;

// Default threshold if dynamic calibration difference is too small
int sensorThreshold[5] = {500, 500, 500, 500, 500};
int sensorMin[5]       = {1023, 1023, 1023, 1023, 1023};
int sensorMax[5]       = {0, 0, 0, 0, 0};

// Motor Speeds (0 - 255)
const int BASE_SPEED     = 80;
const int MAX_SPEED      = 100;
const int TURN_SPEED     = 50;
const int CAL_SPEED      = 50;
const int INCH_SPEED     = 95;

// Timings
const unsigned long INCH_TIME_MS    = 180;  // Time to inch forward at junctions
const unsigned long TURN_TIMEOUT_MS = 2500; // Safety turn timeout

// PID Coefficients
float Kp = 26.0;
float Ki = 0.0;
float Kd = 14.0;

float previousError = 0;
float integral      = 0;

// -----------------------------------------------------------------------------
// MENU & SYSTEM STATE ENUMS
// -----------------------------------------------------------------------------
enum RobotMode {
  MODE_LINE_FOLLOWER = 0,
  MODE_MAZE_SOLVER   = 1,
  MODE_LOGIC_TESTER  = 2
};

enum JunctionAction {
  ACTION_STOP         = 0,
  ACTION_TURN_LEFT    = 1,
  ACTION_TURN_RIGHT   = 2,
  ACTION_INCH_FORWARD = 3,
  ACTION_START_PID    = 4
};

enum DetectedCase {
  CASE_STRAIGHT,
  CASE_LEFT_TURN,
  CASE_RIGHT_TURN,
  CASE_T_JUNCTION,
  CASE_CROSS_JUNCTION,
  CASE_DEAD_END,
  CASE_UNKNOWN
};

// Mode & Action Names stored in PROGMEM to save RAM on ATmega328P
const char mName0[] PROGMEM = "Line Follower";
const char mName1[] PROGMEM = "Maze Solver";
const char mName2[] PROGMEM = "Logic Tester";
const char* const modeNames[] PROGMEM = {mName0, mName1, mName2};

const char aName0[] PROGMEM = "Stop";
const char aName1[] PROGMEM = "Turn Left";
const char aName2[] PROGMEM = "Turn Right";
const char aName3[] PROGMEM = "Inch Forward";
const char aName4[] PROGMEM = "Start PID";
const char* const actionNames[] PROGMEM = {aName0, aName1, aName2, aName3, aName4};

RobotMode selectedMode       = MODE_LINE_FOLLOWER;
JunctionAction testerAction  = ACTION_STOP;
bool robotRunning            = false;

// Sensor reading variables
int  sensorRaw[5];
bool sensorBit[5];
byte sensorMask   = 0; // 5-bit binary mask: bit4=S0 ... bit0=S4
int  linePosition = 0;
const int sensorWeights[5] = {-2000, -1000, 0, 1000, 2000};

unsigned long lastDisplayUpdate = 0;

// -----------------------------------------------------------------------------
// FREE RAM CHECK (AVR Diagnostic)
// -----------------------------------------------------------------------------
int getFreeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// -----------------------------------------------------------------------------
// HARDWARE INITIALIZATION & UTILITIES
// -----------------------------------------------------------------------------
void setStoppedLEDs() {
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);
}

void setRunningLEDs() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
}

bool buttonPressed(byte pin) {
  if (digitalRead(pin) == LOW) {
    delay(25); // Debounce
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW) {
        delay(5); // Wait for release
      }
      delay(25);
      return true;
    }
  }
  return false;
}

// -----------------------------------------------------------------------------
// MOTOR CONTROL FUNCTIONS (TB6612FNG)
// -----------------------------------------------------------------------------
// TB6612FNG Short Brake Mode:
// IN1 = HIGH, IN2 = HIGH, PWM = 255
// Electrically shorts motor terminals to achieve instant dynamic braking
void brakeMotors() {
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, HIGH);
  analogWrite(PWMA_PIN, 255);

  digitalWrite(BIN1_PIN, HIGH);
  digitalWrite(BIN2_PIN, HIGH);
  analogWrite(PWMB_PIN, 255);
}

// Active counter-torque pulse to cancel kinetic momentum, followed by short brake
void activeBrake(unsigned long reversePulseMs = 30) {
  drive(-BASE_SPEED, -BASE_SPEED);
  delay(reversePulseMs);
  brakeMotors();
}

void setLeftMotor(int speed) {
  if (INVERT_LEFT_MOTOR) speed = -speed;
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    analogWrite(PWMA_PIN, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, -speed);
  } else {
    // Short brake instead of coasting
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, 255);
  }
}

void setRightMotor(int speed) {
  if (INVERT_RIGHT_MOTOR) speed = -speed;
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
    analogWrite(PWMB_PIN, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, -speed);
  } else {
    // Short brake instead of coasting
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, 255);
  }
}

void drive(int leftSpeed, int rightSpeed) {
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

void stopMotors() {
  brakeMotors();
}

void forwardFor(unsigned long ms, int speed) {
  drive(speed, speed);
  delay(ms);
  // Counter-pulse to kill momentum immediately, then lock short brake
  drive(-speed, -speed);
  delay(25);
  brakeMotors();
}

void backwardFor(unsigned long ms, int speed) {
  drive(-speed, -speed);
  delay(ms);
  drive(speed, speed);
  delay(25);
  brakeMotors();
}

void rotateLeft() {
  drive(-TURN_SPEED, TURN_SPEED);
}

void rotateRight() {
  drive(TURN_SPEED, -TURN_SPEED);
}

void inchForward() {
  forwardFor(INCH_TIME_MS, INCH_SPEED);
}

// -----------------------------------------------------------------------------
// SENSOR READING & BINARY CONVERSION
// -----------------------------------------------------------------------------
void readSensors() {
  sensorMask = 0;
  long weightedSum = 0;
  int activeCount = 0;

  for (byte i = 0; i < 5; i++) {
    sensorRaw[i] = analogRead(sensorPins[i]);

    if (BLACK_IS_HIGH) {
      sensorBit[i] = (sensorRaw[i] >= sensorThreshold[i]);
    } else {
      sensorBit[i] = (sensorRaw[i] <= sensorThreshold[i]);
    }

    // Bit 4 is S0 (Far Left), Bit 0 is S4 (Far Right)
    if (sensorBit[i]) {
      sensorMask |= (1 << (4 - i));
      weightedSum += sensorWeights[i];
      activeCount++;
    }
  }

  if (activeCount > 0) {
    linePosition = weightedSum / activeCount;
  }
}

bool centerOnBlack() {
  readSensors();
  return sensorBit[2]; // S2 is center sensor
}

// -----------------------------------------------------------------------------
// BINARY CASE DETECTION
// -----------------------------------------------------------------------------
DetectedCase detectCurrentCase() {
  readSensors();

  // Dead End: All sensors see white (00000)
  if (sensorMask == 0b00000) {
    return CASE_DEAD_END;
  }

  // Cross Junction: All 5 or wide 4 sensors active (11111)
  if (sensorMask == 0b11111 || sensorMask == 0b11110 || sensorMask == 0b01111) {
    return CASE_CROSS_JUNCTION;
  }

  // T-Junction: Both outer sensors active and center active
  if ((sensorBit[0] && sensorBit[4]) && (sensorBit[1] || sensorBit[2] || sensorBit[3])) {
    return CASE_T_JUNCTION;
  }

  // Left Turn / Left Branch: Leftmost sensors active, rightmost clear
  if ((sensorBit[0] && sensorBit[1]) && !sensorBit[4]) {
    return CASE_LEFT_TURN;
  }

  // Right Turn / Right Branch: Rightmost sensors active, leftmost clear
  if ((sensorBit[3] && sensorBit[4]) && !sensorBit[0]) {
    return CASE_RIGHT_TURN;
  }

  // Straight / Curved line follower patterns (00100, 01100, 00110, 01000, 00010)
  if (sensorBit[2] || sensorBit[1] || sensorBit[3]) {
    return CASE_STRAIGHT;
  }

  return CASE_UNKNOWN;
}

bool isJunction(DetectedCase detected) {
  return (detected == CASE_LEFT_TURN ||
          detected == CASE_RIGHT_TURN ||
          detected == CASE_T_JUNCTION ||
          detected == CASE_CROSS_JUNCTION ||
          detected == CASE_DEAD_END);
}

const __FlashStringHelper* getCaseNameF(DetectedCase c) {
  switch (c) {
    case CASE_STRAIGHT:       return F("STRAIGHT");
    case CASE_LEFT_TURN:      return F("LEFT TURN");
    case CASE_RIGHT_TURN:     return F("RIGHT TURN");
    case CASE_T_JUNCTION:     return F("T JUNCTION");
    case CASE_CROSS_JUNCTION: return F("CROSS (4-WAY)");
    case CASE_DEAD_END:       return F("DEAD END");
    default:                  return F("LINE LOST");
  }
}

// -----------------------------------------------------------------------------
// OLED DISPLAY RENDERING (Continuous Sensor Binary Display)
// -----------------------------------------------------------------------------
void drawBinaryFooter() {
  display.drawLine(0, 48, 127, 48, SSD1306_WHITE);
  display.setCursor(0, 53);
  display.print(F("BIN:"));

  // Draw 5 sensor graphical blocks & binary numbers
  for (byte i = 0; i < 5; i++) {
    int boxX = 32 + i * 19;
    if (sensorBit[i]) {
      display.fillRect(boxX, 51, 14, 11, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(boxX + 4, 53);
      display.print(F("1"));
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.drawRect(boxX, 51, 14, 11, SSD1306_WHITE);
      display.setCursor(boxX + 4, 53);
      display.print(F("0"));
    }
  }
}

void renderDisplay(const __FlashStringHelper* header, const __FlashStringHelper* line1, const __FlashStringHelper* line2) {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(header);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  if (line1 != NULL) {
    display.setCursor(0, 15);
    display.print(line1);
  }

  if (line2 != NULL) {
    display.setCursor(0, 28);
    display.print(line2);
  }

  drawBinaryFooter();
  display.display();
}

void updateRunningDisplay(const __FlashStringHelper* currentAction, DetectedCase currentCase) {
  if (!oledReady) return;

  unsigned long now = millis();
  if (now - lastDisplayUpdate < 60) return; // Smooth refresh rate
  lastDisplayUpdate = now;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (selectedMode == MODE_LINE_FOLLOWER) {
    display.print(F("LINE FOLLOWER"));
  } else if (selectedMode == MODE_MAZE_SOLVER) {
    display.print(F("MAZE SOLVER"));
  } else {
    display.print(F("LOGIC TESTER"));
  }

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 15);
  display.print(F("CASE: "));
  display.print(getCaseNameF(currentCase));

  display.setCursor(0, 28);
  display.print(F("ACT : "));
  display.print(currentAction);

  drawBinaryFooter();
  display.display();
}

// -----------------------------------------------------------------------------
// PID LINE FOLLOWING
// -----------------------------------------------------------------------------
void resetPID() {
  previousError = 0;
  integral      = 0;
}

void followLinePID() {
  float error = (float)linePosition / 1000.0;

  integral += error;
  integral = constrain(integral, -20.0, 20.0);

  float derivative = error - previousError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  previousError = error;

  int leftSpeed  = constrain((int)(BASE_SPEED + correction), -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain((int)(BASE_SPEED - correction), -MAX_SPEED, MAX_SPEED);

  drive(leftSpeed, rightSpeed);
}

// -----------------------------------------------------------------------------
// TURN LOGIC (Left-First & Center Lock)
// -----------------------------------------------------------------------------
bool turnUntilCenterOnBlack(bool turnLeft) {
  unsigned long start = millis();
  bool centerLeftLine = false;

  if (turnLeft) {
    rotateLeft();
  } else {
    rotateRight();
  }

  // Blind spin for 120ms to swing away from current black line
  delay(120);

  while (millis() - start < TURN_TIMEOUT_MS) {
    readSensors();
    updateRunningDisplay(turnLeft ? F("TURNING LEFT") : F("TURNING RIGHT"), detectCurrentCase());

    if (!centerOnBlack()) {
      centerLeftLine = true;
    }

    // Stop once center sensor locks onto black line after clearing original line
    if (centerLeftLine && centerOnBlack()) {
      stopMotors();
      resetPID();
      delay(40);
      return true;
    }

    if (turnLeft) {
      rotateLeft();
    } else {
      rotateRight();
    }
  }

  stopMotors();
  Serial.println(F("Turn timeout reached!"));
  return false;
}

// -----------------------------------------------------------------------------
// MAZE SOLVER LOGIC (Left-Hand Rule Priority)
// -----------------------------------------------------------------------------
void handleMazeJunction(DetectedCase initialCase) {
  // 1. Immediately active brake to kill forward momentum
  activeBrake(30);
  Serial.println(F("Junction detected: PID suspended."));
  updateRunningDisplay(F("JUNCTION INCH"), initialCase);

  // 2. Inch forward to align wheel axis with intersection
  inchForward();

  // 3. Re-read sensors after inching to see available paths ahead
  DetectedCase postInchCase = detectCurrentCase();

  // LEFT-HAND RULE:
  // Priority 1: Left Turn if left was detected at junction
  if (initialCase == CASE_LEFT_TURN || initialCase == CASE_T_JUNCTION || initialCase == CASE_CROSS_JUNCTION) {
    Serial.println(F("Maze: Taking LEFT turn."));
    updateRunningDisplay(F("TURN LEFT"), initialCase);
    turnUntilCenterOnBlack(true);
    return;
  }

  // Priority 2: Straight if path ahead is clear and center is on black
  if (centerOnBlack()) {
    Serial.println(F("Maze: Going STRAIGHT."));
    updateRunningDisplay(F("GO STRAIGHT"), initialCase);
    resetPID();
    return;
  }

  // Priority 3: Right Turn
  if (initialCase == CASE_RIGHT_TURN || postInchCase == CASE_RIGHT_TURN) {
    Serial.println(F("Maze: Taking RIGHT turn."));
    updateRunningDisplay(F("TURN RIGHT"), initialCase);
    turnUntilCenterOnBlack(false);
    return;
  }

  // Priority 4: Dead End - Turn Around (180 deg)
  Serial.println(F("Maze: DEAD END - Turning around."));
  updateRunningDisplay(F("U-TURN (180)"), initialCase);
  turnUntilCenterOnBlack(true);
  turnUntilCenterOnBlack(true);
}

// -----------------------------------------------------------------------------
// LOGIC TESTER HANDLER
// -----------------------------------------------------------------------------
void waitInLogicTesterResume(const __FlashStringHelper* reason) {
  stopMotors();
  setStoppedLEDs();

  Serial.println(F("Logic Tester Stopped. Press BNT2 to run again."));

  while (true) {
    readSensors();

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("LOGIC TESTER"));
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    display.setCursor(0, 15);
    display.print(reason);

    display.setCursor(0, 28);
    display.print(F("Press B2 to run"));

    drawBinaryFooter();
    display.display();

    if (buttonPressed(BNT2_PIN)) {
      setRunningLEDs();
      resetPID();
      Serial.println(F("Logic Tester resumed by BNT2!"));
      delay(150);
      return;
    }
    delay(40);
  }
}

void handleTesterJunction(DetectedCase c) {
  activeBrake(30);
  Serial.println(F("Logic Tester Junction detected!"));

  // Mandatory Inch Forward after junction of any kind (including left/right turns)
  updateRunningDisplay(F("INCHING FORWARD"), c);
  inchForward();

  // Execute selected tester action
  switch (testerAction) {
    case ACTION_STOP:
      updateRunningDisplay(F("ACTION: STOPPED"), c);
      waitInLogicTesterResume(F("TEST: STOPPED"));
      break;

    case ACTION_TURN_LEFT:
      updateRunningDisplay(F("ACTION: TURN LEFT"), c);
      turnUntilCenterOnBlack(true);
      stopMotors();
      waitInLogicTesterResume(F("LEFT TURN DONE"));
      break;

    case ACTION_TURN_RIGHT:
      updateRunningDisplay(F("ACTION: TURN RIGHT"), c);
      turnUntilCenterOnBlack(false);
      stopMotors();
      waitInLogicTesterResume(F("RIGHT TURN DONE"));
      break;

    case ACTION_INCH_FORWARD:
      updateRunningDisplay(F("ACTION: INCH AGAIN"), c);
      inchForward();
      stopMotors();
      waitInLogicTesterResume(F("INCH FORWARD DONE"));
      break;

    case ACTION_START_PID:
      updateRunningDisplay(F("ACTION: START PID"), c);
      resetPID();
      break;
  }
}

// -----------------------------------------------------------------------------
// CALIBRATION SEQUENCE
// -----------------------------------------------------------------------------
void calibrateRobot() {
  Serial.println(F("Calibration sequence initialized."));
  setStoppedLEDs();
  stopMotors();

  // 2-second delay before calibration movement
  unsigned long calWaitStart = millis();
  while (millis() - calWaitStart < 2000) {
    readSensors();
    int remaining = 2 - ((millis() - calWaitStart) / 1000);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("CALIBRATION"));
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
    display.setCursor(0, 16);
    display.print(F("Starting in: "));
    display.print(remaining);
    display.print(F("s"));
    display.setCursor(0, 30);
    display.print(F("Keep wheels clear"));
    drawBinaryFooter();
    display.display();
    delay(50);
  }

  // Initialize min/max values
  for (byte i = 0; i < 5; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  // Move forward for 500ms while sampling
  Serial.println(F("Calibrating: Moving forward 500ms"));
  drive(CAL_SPEED, CAL_SPEED);
  unsigned long moveStart = millis();
  while (millis() - moveStart < 500) {
    for (byte i = 0; i < 5; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    readSensors();
    renderDisplay(F("CALIBRATION"), F("Moving Forward..."), F("Sampling Line..."));
    delay(5);
  }
  stopMotors();
  delay(100);

  // Move backward for 500ms while sampling
  Serial.println(F("Calibrating: Moving backward 500ms"));
  drive(-CAL_SPEED, -CAL_SPEED);
  moveStart = millis();
  while (millis() - moveStart < 500) {
    for (byte i = 0; i < 5; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    readSensors();
    renderDisplay(F("CALIBRATION"), F("Moving Backward..."), F("Sampling Line..."));
    delay(5);
  }
  stopMotors();

  // Calculate dynamic threshold per sensor
  for (byte i = 0; i < 5; i++) {
    if ((sensorMax[i] - sensorMin[i]) > 100) {
      sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
    } else {
      sensorThreshold[i] = 500; // Fallback default
    }
  }

  Serial.println(F("Calibration complete. Thresholds:"));
  for (byte i = 0; i < 5; i++) {
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(sensorThreshold[i]);
    Serial.print(F("  "));
  }
  Serial.println();

  resetPID();
}

// -----------------------------------------------------------------------------
// USER INPUT & MENU SYSTEM
// -----------------------------------------------------------------------------
void showMainMenu(byte currentSelection) {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("SELECT MODE:"));
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  char buffer[20];
  for (byte i = 0; i < 3; i++) {
    display.setCursor(0, 14 + (i * 11));
    display.print((i == currentSelection) ? F("> ") : F("  "));
    strcpy_P(buffer, (char*)pgm_read_ptr(&(modeNames[i])));
    display.print(buffer);
  }

  display.drawLine(0, 48, 127, 48, SSD1306_WHITE);
  display.setCursor(0, 52);
  display.print(F("B1:MOVE   B2:SELECT"));

  display.display();
}

void showTesterActionMenu(byte currentSelection) {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("JUNCTION ACTION:"));
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  char buffer[20];
  for (byte i = 0; i < 5; i++) {
    display.setCursor(0, 11 + (i * 8));
    display.print((i == currentSelection) ? F(">") : F(" "));
    strcpy_P(buffer, (char*)pgm_read_ptr(&(actionNames[i])));
    display.print(buffer);
  }

  display.drawLine(0, 53, 127, 53, SSD1306_WHITE);
  display.setCursor(0, 56);
  display.print(F("B1:MOVE   B2:OK"));

  display.display();
}

void waitForStartInput() {
  stopMotors();
  setStoppedLEDs();

  Serial.println(F("Ready. Press BNT2 to start execution!"));

  char actBuf[20];
  strcpy_P(actBuf, (char*)pgm_read_ptr(&(actionNames[testerAction])));

  // Continuously update OLED with live binary sensor values while waiting
  while (true) {
    readSensors();

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);

    if (selectedMode == MODE_LINE_FOLLOWER) {
      display.print(F("LINE FOLLOWER"));
      display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      display.setCursor(0, 15);
      display.print(F("Press BNT2 to start:"));
      display.setCursor(0, 28);
      display.print(F("Line Follower Mode"));
    } else if (selectedMode == MODE_MAZE_SOLVER) {
      display.print(F("MAZE SOLVER"));
      display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      display.setCursor(0, 15);
      display.print(F("Press BNT2 to start:"));
      display.setCursor(0, 28);
      display.print(F("Explore Maze"));
    } else {
      display.print(F("LOGIC TESTER"));
      display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
      display.setCursor(0, 15);
      display.print(F("Press BNT2 to start:"));
      display.setCursor(0, 28);
      display.print(F("Act: "));
      display.print(actBuf);
    }

    drawBinaryFooter();
    display.display();

    if (buttonPressed(BNT2_PIN)) {
      robotRunning = true;
      setRunningLEDs();
      resetPID();
      Serial.println(F("Execution started!"));
      return;
    }
    delay(40);
  }
}

void runMenuSequence() {
  byte menuIndex = 0;

  // 1. Main Mode Menu (Line Follower, Maze Solver, Logic Tester)
  while (true) {
    showMainMenu(menuIndex);

    if (buttonPressed(BNT1_PIN)) {
      menuIndex = (menuIndex + 1) % 3;
    }

    if (buttonPressed(BNT2_PIN)) {
      selectedMode = (RobotMode)menuIndex;
      break;
    }
    delay(20);
  }

  // 2. Logic Tester Sub-Menu (if Logic Tester selected)
  if (selectedMode == MODE_LOGIC_TESTER) {
    byte actionIndex = 0;
    while (true) {
      showTesterActionMenu(actionIndex);

      if (buttonPressed(BNT1_PIN)) {
        actionIndex = (actionIndex + 1) % 5;
      }

      if (buttonPressed(BNT2_PIN)) {
        testerAction = (JunctionAction)actionIndex;
        break;
      }
      delay(20);
    }
  }

  // 3. Automatic 2s delay + Forward/Backward Calibration
  calibrateRobot();

  // 4. Wait for BNT2 to start execution with live binary display
  waitForStartInput();
}

// -----------------------------------------------------------------------------
// MAIN MODES EXECUTION
// -----------------------------------------------------------------------------
void executeLineFollower() {
  DetectedCase currentCase = detectCurrentCase();

  // If a junction of any kind is detected, stop PID and inch through
  if (isJunction(currentCase)) {
    activeBrake(30);
    Serial.println(F("Line Follower: Junction hit. Stopping PID."));
    updateRunningDisplay(F("JUNCTION INCH"), currentCase);

    inchForward();

    // Move past the junction line
    unsigned long passStart = millis();
    while (isJunction(detectCurrentCase()) && (millis() - passStart < 800)) {
      drive(70, 70);
      updateRunningDisplay(F("CLEARING JUNCT"), currentCase);
      delay(10);
    }
    stopMotors();
    resetPID();
    return;
  }

  // If completely lost line
  if (currentCase == CASE_DEAD_END) {
    stopMotors();
    updateRunningDisplay(F("LINE LOST / STOP"), currentCase);
    return;
  }

  // Follow straight or curved path using PID
  followLinePID();
  updateRunningDisplay(F("PID FOLLOWING"), currentCase);
}

void executeMazeSolver() {
  DetectedCase currentCase = detectCurrentCase();

  // When a junction or dead end is detected, stop PID and execute Left-First Maze logic
  if (isJunction(currentCase)) {
    handleMazeJunction(currentCase);
    return;
  }

  // Straight line or smooth curves -> PID line follower active
  followLinePID();
  updateRunningDisplay(F("PID FOLLOWING"), currentCase);
}

void executeLogicTester() {
  DetectedCase currentCase = detectCurrentCase();

  // When a junction or turn is detected:
  if (isJunction(currentCase)) {
    handleTesterJunction(currentCase);
    return;
  }

  // If completely lost line
  if (currentCase == CASE_DEAD_END) {
    waitInLogicTesterResume(F("LINE LOST"));
    return;
  }

  // On normal line: follow PID
  followLinePID();
  updateRunningDisplay(F("PID FOLLOWING"), currentCase);
}

// -----------------------------------------------------------------------------
// I2C ADDRESS DETECTOR (Finds 0x3C or 0x3D safely without RAM leak)
// -----------------------------------------------------------------------------
byte detectOLEDAddress() {
  // Check 0x3D first (as in original user code), then 0x3C
  Wire.beginTransmission(0x3D);
  if (Wire.endTransmission() == 0) return 0x3D;

  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() == 0) return 0x3C;

  // General bus probe
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      return addr;
    }
  }
  return 0;
}

// -----------------------------------------------------------------------------
// ARDUINO SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.print(F("Free RAM before OLED: "));
  Serial.print(getFreeRam());
  Serial.println(F(" bytes"));

  // Status LEDs & Motors initialized early for immediate boot feedback
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  setStoppedLEDs();

  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);

  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH); // Enable motor driver
  stopMotors();

  pinMode(BNT1_PIN, INPUT_PULLUP);
  pinMode(BNT2_PIN, INPUT_PULLUP);

  for (byte i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Standard 100kHz I2C for high reliability with jumper wires
  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  byte targetAddr = detectOLEDAddress();
  Serial.print(F("Detected OLED I2C Address: 0x"));
  Serial.println(targetAddr, HEX);

  if (targetAddr == 0) {
    // Fallback: If bus didn't acknowledge during probe, try default 0x3D then 0x3C
    targetAddr = 0x3D;
  }

  // Call display.begin ONCE for the detected address to prevent RAM exhaustion
  if (display.begin(SSD1306_SWITCHCAPVCC, targetAddr)) {
    oledReady = true;
    Serial.println(F("SSD1306 OLED initialized successfully!"));
  } else if (targetAddr != 0x3C && display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledReady = true;
    Serial.println(F("SSD1306 OLED initialized on fallback 0x3C!"));
  } else {
    Serial.println(F("OLED init failed! Check SDA(A4), SCL(A5), VCC, GND."));
    oledReady = false;
  }

  Serial.print(F("Free RAM after OLED: "));
  Serial.print(getFreeRam());
  Serial.println(F(" bytes"));

  if (oledReady) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(18, 15);
    display.print(F("PAPA BOT V2.0"));
    display.setCursor(8, 35);
    display.print(F("Robotics System"));
    display.drawLine(0, 50, 127, 50, SSD1306_WHITE);
    display.display();
    delay(1200);
  }

  // Launch Menu, Calibration, and Wait for Start
  runMenuSequence();
}

// -----------------------------------------------------------------------------
// ARDUINO MAIN LOOP
// -----------------------------------------------------------------------------
void loop() {
  if (!robotRunning) {
    stopMotors();
    return;
  }

  switch (selectedMode) {
    case MODE_LINE_FOLLOWER:
      executeLineFollower();
      break;

    case MODE_MAZE_SOLVER:
      executeMazeSolver();
      break;

    case MODE_LOGIC_TESTER:
      executeLogicTester();
      break;
  }
}
