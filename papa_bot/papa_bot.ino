
/*
  5-SENSOR LINE FOLLOWER + LEFT-HAND MAZE SOLVER
  Board: Arduino Nano (classic ATmega328P)
  Motor driver: TB6612FNG

  BNT1 = D12 (next menu item)
  BNT2 = D11 (select / OK)

  Sensor order: S0 LEFT ... S4 RIGHT
  Binary pattern example:
  00100 = center sensor sees black

  IMPORTANT:
  - Tune SENSOR_THRESHOLD for your sensors.
  - Verify motor direction before placing the robot on a track.
*/

#include <Arduino.h>

// -------------------- PIN DEFINITIONS --------------------

const byte PWMA = 6;
const byte AIN1 = 4;
const byte AIN2 = 5;

const byte PWMB = 9;
const byte BIN1 = 7;
const byte BIN2 = 8;

const byte STBY = 10;

const byte RED_LED   = 2;
const byte GREEN_LED = 3;

const byte BNT1 = 12;
const byte BNT2 = 11;

const byte sensorPins[5] = {A0, A1, A2, A3, A6};

// -------------------- SENSOR SETTINGS --------------------

// Adjust after viewing sensor readings in Serial Monitor.
const int SENSOR_THRESHOLD = 500;

// true: analog value ABOVE threshold means black.
// false: analog value BELOW threshold means black.
const bool BLACK_IS_HIGH = true;

// Set true if the robot drives away from the line.
const bool REVERSE_SENSOR_ORDER = false;

// -------------------- MOTOR SETTINGS --------------------

const int BASE_SPEED = 115;
const int MAX_SPEED  = 190;
const int TURN_SPEED = 110;
const int CAL_SPEED  = 100;
const int INCH_SPEED = 95;

const unsigned long INCH_TIME_MS = 120;
const unsigned long LOST_LINE_MS = 120;
const unsigned long TURN_TIMEOUT_MS = 2500;

// -------------------- PID SETTINGS --------------------

// Starting values; tune on your actual robot.
float Kp = 24.0;
float Ki = 0.0;
float Kd = 12.0;

float previousError = 0;
float integral = 0;

// -------------------- MENU MODES --------------------

enum RobotMode {
  LINE_FOLLOWER,
  MAZE_SOLVER,
  LOGIC_TESTER
};

enum JunctionAction {
  ACTION_STOP,
  ACTION_LEFT,
  ACTION_RIGHT,
  ACTION_INCH,
  ACTION_PID
};

const char* modeNames[] = {
  "Line Follower",
  "Maze Solver",
  "Logic Tester"
};

const char* actionNames[] = {
  "Stop",
  "Turn Left",
  "Turn Right",
  "Inch Forward",
  "Start PID"
};

RobotMode selectedMode = LINE_FOLLOWER;
JunctionAction selectedAction = ACTION_STOP;

bool robotRunning = false;

// -------------------- SENSOR VARIABLES --------------------

int sensorValues[5];
bool sensorBlack[5];

byte sensorMask = 0;
int linePosition = 0;

// Sensor weights: left is negative, right is positive.
const int sensorWeights[5] = {-2000, -1000, 0, 1000, 2000};

// -------------------- BUTTON HANDLING --------------------

bool buttonPressed(byte pin) {
  if (digitalRead(pin) != LOW) {
    return false;
  }

  delay(25);  // Debounce

  if (digitalRead(pin) != LOW) {
    return false;
  }

  while (digitalRead(pin) == LOW) {
    delay(1);
  }

  delay(25);
  return true;
}

// -------------------- LED STATUS --------------------

void setStoppedLEDs() {
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
}

void setRunningLEDs() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
}

// -------------------- MOTOR CONTROL --------------------

// Positive speed = forward for each motor.
// If a motor runs backward, reverse its direction here.

void setMotorA(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  analogWrite(PWMA, abs(speed));
}

void setMotorB(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  analogWrite(PWMB, abs(speed));
}

void drive(int leftSpeed, int rightSpeed) {
  setMotorA(leftSpeed);
  setMotorB(rightSpeed);
}

void stopMotors() {
  drive(0, 0);
}

void forwardFor(unsigned long duration, int speed) {
  drive(speed, speed);
  delay(duration);
  stopMotors();
}

void backwardFor(unsigned long duration, int speed) {
  drive(-speed, -speed);
  delay(duration);
  stopMotors();
}

void rotateLeft() {
  drive(-TURN_SPEED, TURN_SPEED);
}

void rotateRight() {
  drive(TURN_SPEED, -TURN_SPEED);
}

// -------------------- SENSOR READING --------------------

void readSensors() {
  sensorMask = 0;

  for (byte i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);

    if (BLACK_IS_HIGH) {
      sensorBlack[i] =
        (sensorValues[i] > SENSOR_THRESHOLD);
    } else {
      sensorBlack[i] =
        (sensorValues[i] < SENSOR_THRESHOLD);
    }
  }

  for (byte i = 0; i < 5; i++) {
    byte index = REVERSE_SENSOR_ORDER ? 4 - i : i;

    if (sensorBlack[index]) {
      sensorMask |= (1 << i);
    }
  }

  long weightedSum = 0;
  int activeCount = 0;

  for (byte i = 0; i < 5; i++) {
    if (sensorMask & (1 << i)) {
      weightedSum += sensorWeights[i];
      activeCount++;
    }
  }

  if (activeCount > 0) {
    linePosition = weightedSum / activeCount;
  }
}

// -------------------- SENSOR DIAGNOSTICS --------------------

void printSensorData() {
  readSensors();

  Serial.print("Analog: ");

  for (byte i = 0; i < 5; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }

  Serial.print(" | Binary: ");

  for (int i = 4; i >= 0; i--) {
    Serial.print((sensorMask >> i) & 1);
  }

  Serial.print(" | Position: ");
  Serial.println(linePosition);
}

// -------------------- BINARY CASE DETECTION --------------------

int countActiveSensors(byte mask) {
  int count = 0;

  for (byte i = 0; i < 5; i++) {
    if (mask & (1 << i)) {
      count++;
    }
  }

  return count;
}

bool centerOnBlack() {
  readSensors();
  return (sensorMask & 0b00100) != 0;
}

bool allSensorsWhite() {
  readSensors();
  return sensorMask == 0;
}

bool isDeadEnd() {
  return allSensorsWhite();
}

// Pattern examples, assuming S0 is leftmost:
//
// 00100 = center line
// 01100 = line toward left
// 00110 = line toward right
// 11100 = strong left branch / left junction
// 00111 = strong right branch / right junction
// 11111 = wide crossing
//
// The masks are illustrative. Real readings depend on
// sensor spacing and the width of your track.

bool isLeftTurnPattern() {
  readSensors();
  return (sensorMask & 0b00011) == 0b00011;
}

bool isRightTurnPattern() {
  readSensors();
  return (sensorMask & 0b11000) == 0b11000;
}

bool isTJunction() {
  readSensors();

  bool leftSide  = (sensorMask & 0b00011) != 0;
  bool rightSide = (sensorMask & 0b11000) != 0;
  bool center    = (sensorMask & 0b00100) != 0;

  return leftSide && rightSide && center;
}

bool isCrossJunction() {
  readSensors();

  return countActiveSensors(sensorMask) >= 4;
}

bool isJunction() {
  readSensors();

  // Broad intersections and T-junctions.
  if (isCrossJunction() || isTJunction()) {
    return true;
  }

  // A strong branch on either side.
  // Tune these patterns to suit your physical sensor array.
  if (isLeftTurnPattern() || isRightTurnPattern()) {
    return true;
  }

  return false;
}

// -------------------- PID LINE FOLLOWER --------------------

void resetPID() {
  previousError = 0;
  integral = 0;
}

void followLinePID() {
  readSensors();

  // Junction handling belongs to the mode controller,
  // not to the PID controller.
  if (isJunction()) {
    stopMotors();
    return;
  }

  if (sensorMask == 0) {
    stopMotors();
    return;
  }

  float error = (float)linePosition / 1000.0;

  integral += error;
  integral = constrain(integral, -20.0, 20.0);

  float derivative = error - previousError;

  float correction =
    Kp * error +
    Ki * integral +
    Kd * derivative;

  previousError = error;

  int leftSpeed =
    constrain((int)(BASE_SPEED + correction),
              -MAX_SPEED, MAX_SPEED);

  int rightSpeed =
    constrain((int)(BASE_SPEED - correction),
              -MAX_SPEED, MAX_SPEED);

  drive(leftSpeed, rightSpeed);
}

// -------------------- CALIBRATION SEQUENCE --------------------

void calibrateRobot() {
  Serial.println();
  Serial.println("Calibration begins in 2 seconds...");

  setStoppedLEDs();
  stopMotors();

  delay(2000);

  Serial.println("Moving forward for 500 ms");
  forwardFor(500, CAL_SPEED);

  delay(100);

  Serial.println("Moving backward for 500 ms");
  backwardFor(500, CAL_SPEED);

  stopMotors();
  resetPID();

  Serial.println("Calibration movement complete.");
}

// -------------------- MENU DISPLAY --------------------

void printMainMenu() {
  Serial.println();
  Serial.println("====== ROBOT MENU ======");

  for (byte i = 0; i < 3; i++) {
    Serial.print(i == (byte)selectedMode ? "> " : "  ");
    Serial.println(modeNames[i]);
  }

  Serial.println("BNT1 = NEXT");
  Serial.println("BNT2 = SELECT");
}

void printActionMenu() {
  Serial.println();
  Serial.println("=== LOGIC TESTER ACTION ===");

  for (byte i = 0; i < 5; i++) {
    Serial.print(i == (byte)selectedAction ? "> " : "  ");
    Serial.println(actionNames[i]);
  }

  Serial.println("BNT1 = NEXT ACTION");
  Serial.println("BNT2 = SELECT ACTION");
}

void waitForStart() {
  stopMotors();
  setStoppedLEDs();

  Serial.println();
  Serial.println("Press BNT2 to start.");

  if (selectedMode == LINE_FOLLOWER) {
    Serial.println("Ready: Line Follower");
  } else if (selectedMode == MAZE_SOLVER) {
    Serial.println("Ready: Explore Maze");
  } else {
    Serial.print("Ready: Logic Tester - ");
    Serial.println(actionNames[selectedAction]);
    Serial.println("Chosen action runs when a junction is detected.");
  }

  while (true) {
    if (buttonPressed(BNT2)) {
      robotRunning = true;
      setRunningLEDs();
      resetPID();

      Serial.println("Robot started!");
      return;
    }
  }
}

// -------------------- MENU SELECTION --------------------

void selectLogicTesterAction() {
  selectedAction = ACTION_STOP;

  while (true) {
    printActionMenu();

    if (buttonPressed(BNT1)) {
      selectedAction = (JunctionAction)(
        ((int)selectedAction + 1) % 5
      );

      printActionMenu();
    }

    if (buttonPressed(BNT2)) {
      Serial.print("Selected action: ");
      Serial.println(actionNames[selectedAction]);
      return;
    }
  }
}

void configureSelectedMode() {
  if (selectedMode == LOGIC_TESTER) {
    selectLogicTesterAction();
  }

  calibrateRobot();
  waitForStart();
}

// -------------------- JUNCTION APPROACH --------------------

// Every detected junction or branch must be approached by
// inching forward before the selected turn/action.
//
// This routine is deliberately short. Tune INCH_TIME_MS
// for your sensor-to-wheel distance and robot speed.

void inchForward() {
  forwardFor(INCH_TIME_MS, INCH_SPEED);
}

// -------------------- TURN UNTIL CENTER FINDS LINE --------------------

bool turnUntilCenter(bool leftTurn) {
  unsigned long start = millis();
  bool centerHasLeftLine = false;

  while (millis() - start < TURN_TIMEOUT_MS) {
    readSensors();

    if (!centerOnBlack()) {
      centerHasLeftLine = true;
    }

    // Do not stop immediately just because the center sensor
    // still sees the original line.
    if (centerHasLeftLine && centerOnBlack()) {
      stopMotors();
      resetPID();
      delay(60);
      return true;
    }

    if (leftTurn) {
      rotateLeft();
    } else {
      rotateRight();
    }
  }

  stopMotors();

  Serial.println("WARNING: Turn timeout; line not found.");
  return false;
}

void turnLeftToLine() {
  turnUntilCenter(true);
}

void turnRightToLine() {
  turnUntilCenter(false);
}

// -------------------- MAZE SOLVER --------------------

// Left-hand rule:
// 1. Detect junction.
// 2. Stop PID.
// 3. Inch forward.
// 4. Prefer left, then straight, then right.
// 5. If no line exists, treat as a dead end and turn around.
//
// This basic version assumes the robot is following a
// continuous black line on a light background.

void handleMazeJunction() {
  stopMotors();
  Serial.println("Junction detected - PID suspended.");

  inchForward();
  readSensors();

  // Prefer a left branch.
  if (isLeftTurnPattern()) {
    Serial.println("Maze: turning LEFT.");
    turnLeftToLine();
    return;
  }

  // If a left branch was not detected, continue straight
  // when the center sensor still sees the line.
  if (centerOnBlack()) {
    Serial.println("Maze: continuing STRAIGHT.");
    resetPID();
    return;
  }

  // Otherwise try the right branch.
  if (isRightTurnPattern()) {
    Serial.println("Maze: turning RIGHT.");
    turnRightToLine();
    return;
  }

  // No usable branch found: dead end / lost line.
  Serial.println("Maze: dead end - turning around.");
  turnRightToLine();
  turnRightToLine();
}

// -------------------- DEAD-END / LINE-LOSS HANDLING --------------------

void handleLostLine() {
  stopMotors();

  Serial.println("Line lost / possible dead end.");

  if (selectedMode == MAZE_SOLVER) {
    // A dead end is a junction case too: inch first.
    inchForward();

    // U-turn using two right turns.
    turnRightToLine();
    turnRightToLine();
  } else {
    Serial.println("Stopped. Inspect sensor readings.");
  }
}

// -------------------- LOGIC TESTER --------------------

void executeTesterAction() {
  stopMotors();

  Serial.print("Tester action: ");
  Serial.println(actionNames[selectedAction]);

  switch (selectedAction) {
    case ACTION_STOP:
      stopMotors();
      break;

    case ACTION_LEFT:
      turnLeftToLine();
      break;

    case ACTION_RIGHT:
      turnRightToLine();
      break;

    case ACTION_INCH:
      inchForward();
      break;

    case ACTION_PID:
      // The caller keeps PID active after the junction.
      resetPID();
      break;
  }
}

void handleLogicTesterJunction() {
  stopMotors();
  Serial.println("TESTER: Junction detected.");

  // Mandatory inch forward for all junction types.
  inchForward();

  executeTesterAction();

  if (selectedAction != ACTION_PID) {
    stopMotors();

    // Avoid immediately triggering on the same wide crossing.
    delay(150);
  }
}

// -------------------- RUNNING MODES --------------------

void runLineFollower() {
  readSensors();

  if (isJunction()) {
    stopMotors();
    Serial.println("Line follower: junction detected.");

    // Pause instead of allowing PID to steer across a junction.
    // Wait until the junction pattern clears.
    while (isJunction()) {
      stopMotors();
      delay(10);
    }

    resetPID();
    return;
  }

  if (sensorMask == 0) {
    handleLostLine();
    return;
  }

  followLinePID();
}

void runMazeSolver() {
  readSensors();

  if (isJunction()) {
    handleMazeJunction();
    return;
  }

  if (sensorMask == 0) {
    handleLostLine();
    return;
  }

  followLinePID();
}

void runLogicTester() {
  readSensors();

  if (isJunction()) {
    handleLogicTesterJunction();
    return;
  }

  if (sensorMask == 0) {
    stopMotors();
    return;
  }

  if (selectedAction == ACTION_PID) {
    followLinePID();
  } else {
    stopMotors();
  }
}

// -------------------- SETUP --------------------

void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(BNT1, INPUT_PULLUP);
  pinMode(BNT2, INPUT_PULLUP);

  for (byte i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  digitalWrite(STBY, HIGH);

  stopMotors();
  setStoppedLEDs();

  Serial.println();
  Serial.println("5-SENSOR ROBOT READY");
  Serial.println("Open Serial Monitor at 115200 baud.");

  // Main menu. Selection happens before calibration.
  while (true) {
    printMainMenu();

    if (buttonPressed(BNT1)) {
      selectedMode = (RobotMode)(
        ((int)selectedMode + 1) % 3
      );
      printMainMenu();
    }

    if (buttonPressed(BNT2)) {
      Serial.print("Selected mode: ");
      Serial.println(modeNames[selectedMode]);

      configureSelectedMode();
      break;
    }
  }
}

// -------------------- MAIN LOOP --------------------

void loop() {
  if (!robotRunning) {
    stopMotors();
    return;
  }

  switch (selectedMode) {
    case LINE_FOLLOWER:
      runLineFollower();
      break;

    case MAZE_SOLVER:
      runMazeSolver();
      break;

    case LOGIC_TESTER:
      runLogicTester();
      break;
  }
}
