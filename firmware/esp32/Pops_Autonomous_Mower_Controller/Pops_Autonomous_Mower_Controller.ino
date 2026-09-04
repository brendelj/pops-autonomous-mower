/*
  Pops' Autonomous Mower Controller - ESP32
  Version 0.1

  Hardware architecture:
    - ESP32 main propulsion controller
    - FlySky receiver: throttle, steering, mode
    - 4 x BTS7960 motor drivers
    - 4 x 755 brushed DC motors
    - BU04 / DW3000 UWB mower ranging module (SPI integration hook)
    - Four fixed BU04/DW3000 anchors in yard

  SAFETY:
    - RC mode and Autonomous mode are mutually exclusive.
    - RC signal timeout stops all propulsion.
    - Autonomous command timeout stops all propulsion.
    - Mode transition always stops motors first.
    - Add a physical E-STOP that removes MOTOR POWER independently
      of this software.

  IMPORTANT:
    Pin assignments below are a proposed starting map.
    Verify them against your exact ESP32 board before wiring.
*/

#include <Arduino.h>
#include <SPI.h>

// ================================================================
// USER CONFIGURATION
// ================================================================

// ---------- FlySky receiver inputs ----------
static const int PIN_RC_THROTTLE = 32;
static const int PIN_RC_STEERING = 33;
static const int PIN_RC_MODE     = 25;

// Measured FlySky pulse widths.
// CHANGE THESE after running calibration.
int RC_THROTTLE_REVERSE = 1000;
int RC_THROTTLE_NEUTRAL = 1500;
int RC_THROTTLE_FORWARD = 2000;

int RC_STEERING_LEFT    = 1000;
int RC_STEERING_CENTER  = 1500;
int RC_STEERING_RIGHT   = 2000;

// Typical 3-position / 2-position mode channel threshold.
// Below threshold = RC/manual; above threshold = autonomous.
int RC_MODE_THRESHOLD_US = 1600;

// Input deadband around center.
int RC_DEADBAND_US = 35;

// Fail-safe: no valid RC frame for this long -> STOP.
static const uint32_t RC_TIMEOUT_MS = 300;

// ---------- Wheel scaling ----------
// Front wheels are slightly smaller than rear.
// A smaller wheel must rotate faster for the same ground speed.
// Tune these experimentally.
float SCALE_FRONT_LEFT  = 1.08f;
float SCALE_FRONT_RIGHT = 1.08f;
float SCALE_REAR_LEFT   = 1.00f;
float SCALE_REAR_RIGHT  = 1.00f;

// Per-wheel direction correction.
// Change to -1 if a motor runs backward relative to the others.
int DIR_FRONT_LEFT  =  1;
int DIR_FRONT_RIGHT =  1;
int DIR_REAR_LEFT   =  1;
int DIR_REAR_RIGHT  =  1;

// Overall maximum propulsion command percentage.
int MAX_DRIVE_PERCENT = 85;

// Minimum PWM needed to overcome motor/gearbox static friction.
// Set to 0 while bench-testing.
// Tune later if needed.
int MIN_MOVING_PWM = 0;

// ---------- Autonomous fail-safe ----------
static const uint32_t AUTO_COMMAND_TIMEOUT_MS = 500;

// ================================================================
// MOTOR PIN MAP
//
// Each BTS7960 has:
//   RPWM = forward PWM
//   LPWM = reverse PWM
//   R_EN and L_EN can normally be tied HIGH or controlled.
//
// This version controls the PWM lines and gives each driver one
// shared software ENABLE signal. If your board wiring uses separate
// R_EN/L_EN GPIOs, we can expand this.
// ================================================================

// Front Left
static const int PIN_FL_RPWM = 13;
static const int PIN_FL_LPWM = 14;
static const int PIN_FL_EN   = 27;

// Front Right
static const int PIN_FR_RPWM = 26;
static const int PIN_FR_LPWM = 12;
static const int PIN_FR_EN   = 4;

// Rear Left
static const int PIN_RL_RPWM = 18;
static const int PIN_RL_LPWM = 19;
static const int PIN_RL_EN   = 23;

// Rear Right
static const int PIN_RR_RPWM = 5;
static const int PIN_RR_LPWM = 17;
static const int PIN_RR_EN   = 16;

// ================================================================
// BU04 / DW3000 SPI PINS
//
// IMPORTANT:
// These pins are reserved for the UWB interface. They may need to be
// remapped depending on the final motor/ESP32 wiring.
//
// BU04 exposes SPI_CLK, SPI_MOSI, SPI_MISO, SPI_CSN, IRQ,
// DW_RSTN and WAKEUP.
//
// The actual two-way-ranging protocol/library will be plugged into
// the UWB functions below.
// ================================================================

static const int PIN_UWB_SCK    = 22;
static const int PIN_UWB_MOSI   = 21;
static const int PIN_UWB_MISO   = 34;  // input-only pin is OK for MISO
static const int PIN_UWB_CS     = 15;
static const int PIN_UWB_IRQ    = 35;  // input-only pin is OK for IRQ
static const int PIN_UWB_RST    = 2;
static const int PIN_UWB_WAKE   = 0;

// ================================================================
// TYPES / STATE
// ================================================================

enum DriveMode {
  MODE_STOPPED,
  MODE_RC,
  MODE_AUTONOMOUS
};

DriveMode currentMode = MODE_STOPPED;

struct Motor {
  int rpwm;
  int lpwm;
  int enable;
  float scale;
  int direction;
};

Motor motorFL = {PIN_FL_RPWM, PIN_FL_LPWM, PIN_FL_EN, SCALE_FRONT_LEFT,  DIR_FRONT_LEFT};
Motor motorFR = {PIN_FR_RPWM, PIN_FR_LPWM, PIN_FR_EN, SCALE_FRONT_RIGHT, DIR_FRONT_RIGHT};
Motor motorRL = {PIN_RL_RPWM, PIN_RL_LPWM, PIN_RL_EN, SCALE_REAR_LEFT,   DIR_REAR_LEFT};
Motor motorRR = {PIN_RR_RPWM, PIN_RR_LPWM, PIN_RR_EN, SCALE_REAR_RIGHT,  DIR_REAR_RIGHT};

struct RCState {
  int throttleUs;
  int steeringUs;
  int modeUs;
  bool valid;
  uint32_t lastGoodMs;
};

RCState rc = {1500, 1500, 1000, false, 0};

struct AutoCommand {
  float forward;     // -1.0 reverse .. +1.0 forward
  float turn;        // -1.0 left    .. +1.0 right
  bool valid;
  uint32_t timestampMs;
};

AutoCommand autoCommand = {0.0f, 0.0f, false, 0};

// UWB ranges, meters
struct UWBRanges {
  float a1;
  float a2;
  float a3;
  float a4;
  bool valid;
  uint32_t timestampMs;
};

UWBRanges uwbRanges = {0, 0, 0, 0, false, 0};

// ================================================================
// UTILITY
// ================================================================

float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Maps receiver pulse to -1.0 .. +1.0 with a real center/deadband.
float mapCenteredPulse(int pulse,
                       int lowValue,
                       int centerValue,
                       int highValue,
                       int deadband) {
  if (pulse >= centerValue - deadband &&
      pulse <= centerValue + deadband) {
    return 0.0f;
  }

  if (pulse < centerValue) {
    int usableCenter = centerValue - deadband;
    float value = (float)(pulse - usableCenter) /
                  (float)(usableCenter - lowValue);
    return clampFloat(value, -1.0f, 0.0f);
  }

  int usableCenter = centerValue + deadband;
  float value = (float)(pulse - usableCenter) /
                (float)(highValue - usableCenter);
  return clampFloat(value, 0.0f, 1.0f);
}

// ================================================================
// PWM / MOTOR CONTROL
// ================================================================
//
// Arduino-ESP32 provides analogWrite() on current cores.
// Range is 0..255 here.
//
// If your installed ESP32 core requires explicit LEDC setup, we can
// change only this section.
// ================================================================

void pwmWrite(int pin, int value) {
  analogWrite(pin, clampInt(value, 0, 255));
}

void setMotorEnabled(Motor &m, bool enabled) {
  digitalWrite(m.enable, enabled ? HIGH : LOW);
}

void stopMotor(Motor &m) {
  pwmWrite(m.rpwm, 0);
  pwmWrite(m.lpwm, 0);
}

void stopAllMotors() {
  stopMotor(motorFL);
  stopMotor(motorFR);
  stopMotor(motorRL);
  stopMotor(motorRR);
}

void disableAllMotors() {
  stopAllMotors();

  setMotorEnabled(motorFL, false);
  setMotorEnabled(motorFR, false);
  setMotorEnabled(motorRL, false);
  setMotorEnabled(motorRR, false);
}

void enableAllMotors() {
  setMotorEnabled(motorFL, true);
  setMotorEnabled(motorFR, true);
  setMotorEnabled(motorRL, true);
  setMotorEnabled(motorRR, true);
}

// command = -1.0 .. +1.0
void driveMotor(Motor &m, float command) {
  command = clampFloat(command, -1.0f, 1.0f);

  // Apply wheel-size correction and direction correction.
  command *= m.scale;
  command *= m.direction;
  command = clampFloat(command, -1.0f, 1.0f);

  int maxPwm = (255 * MAX_DRIVE_PERCENT) / 100;
  int pwm = (int)(fabs(command) * maxPwm);

  if (pwm > 0 && pwm < MIN_MOVING_PWM) {
    pwm = MIN_MOVING_PWM;
  }

  if (command > 0.0f) {
    pwmWrite(m.lpwm, 0);
    pwmWrite(m.rpwm, pwm);
  }
  else if (command < 0.0f) {
    pwmWrite(m.rpwm, 0);
    pwmWrite(m.lpwm, pwm);
  }
  else {
    stopMotor(m);
  }
}

// Differential/skid steering mixer.
//
// forward: -1 reverse .. +1 forward
// turn:    -1 left    .. +1 right
//
// This produces left/right side commands, then each wheel's scale
// factor compensates for wheel diameter differences.
void driveMixer(float forward, float turn) {
  forward = clampFloat(forward, -1.0f, 1.0f);
  turn    = clampFloat(turn,    -1.0f, 1.0f);

  float left  = forward + turn;
  float right = forward - turn;

  // Normalize instead of clipping one side independently.
  float biggest = max(fabs(left), fabs(right));
  if (biggest > 1.0f) {
    left  /= biggest;
    right /= biggest;
  }

  driveMotor(motorFL, left);
  driveMotor(motorRL, left);
  driveMotor(motorFR, right);
  driveMotor(motorRR, right);
}

// ================================================================
// RC INPUT
// ================================================================

int readRcPulse(int pin) {
  // Timeout is intentionally short so a missing signal does not
  // block the control loop for too long.
  unsigned long value = pulseIn(pin, HIGH, 25000);

  if (value < 800 || value > 2200) {
    return 0;
  }

  return (int)value;
}

void readRC() {
  int t = readRcPulse(PIN_RC_THROTTLE);
  int s = readRcPulse(PIN_RC_STEERING);
  int m = readRcPulse(PIN_RC_MODE);

  if (t != 0 && s != 0 && m != 0) {
    rc.throttleUs = t;
    rc.steeringUs = s;
    rc.modeUs = m;
    rc.valid = true;
    rc.lastGoodMs = millis();
  }
  else {
    if (millis() - rc.lastGoodMs > RC_TIMEOUT_MS) {
      rc.valid = false;
    }
  }
}

DriveMode requestedModeFromRC() {
  if (!rc.valid) {
    return MODE_STOPPED;
  }

  if (rc.modeUs >= RC_MODE_THRESHOLD_US) {
    return MODE_AUTONOMOUS;
  }

  return MODE_RC;
}

void processRCDrive() {
  if (!rc.valid) {
    stopAllMotors();
    return;
  }

  float throttle = mapCenteredPulse(
      rc.throttleUs,
      RC_THROTTLE_REVERSE,
      RC_THROTTLE_NEUTRAL,
      RC_THROTTLE_FORWARD,
      RC_DEADBAND_US);

  float steering = mapCenteredPulse(
      rc.steeringUs,
      RC_STEERING_LEFT,
      RC_STEERING_CENTER,
      RC_STEERING_RIGHT,
      RC_DEADBAND_US);

  driveMixer(throttle, steering);
}

// ================================================================
// AUTONOMOUS CONTROL INTERFACE
// ================================================================
//
// The navigation planner eventually calls this function.
// It deliberately does NOT drive the motors immediately.
// The main safety loop decides whether autonomous control is allowed.
// ================================================================

void setAutonomousDrive(float forward, float turn) {
  autoCommand.forward = clampFloat(forward, -1.0f, 1.0f);
  autoCommand.turn = clampFloat(turn, -1.0f, 1.0f);
  autoCommand.valid = true;
  autoCommand.timestampMs = millis();
}

void clearAutonomousDrive() {
  autoCommand.forward = 0.0f;
  autoCommand.turn = 0.0f;
  autoCommand.valid = false;
}

void processAutonomousDrive() {
  if (!autoCommand.valid) {
    stopAllMotors();
    return;
  }

  if (millis() - autoCommand.timestampMs > AUTO_COMMAND_TIMEOUT_MS) {
    clearAutonomousDrive();
    stopAllMotors();
    return;
  }

  driveMixer(autoCommand.forward, autoCommand.turn);
}

// ================================================================
// BU04 / DW3000 UWB INTERFACE
// ================================================================
//
// Ai-Thinker BU04 exposes DW3000 SPI signals.
// This section reserves and initializes the bus.
//
// Actual ranging requires the BU04/DW3000 firmware/library and anchor
// protocol. That will populate a1..a4 with distances in meters.
// ================================================================

SPIClass uwbSPI(VSPI);

void setupUWB() {
  pinMode(PIN_UWB_CS, OUTPUT);
  digitalWrite(PIN_UWB_CS, HIGH);

  pinMode(PIN_UWB_RST, OUTPUT);
  pinMode(PIN_UWB_WAKE, OUTPUT);
  pinMode(PIN_UWB_IRQ, INPUT);

  digitalWrite(PIN_UWB_WAKE, LOW);

  // BU04/DW3000 reset pulse.
  // RSTN is active low.
  digitalWrite(PIN_UWB_RST, LOW);
  delay(5);
  digitalWrite(PIN_UWB_RST, HIGH);
  delay(10);

  uwbSPI.begin(PIN_UWB_SCK,
               PIN_UWB_MISO,
               PIN_UWB_MOSI,
               PIN_UWB_CS);

  // TODO:
  // Initialize DW3000/BU04 ranging library here.
  // Configure this unit as the mower/tag.
  // Configure anchor IDs A1, A2, A3, A4.
}

bool updateUWBRanges() {
  // TODO:
  // Replace with real two-way-ranging calls.
  //
  // Example desired result:
  // uwbRanges.a1 = rangeToAnchor(1);
  // uwbRanges.a2 = rangeToAnchor(2);
  // uwbRanges.a3 = rangeToAnchor(3);
  // uwbRanges.a4 = rangeToAnchor(4);
  // uwbRanges.valid = true;
  // uwbRanges.timestampMs = millis();
  //
  // return true;

  return false;
}

// ================================================================
// POSITION SOLVER HOOK
// ================================================================
//
// Once anchor coordinates are measured and entered, the four UWB
// ranges can be trilaterated into mower X/Y coordinates.
//
// We will add:
//   Anchor A1 = (x1,y1)
//   Anchor A2 = (x2,y2)
//   Anchor A3 = (x3,y3)
//   Anchor A4 = (x4,y4)
//
// and:
//   calculateMowerPosition()
//
// after the BU04 ranging exchange is talking reliably.
// ================================================================

// ================================================================
// MODE / SAFETY
// ================================================================

const char *modeName(DriveMode mode) {
  switch (mode) {
    case MODE_RC:         return "RC";
    case MODE_AUTONOMOUS: return "AUTONOMOUS";
    default:              return "STOPPED";
  }
}

void changeMode(DriveMode newMode) {
  if (newMode == currentMode) {
    return;
  }

  // Mandatory neutral period on EVERY mode transition.
  disableAllMotors();
  delay(100);

  currentMode = newMode;

  if (currentMode != MODE_STOPPED) {
    enableAllMotors();
  }

  if (currentMode != MODE_AUTONOMOUS) {
    clearAutonomousDrive();
  }

  Serial.print("MODE -> ");
  Serial.println(modeName(currentMode));
}

// ================================================================
// SERIAL TEST COMMANDS
//
// These allow bench testing autonomous mode before navigation exists.
//
// In Serial Monitor:
//   A 0.25 0.00     = 25% forward
//   A 0.00 0.25     = turn right
//   A 0.00 -0.25    = turn left
//   A -0.20 0.00    = reverse
//   X                = stop autonomous command
//
// The RC mode switch STILL decides whether AUTO commands can move
// the mower.
// ================================================================

void processSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();

  if (command == 'A' || command == 'a') {
    float forward = Serial.parseFloat();
    float turn = Serial.parseFloat();
    setAutonomousDrive(forward, turn);

    Serial.print("AUTO CMD forward=");
    Serial.print(forward, 3);
    Serial.print(" turn=");
    Serial.println(turn, 3);
  }
  else if (command == 'X' || command == 'x') {
    clearAutonomousDrive();
    stopAllMotors();
    Serial.println("AUTO CMD STOP");
  }

  while (Serial.available()) {
    Serial.read();
  }
}

// ================================================================
// CALIBRATION DISPLAY
//
// Type C in Serial Monitor and then move:
//   throttle full reverse / neutral / full forward
//   steering full left / center / full right
//
// The live pulse numbers can be copied into USER CONFIGURATION.
// ================================================================

bool calibrationDisplay = false;

void calibrationOutput() {
  static uint32_t previous = 0;

  if (!calibrationDisplay) return;
  if (millis() - previous < 250) return;

  previous = millis();

  Serial.print("THROTTLE=");
  Serial.print(rc.throttleUs);
  Serial.print("  STEERING=");
  Serial.print(rc.steeringUs);
  Serial.print("  MODE=");
  Serial.print(rc.modeUs);
  Serial.print("  VALID=");
  Serial.println(rc.valid ? "YES" : "NO");
}

// ================================================================
// SETUP
// ================================================================

void setupMotor(Motor &m) {
  pinMode(m.rpwm, OUTPUT);
  pinMode(m.lpwm, OUTPUT);
  pinMode(m.enable, OUTPUT);

  digitalWrite(m.enable, LOW);
  pwmWrite(m.rpwm, 0);
  pwmWrite(m.lpwm, 0);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("Pops' Autonomous Mower Controller");
  Serial.println("--------------------------------");

  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_STEERING, INPUT);
  pinMode(PIN_RC_MODE, INPUT);

  setupMotor(motorFL);
  setupMotor(motorFR);
  setupMotor(motorRL);
  setupMotor(motorRR);

  disableAllMotors();

  setupUWB();

  Serial.println("Controller initialized.");
  Serial.println("Motors remain stopped until a valid RC mode signal exists.");
  Serial.println();
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop() {
  // Input gathering
  readRC();
  updateUWBRanges();

  // Bench command input
  if (Serial.available()) {
    char peeked = Serial.peek();

    if (peeked == 'C' || peeked == 'c') {
      Serial.read();
      calibrationDisplay = !calibrationDisplay;
      Serial.print("RC calibration display: ");
      Serial.println(calibrationDisplay ? "ON" : "OFF");
      while (Serial.available()) Serial.read();
    }
    else {
      processSerialCommands();
    }
  }

  calibrationOutput();

  // Hardware RC signal has authority over allowed operating mode.
  DriveMode requestedMode = requestedModeFromRC();

  if (requestedMode != currentMode) {
    changeMode(requestedMode);
  }

  // Execute exactly ONE control source.
  switch (currentMode) {
    case MODE_RC:
      processRCDrive();
      break;

    case MODE_AUTONOMOUS:
      processAutonomousDrive();
      break;

    case MODE_STOPPED:
    default:
      disableAllMotors();
      break;
  }

  delay(5);
}