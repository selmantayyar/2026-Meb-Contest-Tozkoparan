/* =====================================================================
   TOZKOPARAN ROBOT 2026 - v5 STATE MACHINE

   v5 changes:
   - 3 second pause BEFORE calibration (place robot, ready)
   - Single sensor read per loop (no more double-read jitter)
   - Smoothed PID (filter to reduce shaking)
   - Stronger Kp for actual turning
   - Better state transitions (require N consecutive frames)
   ===================================================================== */

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <QTRSensors.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// =====================================================================
// PINS
// =====================================================================
#define MZ80_PIN      2
#define SERVO_PIN     3
#define LASER_PIN     4
#define NEOPIXEL_PIN  5
#define R_PWM         6
#define R_DIR1        7
#define R_DIR2        8
#define L_DIR2        9
#define L_DIR1        10
#define L_PWM         11
#define DIP_SWITCH    12
#define NUM_PIXELS    8
#define NUM_QTR_SENSORS 4

// =====================================================================
// CONFIGURATION - tune these!
// =====================================================================

// PID - higher Kp for stronger turns, more Kd to prevent oscillation
float Kp = 0.08;        // proportional - if shakes, lower this
float Kd = 3.0;         // derivative - higher = less shaking

// Speeds
int BASE_SPEED  = 100;
int MAX_SPEED   = 180;
int TURN_SPEED  = 100;
int CALIB_SPEED = 90;

// Pre-calibration pause (place robot, get ready)
const int PRE_CALIB_PAUSE_MS = 3000;

// Calibration timings
const int CALIB_45_MS = 350;
const int CALIB_90_MS = 700;

// Turn 90 after shot
const int TURN_90_MS = 550;

// Servo
int SERVO_LOADED  = 180;
int SERVO_RELEASE = 90;

// Color thresholds
int TURQUOISE_R_MAX = 90;
int TURQUOISE_G_MIN = 95;
int TURQUOISE_B_MIN = 90;
int GREEN_R_MAX = 100;
int GREEN_G_MIN = 110;
int GREEN_B_MAX = 85;

// Background detection (avg of 4 sensor values)
// HIGH avg = mostly black floor = white line normal
// LOW avg = mostly white floor = black line (Dalgasi)
const int BG_WHITE_THRESHOLD = 350;  // below = on white area
const int BG_BLACK_THRESHOLD = 650;  // above = on black area

// Number of consecutive frames to confirm state change
const int STATE_CHANGE_FRAMES = 8;

// =====================================================================
// OBJECTS
// =====================================================================
QTRSensors qtra;
unsigned int sensorValues[NUM_QTR_SENSORS];
Servo arbalet;
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// =====================================================================
// STATE MACHINE
// =====================================================================
enum RobotState {
  STATE_START_LINE,
  STATE_DALGASI,
  STATE_AFTER_DALGASI,
  STATE_TURQUOISE_SHOOT,
  STATE_SEARCH_LINE,
  STATE_AFTER_SHOT_LINE,
  STATE_GREEN_BRIDGE,
  STATE_FINISH
};

RobotState currentState = STATE_START_LINE;
int pistMode = 1;
int lastError = 0;
bool tcs_ok = false;
int stateChangeCounter = 0;  // for confirming state changes

// Current sensor readings (read ONCE per loop, used everywhere)
unsigned int currentPosition = 0;
int currentAvg = 0;

#define COLOR_NONE      0
#define COLOR_TURQUOISE 1
#define COLOR_GREEN     2

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(9600);
  delay(500);

  Serial.println(F("=========================="));
  Serial.println(F("TOZKOPARAN 2026 v5"));
  Serial.println(F("=========================="));

  // Pins
  pinMode(R_PWM, OUTPUT); pinMode(R_DIR1, OUTPUT); pinMode(R_DIR2, OUTPUT);
  pinMode(L_PWM, OUTPUT); pinMode(L_DIR1, OUTPUT); pinMode(L_DIR2, OUTPUT);
  motorStop();

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, HIGH);

  pinMode(MZ80_PIN, INPUT);
  pinMode(DIP_SWITCH, INPUT_PULLUP);

  arbalet.attach(SERVO_PIN);
  arbalet.write(SERVO_LOADED);

  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A3}, NUM_QTR_SENSORS);

  pixels.begin();
  pixels.clear();
  pixels.show();

  if (tcs.begin()) {
    Serial.println(F("OK: TCS34725"));
    tcs_ok = true;
  } else {
    Serial.println(F("ERROR: TCS not found"));
  }

  if (digitalRead(DIP_SWITCH) == HIGH) {
    pistMode = 1;
    Serial.println(F("MODE: PIST-A"));
  } else {
    pistMode = 2;
    Serial.println(F("MODE: PIST-B"));
  }

  // ===== 3 SECOND PAUSE =====
  // Place robot, get ready
  Serial.println(F("Pause 3 sec - place robot on white line"));
  delay(PRE_CALIB_PAUSE_MS);

  // ===== CALIBRATION =====
  doCalibration();

  digitalWrite(LASER_PIN, LOW);
  Serial.println(F("Waiting for MZ80..."));

  while (digitalRead(MZ80_PIN) == LOW) {
    delay(10);
  }

  Serial.println(F(">>> START <<<"));
  Serial.println(F("State 1: START_LINE"));
  delay(100);
}

// =====================================================================
// CALIBRATION (turn-only - 45R + 90L + 90R + 45L)
// =====================================================================
void doCalibration() {
  Serial.println(F("CALIBRATION..."));

  Serial.println(F("Calib: 45 RIGHT"));
  unsigned long ts = millis();
  while (millis() - ts < CALIB_45_MS) {
    motorLeft(CALIB_SPEED);
    motorRight(-CALIB_SPEED);
    qtra.calibrate();
    delay(30);
  }

  Serial.println(F("Calib: 90 LEFT"));
  ts = millis();
  while (millis() - ts < CALIB_90_MS) {
    motorLeft(-CALIB_SPEED);
    motorRight(CALIB_SPEED);
    qtra.calibrate();
    delay(30);
  }

  Serial.println(F("Calib: 90 RIGHT"));
  ts = millis();
  while (millis() - ts < CALIB_90_MS) {
    motorLeft(CALIB_SPEED);
    motorRight(-CALIB_SPEED);
    qtra.calibrate();
    delay(30);
  }

  Serial.println(F("Calib: 45 LEFT"));
  ts = millis();
  while (millis() - ts < CALIB_45_MS) {
    motorLeft(-CALIB_SPEED);
    motorRight(CALIB_SPEED);
    qtra.calibrate();
    delay(30);
  }

  motorStop();
  Serial.println(F("Calibration done"));

  // Diagnostic
  qtra.readCalibrated(sensorValues);
  Serial.print(F("Calibrated: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(F(" "));
  }
  Serial.println();
  Serial.print(F("Avg: "));
  Serial.println((sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3]) / 4);

  delay(500);
}

// =====================================================================
// MAIN LOOP - reads sensors ONCE then dispatches
// =====================================================================
void loop() {
  // Read sensors ONCE per loop
  // Use readLineWhite by default - we'll read black version when needed

  switch (currentState) {
    case STATE_START_LINE:
      doStartLine();
      break;
    case STATE_DALGASI:
      doDalgasi();
      break;
    case STATE_AFTER_DALGASI:
      doAfterDalgasi();
      break;
    case STATE_TURQUOISE_SHOOT:
      doTurquoiseShoot();
      break;
    case STATE_SEARCH_LINE:
      doSearchLine();
      break;
    case STATE_AFTER_SHOT_LINE:
      doAfterShotLine();
      break;
    case STATE_GREEN_BRIDGE:
      doGreenBridge();
      break;
    case STATE_FINISH:
      doFinish();
      break;
  }
}

// =====================================================================
// STATE 1: START_LINE - white line on black floor
// =====================================================================
void doStartLine() {
  // Read once and use for both PID and state check
  unsigned int position = qtra.readLineWhite(sensorValues);
  int avg = (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3]) / 4;

  // PID follow white line
  pidFollow(position);

  // Periodic diagnostic
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    lastDebug = millis();
    Serial.print(F("S1 pos="));
    Serial.print(position);
    Serial.print(F(" avg="));
    Serial.print(avg);
    Serial.print(F(" sensors: "));
    for (int i = 0; i < 4; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(F(" "));
    }
    Serial.println();
  }

  // State transition: avg drops below white threshold = on Dalgasi
  if (avg < BG_WHITE_THRESHOLD) {
    stateChangeCounter++;
    if (stateChangeCounter >= STATE_CHANGE_FRAMES) {
      Serial.println(F("State 2: DALGASI"));
      currentState = STATE_DALGASI;
      stateChangeCounter = 0;
    }
  } else {
    stateChangeCounter = 0;
  }
}

// =====================================================================
// STATE 2: DALGASI - black line on white floor
// =====================================================================
void doDalgasi() {
  // Read once - use for PID AND state check
  unsigned int position = qtra.readLineBlack(sensorValues);
  int avg = (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3]) / 4;

  // PID follow black line
  pidFollow(position);

  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    lastDebug = millis();
    Serial.print(F("S2 pos="));
    Serial.print(position);
    Serial.print(F(" avg="));
    Serial.println(avg);
  }

  // State transition: avg rises above black threshold = exited Dalgasi
  if (avg > BG_BLACK_THRESHOLD) {
    stateChangeCounter++;
    if (stateChangeCounter >= STATE_CHANGE_FRAMES) {
      Serial.println(F("State 3: AFTER_DALGASI"));
      currentState = STATE_AFTER_DALGASI;
      stateChangeCounter = 0;
    }
  } else {
    stateChangeCounter = 0;
  }
}

// =====================================================================
// STATE 3: AFTER_DALGASI - white line again, until turquoise
// =====================================================================
void doAfterDalgasi() {
  unsigned int position = qtra.readLineWhite(sensorValues);
  pidFollow(position);

  if (tcs_ok && readColor() == COLOR_TURQUOISE) {
    Serial.println(F("State 4: TURQUOISE"));
    currentState = STATE_TURQUOISE_SHOOT;
  }
}

// =====================================================================
// STATE 4: TURQUOISE_SHOOT
// =====================================================================
void doTurquoiseShoot() {
  Serial.println(F(">>> TURQUOISE - SHOOTING <<<"));
  setAllPixels(0, 0, 200);

  motorLeft(80);
  motorRight(80);
  unsigned long ts = millis();
  while (readColor() == COLOR_TURQUOISE && millis() - ts < 2000) {
    delay(20);
  }
  motorStop();
  delay(1000);

  digitalWrite(LASER_PIN, HIGH);
  delay(300);
  Serial.println(F("SHOOT!"));
  arbalet.write(SERVO_RELEASE);
  delay(1000);
  digitalWrite(LASER_PIN, LOW);
  arbalet.write(SERVO_LOADED);
  delay(300);

  Serial.println(F("Backup"));
  motorLeft(-100);
  motorRight(-100);
  delay(400);
  motorStop();
  delay(200);

  Serial.println(F("Turn 90 RIGHT"));
  motorLeft(-TURN_SPEED);
  motorRight(TURN_SPEED);
  delay(TURN_90_MS);
  motorStop();
  delay(200);

  if (pistMode == 2) {
    Serial.println(F("Pist-B: 4 more turns"));
    for (int i = 0; i < 4; i++) {
      motorLeft(BASE_SPEED);
      motorRight(BASE_SPEED);
      delay(400);
      motorStop();
      delay(300);
      motorLeft(-TURN_SPEED);
      motorRight(TURN_SPEED);
      delay(TURN_90_MS);
      motorStop();
      delay(300);
    }
  }

  setAllPixels(0, 0, 0);
  Serial.println(F("State 5: SEARCH_LINE"));
  currentState = STATE_SEARCH_LINE;
}

// =====================================================================
// STATE 5: SEARCH_LINE - drive forward looking for line
// =====================================================================
void doSearchLine() {
  motorLeft(80);
  motorRight(80);

  qtra.readLineWhite(sensorValues);
  if (sensorValues[1] > 500 || sensorValues[2] > 500) {
    Serial.println(F("Line found"));
    Serial.println(F("State 6: AFTER_SHOT_LINE"));
    currentState = STATE_AFTER_SHOT_LINE;
    return;
  }

  static unsigned long searchStart = 0;
  if (searchStart == 0) searchStart = millis();
  if (millis() - searchStart > 3000) {
    Serial.println(F("Search timeout"));
    Serial.println(F("State 6: AFTER_SHOT_LINE"));
    currentState = STATE_AFTER_SHOT_LINE;
  }

  delay(20);
}

// =====================================================================
// STATE 6: AFTER_SHOT_LINE - white line until green
// =====================================================================
void doAfterShotLine() {
  unsigned int position = qtra.readLineWhite(sensorValues);
  pidFollow(position);

  if (tcs_ok && readColor() == COLOR_GREEN) {
    Serial.println(F("State 7: GREEN_BRIDGE"));
    currentState = STATE_GREEN_BRIDGE;
  }
}

// =====================================================================
// STATE 7: GREEN_BRIDGE - soft PID
// =====================================================================
void doGreenBridge() {
  static bool ledOn = false;
  if (!ledOn) {
    setAllPixels(0, 200, 0);
    ledOn = true;
  }

  unsigned int position = qtra.readLineWhite(sensorValues);
  int error = position - 1500;
  // softer: half Kp, half Kd
  int correction = (Kp * 0.5) * error + (Kd * 0.5) * (error - lastError);
  lastError = error;

  int speed = BASE_SPEED - 20;
  int leftSpeed = constrain(speed + correction, 50, MAX_SPEED);
  int rightSpeed = constrain(speed - correction, 50, MAX_SPEED);
  motorLeft(leftSpeed);
  motorRight(rightSpeed);

  static int greenLost = 0;
  if (readColor() != COLOR_GREEN) {
    greenLost++;
    if (greenLost >= 5) {
      setAllPixels(0, 0, 0);
      Serial.println(F("State 8: FINISH"));
      currentState = STATE_FINISH;
      greenLost = 0;
      ledOn = false;
    }
  } else {
    greenLost = 0;
  }

  delay(20);
}

// =====================================================================
// STATE 8: FINISH
// =====================================================================
void doFinish() {
  Serial.println(F(">>> FINISH <<<"));

  unsigned long ts = millis();
  while (millis() - ts < 2000) {
    unsigned int position = qtra.readLineWhite(sensorValues);
    pidFollow(position);
    delay(5);
  }

  motorStop();
  Serial.println(F("RACE COMPLETE!"));

  rainbowAnimation(5000);
  setAllPixels(0, 0, 0);

  while (true) {
    motorStop();
    delay(100);
  }
}

// =====================================================================
// PID FOLLOW - takes position from already-read sensors
// =====================================================================
void pidFollow(unsigned int position) {
  int error = position - 1500;
  int correction = Kp * error + Kd * (error - lastError);
  lastError = error;

  int leftSpeed = constrain(BASE_SPEED + correction, -100, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - correction, -100, MAX_SPEED);
  motorLeft(leftSpeed);
  motorRight(rightSpeed);
}

// =====================================================================
// COLOR
// =====================================================================
int readColor() {
  float r, g, b;
  tcs.setInterrupt(false);
  tcs.getRGB(&r, &g, &b);
  tcs.setInterrupt(true);

  if (r < TURQUOISE_R_MAX && g > TURQUOISE_G_MIN && b > TURQUOISE_B_MIN) {
    return COLOR_TURQUOISE;
  }
  if (r < GREEN_R_MAX && g > GREEN_G_MIN && b < GREEN_B_MAX) {
    return COLOR_GREEN;
  }
  return COLOR_NONE;
}

// =====================================================================
// MOTORS
// =====================================================================
void motorLeft(int speed) {
  if (speed >= 0) {
    digitalWrite(L_DIR1, HIGH);
    digitalWrite(L_DIR2, LOW);
    analogWrite(L_PWM, speed);
  } else {
    digitalWrite(L_DIR1, LOW);
    digitalWrite(L_DIR2, HIGH);
    analogWrite(L_PWM, abs(speed));
  }
}

void motorRight(int speed) {
  if (speed >= 0) {
    digitalWrite(R_DIR1, HIGH);
    digitalWrite(R_DIR2, LOW);
    analogWrite(R_PWM, speed);
  } else {
    digitalWrite(R_DIR1, LOW);
    digitalWrite(R_DIR2, HIGH);
    analogWrite(R_PWM, abs(speed));
  }
}

void motorStop() {
  digitalWrite(L_DIR1, LOW);
  digitalWrite(L_DIR2, LOW);
  analogWrite(L_PWM, 0);
  digitalWrite(R_DIR1, LOW);
  digitalWrite(R_DIR2, LOW);
  analogWrite(R_PWM, 0);
}

// =====================================================================
// NEOPIXEL
// =====================================================================
void setAllPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

void rainbowAnimation(unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    for (int j = 0; j < 256; j++) {
      for (int i = 0; i < NUM_PIXELS; i++) {
        pixels.setPixelColor(i, wheel((i * 256 / NUM_PIXELS + j) & 255));
      }
      pixels.show();
      delay(20);
      if (millis() - start >= duration) return;
    }
  }
}

uint32_t wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
