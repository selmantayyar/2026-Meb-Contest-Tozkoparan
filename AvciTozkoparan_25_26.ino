/* =====================================================================
   TOZKOPARAN ROBOT 2026 - FINAL CODE
   18th International MEB Robot Competition

   Robot: Arduino Nano + TB6612FNG + 4xQTR + TCS34725 + MG90S + NeoPixel

   Logic:
   - One single code that handles BOTH tracks (Pist-A and Pist-B)
   - DIP-switch on D12: HIGH = Pist-A (1 turn), LOW = Pist-B (5 turns)
   - Calibrate the line sensors at startup
   - LEDs turn ON IMMEDIATELY when entering a colored zone
   - Finish animation at the end

   IMPORTANT: uses the NEW QTRSensors library
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

// Right motor
#define R_PWM         6
#define R_DIR1        7
#define R_DIR2        8

// Left motor
#define L_DIR2        9
#define L_DIR1        10
#define L_PWM         11

#define DIP_SWITCH    12
#define NUM_PIXELS    8
#define NUM_QTR_SENSORS 4

// =====================================================================
// CONFIGURATION — CALIBRATE these for your robot!
// =====================================================================

// PID coefficients (taken from Test_3_v2)
float Kp = 0.05;
float Kd = 2.0;

// Speeds
int BASE_SPEED = 130;
int MAX_SPEED  = 200;
int TURN_SPEED = 100;
int CALIB_SPEED = 60;

// TIMINGS (milliseconds) — CALIBRATE on the real track!
int FORWARD_INTO_TURQUOISE_MS = 800;   // How long to drive into the turquoise zone before shooting
int STABILIZE_BEFORE_SHOT_MS  = 1000;  // Pause before the shot (let the robot settle)
int SHOOT_DELAY_MS            = 1000;  // Pause after the shot
int TURN_90_MS                = 550;   // Time it takes to do a 90° turn
int FORWARD_BETWEEN_TURNS_MS  = 400;   // Drive forward between turns (Pist-B)
int PAUSE_BETWEEN_TURNS_MS    = 300;   // Pause between turns
int FINISH_DRIVE_MS           = 2000;  // Time to drive to the finish gate after the bridge

// Servo
int SERVO_LOADED  = 180;    // Cocked / loaded position
int SERVO_RELEASE = 90;     // Release / shooting position

// Color thresholds — CALIBRATED FROM REAL MEASUREMENTS!
// Measurements: TURQUOISE R=50 G=104 B=115 / GREEN R=78 G=123 B=68 / WHITE R=117 G=85 B=57

// TURQUOISE: the key clue is B is high (>100). Green has B=68, white has B=57.
int TURQUOISE_R_MAX = 90;     // Measured 50, white 117 -> threshold 90
int TURQUOISE_G_MIN = 95;     // Measured 104, white 85 -> threshold 95
int TURQUOISE_B_MIN = 90;     // Measured 115, green 68, white 57 -> threshold 90 (KEY!)

// GREEN: the key clue is G is high (>100) AND B is low (<90)
int GREEN_R_MAX = 100;        // Measured 78, white 117 -> threshold 100
int GREEN_G_MIN = 110;        // Measured 123, turquoise 104 -> threshold 110 (separates from turquoise)
int GREEN_B_MAX = 85;         // Measured 68, turquoise 115 -> threshold 85 (KEY!)

// =====================================================================
// OBJECTS
// =====================================================================
QTRSensors qtra;
unsigned int sensorValues[NUM_QTR_SENSORS];

Servo arbalet;   // "arbalet" = crossbow (the arrow-launching servo)

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// =====================================================================
// GLOBAL VARIABLES
// =====================================================================
bool shotFired = false;
int pistMode = 1;
int currentColor = 0;
int lastError = 0;

bool tcs_ok = false;

#define COLOR_NONE      0
#define COLOR_TURQUOISE 1
#define COLOR_GREEN     2

// =====================================================================
// SETUP — runs ONCE when the robot turns on
// =====================================================================
void setup() {
  Serial.begin(9600);
  delay(500);

  Serial.println(F("=========================================="));
  Serial.println(F("TOZKOPARAN ROBOT 2026"));
  Serial.println(F("=========================================="));

  // Motor pins
  pinMode(R_PWM, OUTPUT);
  pinMode(R_DIR1, OUTPUT);
  pinMode(R_DIR2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(L_DIR1, OUTPUT);
  pinMode(L_DIR2, OUTPUT);
  motorStop();

  // Laser
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, HIGH);  // Turn ON to verify the aim point

  // Start sensor (MZ80) — detects when the start gate has opened
  pinMode(MZ80_PIN, INPUT);

  // DIP-switch (chooses Pist-A vs Pist-B)
  pinMode(DIP_SWITCH, INPUT_PULLUP);

  // Servo
  arbalet.attach(SERVO_PIN);
  arbalet.write(SERVO_LOADED);

  // QTR line sensors
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A3}, NUM_QTR_SENSORS);

  // NeoPixel
  pixels.begin();
  pixels.clear();
  pixels.show();

  // Color sensor
  if (tcs.begin()) {
    Serial.println(F("OK: TCS34725"));
    tcs_ok = true;
  } else {
    Serial.println(F("ERROR: TCS34725 not found!"));
    tcs_ok = false;
    // Blink red to signal the error
    for (int i = 0; i < 5; i++) {
      setAllPixels(150, 0, 0);
      delay(300);
      setAllPixels(0, 0, 0);
      delay(300);
    }
  }

  // Read DIP — choose track
  if (digitalRead(DIP_SWITCH) == HIGH) {
    pistMode = 1;
    Serial.println(F("MODE: PIST-A (1 turn)"));
  } else {
    pistMode = 2;
    Serial.println(F("MODE: PIST-B (5 turns)"));
  }

  // ===== QTR CALIBRATION =====
  Serial.println(F("Calibrating QTR (3 seconds)..."));
  setAllPixels(50, 50, 50);  // White light during calibration

  // Calibration like Test_3 — 60 iterations of 50ms = 3 seconds total
  for (int i = 0; i < 60; i++) {
    if (i < 15) {
      motorLeft(-CALIB_SPEED);
      motorRight(CALIB_SPEED);
    } else if (i < 30) {
      motorLeft(CALIB_SPEED);
      motorRight(-CALIB_SPEED);
    } else if (i < 45) {
      motorLeft(-CALIB_SPEED);
      motorRight(CALIB_SPEED);
    } else {
      motorLeft(CALIB_SPEED);
      motorRight(-CALIB_SPEED);
    }
    qtra.calibrate();
    delay(50);
  }
  motorStop();

  Serial.println(F("Calibration complete"));

  // Ready signal — blink green twice
  for (int i = 0; i < 2; i++) {
    setAllPixels(0, 100, 0);
    delay(300);
    setAllPixels(0, 0, 0);
    delay(200);
  }

  digitalWrite(LASER_PIN, LOW);

  Serial.println(F("Waiting for start (MZ80)..."));

  // ===== WAIT FOR START =====
  // MZ80: HIGH = path is clear, LOW = obstacle (gate is still closed)
  // Robot waits until the gate opens (MZ80 goes HIGH)
  while (digitalRead(MZ80_PIN) == LOW) {
    delay(10);
  }

  Serial.println(F(">>> START! <<<"));
  delay(100);
}

// =====================================================================
// LOOP — runs OVER AND OVER once setup() finishes
// =====================================================================
void loop() {
  // ===== DETECT LINE TYPE =====
  // In the Akdeniz Dalgası section the line becomes BLACK on a WHITE background (inverted!)
  // Read calibrated QTR values
  qtra.readCalibrated(sensorValues);

  // Determine the average background color
  // If all 4 sensors show LARGE values = lots of black around -> the line is WHITE
  // If all 4 sensors show SMALL values = lots of white around -> the line is BLACK
  int avgValue = (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3]) / 4;

  // ===== PID LINE FOLLOWING =====
  unsigned int position;
  if (avgValue > 500) {
    // Lots of black around -> look for a WHITE line (normal driving)
    position = qtra.readLineWhite(sensorValues);
  } else {
    // Lots of white around -> look for a BLACK line (Akdeniz Dalgası)
    position = qtra.readLineBlack(sensorValues);
  }

  int error = position - 1500;
  int correction = Kp * error + Kd * (error - lastError);
  lastError = error;

  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;
  leftSpeed = constrain(leftSpeed, -100, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -100, MAX_SPEED);

  motorLeft(leftSpeed);
  motorRight(rightSpeed);

  waitForKey();  // <-- pauses here until you send a character

  // ===== CHECK FOR COLORED ZONES =====
  if (tcs_ok) {
    int detectedColor = readColor();

    // Turquoise zone (shooting)
    if (detectedColor == COLOR_TURQUOISE && !shotFired) {
      handleTurquoiseZone();
    }

    // Green zone (bridge) — only after the shot has been fired
    if (detectedColor == COLOR_GREEN && shotFired) {
      handleGreenZone();
    }

    // Exiting turquoise -> turn LEDs off
    if (currentColor == COLOR_TURQUOISE && detectedColor != COLOR_TURQUOISE) {
      setAllPixels(0, 0, 0);
      currentColor = COLOR_NONE;
      Serial.println(F("Exited turquoise"));
    }

    // Exiting green -> turn LEDs off, then run finish sequence
    if (currentColor == COLOR_GREEN && detectedColor != COLOR_GREEN) {
      setAllPixels(0, 0, 0);
      currentColor = COLOR_NONE;
      Serial.println(F("Exited green"));
      finishRace();
    }
  }
}

void waitForKey() {
  Serial.println(F("STEP COMPLETE — press any key to continue, or power off to stop"));
  while (Serial.available() == 0) {
    motorStop();
    delay(50);
  }
  while (Serial.available() > 0) Serial.read(); // flush
}

// =====================================================================
// TURQUOISE ZONE — drive to end of zone, shoot the arrow, turn right
// =====================================================================
void handleTurquoiseZone() {
  Serial.println(F(">>> TURQUOISE <<<"));

  // 1. Blue LED ON immediately (rules require LED on as soon as we enter)
  setAllPixels(0, 0, 200);
  currentColor = COLOR_TURQUOISE;

  // 2. Drive deeper into the zone — keep going while sensor still reads turquoise
  Serial.println(F("Driving deeper to end of zone"));
  motorLeft(80);
  motorRight(80);
  unsigned long timeoutStart = millis();
  while (readColor() == COLOR_TURQUOISE) {
    delay(20);
    // Safety: max 2 seconds — don't get stuck here forever
    if (millis() - timeoutStart > 2000) break;
  }

  // 3. STOP — we have reached the end of the turquoise zone
  motorStop();
  delay(STABILIZE_BEFORE_SHOT_MS);

  // 4. Turn the laser ON for aiming
  digitalWrite(LASER_PIN, HIGH);
  delay(300);

  // 5. SHOOT the arrow
  Serial.println(F("SHOOT!"));
  arbalet.write(SERVO_RELEASE);
  delay(SHOOT_DELAY_MS);

  // 6. Laser OFF
  digitalWrite(LASER_PIN, LOW);

  // 7. Reset servo back to loaded position
  arbalet.write(SERVO_LOADED);
  delay(300);

  // 8. Turn right — keep spinning until the line sensors find a line
  Serial.println(F("Turning right to find the line"));
  motorLeft(-TURN_SPEED);
  motorRight(TURN_SPEED);
  delay(200);  // short pause to drive off the current line

  // Spin until at least one of the middle sensors sees the white line
  timeoutStart = millis();
  while (true) {
    qtra.readLineWhite(sensorValues);
    // Line is found when at least one of the middle sensors sees white (>500)
    if (sensorValues[1] > 500 || sensorValues[2] > 500) {
      break;
    }
    if (millis() - timeoutStart > 1500) break;  // safety timeout
    delay(10);
  }
  motorStop();
  delay(200);

  // 9. If we are running Pist-B (final round) — do 4 more right turns to line
  if (pistMode == 2) {
    Serial.println(F("Pist-B: 4 more 90 degree turns"));
    for (int i = 0; i < 4; i++) {
      // Drive straight a little
      motorLeft(BASE_SPEED);
      motorRight(BASE_SPEED);
      delay(FORWARD_BETWEEN_TURNS_MS);
      motorStop();
      delay(PAUSE_BETWEEN_TURNS_MS);

      // Then spin right until the line is found again
      motorLeft(-TURN_SPEED);
      motorRight(TURN_SPEED);
      delay(200);  // get off the current line
      timeoutStart = millis();
      while (true) {
        qtra.readLineWhite(sensorValues);
        if (sensorValues[1] > 500 || sensorValues[2] > 500) break;
        if (millis() - timeoutStart > 1500) break;
        delay(10);
      }
      motorStop();
      delay(PAUSE_BETWEEN_TURNS_MS);
    }
  }

  shotFired = true;

  // 10. Continue forward to leave the zone
  motorLeft(BASE_SPEED);
  motorRight(BASE_SPEED);
  delay(500);

  Serial.println(F("Shooting stage complete"));
}

// =====================================================================
// GREEN ZONE (BRIDGE — Toroslar)
// =====================================================================
void handleGreenZone() {
  Serial.println(F(">>> GREEN - BRIDGE <<<"));

  // 1. Green LED ON immediately
  setAllPixels(0, 200, 0);
  currentColor = COLOR_GREEN;

  // 2. Drive across the bridge with a SOFTER PID — keep going while we still see green
  Serial.println(F("Crossing the bridge"));
  unsigned long bridgeStart = millis();
  int greenLostCounter = 0;

  while (true) {
    unsigned int position = qtra.readLineWhite(sensorValues);
    int error = position - 1500;

    // SOFTER PID for the bridge (50% gentler steering)
    int correction = (Kp * 0.5) * error + (Kd * 0.5) * (error - lastError);
    lastError = error;

    int bridgeSpeed = BASE_SPEED - 20;
    int leftSpeed = bridgeSpeed + correction;
    int rightSpeed = bridgeSpeed - correction;
    leftSpeed = constrain(leftSpeed, 50, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 50, MAX_SPEED);

    motorLeft(leftSpeed);
    motorRight(rightSpeed);

    // Check — has the green zone ended?
    // Require 5 consecutive "not green" readings to avoid false triggers
    if (readColor() != COLOR_GREEN) {
      greenLostCounter++;
      if (greenLostCounter >= 5) {
        Serial.println(F("Bridge ended"));
        break;
      }
    } else {
      greenLostCounter = 0;
    }

    // Safety: max 10 seconds on the bridge
    if (millis() - bridgeStart > 10000) {
      Serial.println(F("Bridge timeout"));
      break;
    }

    delay(20);
  }

  Serial.println(F("Bridge crossed"));
}

// =====================================================================
// FINISH
// =====================================================================
void finishRace() {
  Serial.println(F(">>> FINISH <<<"));

  // Drive to the finish gate following the white line
  unsigned long startTime = millis();
  while (millis() - startTime < FINISH_DRIVE_MS) {
    unsigned int position = qtra.readLineWhite(sensorValues);
    int error = position - 1500;
    int correction = Kp * error + Kd * (error - lastError);
    lastError = error;

    int leftSpeed = BASE_SPEED + correction;
    int rightSpeed = BASE_SPEED - correction;
    motorLeft(leftSpeed);
    motorRight(rightSpeed);

    // Check for end of track (all sensors see white = no more line under the robot)
    if (sensorValues[0] < 200 && sensorValues[1] < 200 &&
        sensorValues[2] < 200 && sensorValues[3] < 200) {
      break;
    }
    delay(5);
  }

  motorStop();
  delay(500);

  Serial.println(F("RACE COMPLETED!"));

  // Finish animation — rainbow
  rainbowAnimation(5000);

  // Fade out
  for (int b = 100; b >= 0; b--) {
    setAllPixels(b, b, b);
    delay(20);
  }

  // Stay stopped forever
  while (true) {
    motorStop();
    delay(100);
  }
}

// =====================================================================
// MOTOR CONTROL
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
// COLOR SENSOR
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
  if (WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
