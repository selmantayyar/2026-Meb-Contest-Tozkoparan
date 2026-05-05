// ============================================================
// AvciTozkoparan robot — line follower with color shooter
// HOW IT RUNS: setup() runs ONCE at power-on, then loop() runs
// over and over forever.
// ============================================================

// --- LIBRARIES (extra code we borrow) ---
#include <Wire.h>                    // lets sensors talk to the Arduino
#include "Adafruit_TCS34725.h"       // color sensor
#include <QTRSensors.h>              // line sensors (the 4 under the robot)
#include <Servo.h>                   // servo = the shooter arm

// --- ROBOT MEMORY (variables) ---
int Pist = 0;            // which path we picked (1 = short, 2 = long)
int i = 0;               // helper counter
int R = 0;               // red brightness for LEDs
int G = 0;               // green brightness for LEDs
int B = 0;               // blue brightness for LEDs
int color = 0;           // 0=none, 1=blue, 2=red, 3=green
int say = 0;             // (not used)
int Atis = 0;            // 0 = haven't shot yet, 1 = already shot ("atış" = shot)
int sagdon = 0;          // (not used) "sağ dön" = turn right
int soldon = 0;          // (not used) "sol dön" = turn left
int integral = 0;        // PID: total error added up over time
float Ki = 0.00001;      // PID I — tiny long-term correction
int LedOff = 1;          // 1 = LEDs allowed to turn off
int MZ80 = 2;            // pin for the front distance sensor
int DS1 = 12;            // pin for switch 1 (picks the path)
int DS2 = 13;            // pin for switch 2
int Say = 0;             // counts the crossroads we drove over ("sayı" = number)

// 4 line sensors plugged into pins A0, A1, A2, A3
QTRSensorsAnalog qtra((unsigned char[]){ 0, 1, 2, 3 }, 4);
unsigned int sensorValues[4];   // what the line sensors see

int tabanhiz = 150;      // normal driving speed ("taban hızı" = base speed)
int maxhiz = 200;        // top speed ("max hız" = max speed)
float Kp = 0.1;          // PID P — how hard to correct steering
float Kd = 3;            // PID D — how smoothly to correct
int sonhata = 0;         // last error ("son hata" = last error)
int hata = 0;            // how far off the black line we are right now
int solmotorpwm = 0;     // left motor power
int sagmotorpwm = 0;     // right motor power
int zemin = 1;           // floor mode: 1 = black line on white, 0 = white on black ("zemin" = floor)

Servo myservo;           // the shooter arm
int LPwm  = 11;          // LEFT motor speed pin
int LDir1 = 10;          // LEFT motor direction pin 1
int LDir2 = 9;           // LEFT motor direction pin 2
int RPwm  = 6;           // RIGHT motor speed pin
int RDir1 = 7;           // RIGHT motor direction pin 1
int RDir2 = 8;           // RIGHT motor direction pin 2
int Laser = 4;           // pin for the aiming laser

// --- LED STRIP (8 colored lights on top) ---
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#define PIN 5                                 // LED strip is wired to pin 5
#define NUMPIXELS 8                           // we have 8 LEDs
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 1
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int Btn = 0;             // start button is on pin 0

// =====================================================
// setup() — runs ONCE when the robot turns on
// =====================================================
void setup() {
  Serial.begin(9600);                // open serial (so we can print messages to computer)
  myservo.attach(3);                 // shooter arm is on pin 3
  myservo.write(180);                // park the arm in "ready" position
  pinMode(Btn, INPUT_PULLUP);        // start button is an input
  pinMode(Laser, OUTPUT);            // laser is an output (we turn it on/off)
  pinMode(6, OUTPUT);                // motor pins are all outputs
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, INPUT_PULLUP);         // switch DS1 — input
  pinMode(13, INPUT_PULLUP);         // switch DS2 — input
  pinMode(MZ80, INPUT);              // distance sensor — input
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  pixels.begin();                    // start the LED strip

  // Check that the color sensor is plugged in
  if (tcs.begin()) {
    Serial.println("Sensor VAR");    // "Sensör var" = "Sensor is here" — found it!
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ;  // hata!  (= "error" — stop here forever)
  }

  // --- CALIBRATION ---
  // Wiggle left and right 8 times so the line sensors learn
  // what the white floor and the black line look like.
  for (i = 0; i < 80; i++) {
    if (0 <= i && i < 5)  motorkontrol(-150, 150);   // wiggle one way
    if (5 <= i && i < 15) motorkontrol(150, -150);   // wiggle the other way
    if (15 <= i && i < 25) motorkontrol(-150, 150);
    if (25 <= i && i < 35) motorkontrol(150, -150);
    if (35 <= i && i < 45) motorkontrol(-150, 150);
    if (45 <= i && i < 55) motorkontrol(150, -150);
    if (55 <= i && i < 65) motorkontrol(-150, 150);
    if (65 <= i && i < 70) motorkontrol(150, -150);
    if (i >= 70) {
      motorkontrol(0, 0);             // last steps: STOP
      delay(5);
    }
    qtra.calibrate();                 // remember what the floor looks like
    delay(3);
  }
  digitalWrite(Laser, LOW);           // turn laser OFF
  motorkontrol(0, 0);                 // make sure motors are stopped
}

// =====================================================
// loop() — runs OVER AND OVER forever
// =====================================================
void loop() {
  Stop();                                          // pause if something is in the way

  // --- READ THE LINE ---
  unsigned int sensors[4];
  unsigned int position = qtra.readLine(sensors, 1, zemin);
  hata = position - 1500;                          // how far off the line we are (0 = perfect)
  integral += hata;                                // add error to running total
  int duzeltmehizi = Kp * hata + Kd * (hata - sonhata) + Ki * integral;  // PID math: how much to steer
  sonhata = hata;                                  // remember error for next loop

  // Auto-detect: are we on a white floor or a black floor?
  if (sensors[0] < 200 && sensors[3] < 200) { zemin = 0; }  // white floor
  if (sensors[0] > 201 && sensors[3] > 201) { zemin = 1; }  // black floor

  //    Serial.print("s0=");
  //    Serial.print(sensors[0]);
  //    Serial.print("    s2=");
  //    Serial.print(sensors[1]);
  //    Serial.print("    s3=");
  //    Serial.print(sensors[2]);
  //    Serial.print("    s4=");
  //    Serial.print(sensors[3]);
  //Serial.print("   Hata=");
  // Serial.println(hata);



  // --- DRIVE THE MOTORS ---
  // Make one wheel faster than the other to steer toward the line.
  sagmotorpwm = tabanhiz + duzeltmehizi;           // right motor
  solmotorpwm = tabanhiz - duzeltmehizi;           // left motor
  sagmotorpwm = constrain(sagmotorpwm, -100, maxhiz);   // never go faster than max
  solmotorpwm = constrain(solmotorpwm, -100, maxhiz);

  // If the line totally disappeared on one side, do a sharp turn to find it
  if (hata == (-1500)) motorkontrol(175, -125);    // line lost on the LEFT  → spin LEFT  to chase it
  else if (hata == 1500) motorkontrol(-125, 175);  // line lost on the RIGHT → spin RIGHT to chase it
  else
    motorkontrol(solmotorpwm, sagmotorpwm);        // normal smooth driving (PID)

  // --- CROSSROAD DETECTED ---
  // When ALL 4 line sensors see black at once, we just drove over a crossroad.
  if (analogRead(A3) < 700 && analogRead(A2) < 700 && analogRead(A1) < 700 && analogRead(A0) < 700) {
    Say = Say + 1;                                 // count it
    motorkontrol(0, 0);                            // stop briefly
    delay(500);
    motorkontrol(60, 60);                          // creep forward
    delay(250);
  }

  // --- AT THE 3rd CROSSROAD: READ COLOR AND SHOOT ---
  if (analogRead(A3) < 700 && analogRead(A2) < 700 && analogRead(A1) < 700 && analogRead(A0) < 700 && Atis == 0 && Say == 3) {
    motorkontrol(60, 60);
    delay(50);
    TCSRead();                                     // look at the color sensor
    if (color == 2) {                              // color 2 = RED
      if (digitalRead(DS1) == HIGH) {              // SWITCH UP → take PATH 1 (short)
        Pist = 1;
        motorkontrol(60, 60);
        delay(3600);  // Kırmızıda ileri gitme süresi  (drive forward at red)
        motorkontrol(0, 0);
        delay(1000);
        myservo.write(90);                         // SHOOT (move arm to 90°)
        delay(1000);
        Atis = 1;                                  // remember: we have shot
        motorkontrol(80, -80);
        delay(550);  // 90 dönüş süresi              (turn 90 degrees)
        motorkontrol(60, 60);
        delay(4500);  // atış sonrası düz gitme süresi  (drive forward after shooting)
        hata = 0;
      }
      if (digitalRead(DS1) == LOW) {               // SWITCH DOWN → take PATH 2 (long, with 5 turns)
        Pist = 2;
        motorkontrol(60, 60);
        delay(3600);  // Kırmızıda ileri gitme süresi
        motorkontrol(0, 0);
        delay(1000);
        myservo.write(90);                         // SHOOT
        delay(1000);
        Atis = 1;
        motorkontrol(80, -80);
        delay(550);  // 90 dönüş süresi              (turn 1)
        motorkontrol(0, 0);
        delay(500);
        motorkontrol(80, -80);
        delay(550);  // 90 dönüş süresi              (turn 2)
        motorkontrol(0, 0);
        delay(500);
        motorkontrol(80, -80);
        delay(550);  // 90 dönüş süresi              (turn 3)
        motorkontrol(0, 0);
        delay(500);
        motorkontrol(80, -80);
        delay(550);  // 90 dönüş süresi              (turn 4)
        motorkontrol(0, 0);
        delay(500);
        motorkontrol(80, -80);
        delay(550);  // 90 dönüş süresi              (turn 5)
        motorkontrol(0, 0);
        delay(500);
        motorkontrol(60, 60);
        delay(4500);  // atış sonrası düz gitme süresi
        hata = 0;
      }

      // Led Söndür   (turn the LEDs OFF)
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
        pixels.show();
        //delay(0.1);
      }
    }
  }


  // --- AT THE 5th CROSSROAD: GREEN FINISH ---
  if (analogRead(A3) < 700 && analogRead(A2) < 700 && analogRead(A1) < 700 && analogRead(A0) < 700 && Atis == 1 && Say == 5) {
    // Light all 8 LEDs GREEN
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 100, 0));
      pixels.show();
      //delay(0.1);
    }
    motorkontrol(60, 60);
    delay(8500);  // Yeşilde ileri gitme süresi   (drive forward at green)

    // Turn LEDs OFF
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      pixels.show();
      //delay(0.1);
    }
  }
}


// =====================================================
// Stop() — pause motors when the front sensor sees something
// =====================================================
void Stop() {

  while (digitalRead(MZ80) == LOW) {  // while obstacle is in front...
    motorkontrol(0, 0);               // ...keep motors stopped
    delay(1);
  }
}

// =====================================================
// motorkontrol() — drives the two motors
//   sagmotorpwm = right motor power (negative = backward)
//   solmotorpwm = left  motor power (negative = backward)
// =====================================================
void motorkontrol(int sagmotorpwm, int solmotorpwm) {
  if (sagmotorpwm <= 0) {              // RIGHT motor: negative number → go BACKWARD
    sagmotorpwm = abs(sagmotorpwm);
    analogWrite(RPwm, sagmotorpwm);
    digitalWrite(RDir1, LOW);
    digitalWrite(RDir2, HIGH);
  } else {                             // RIGHT motor: positive → go FORWARD
    analogWrite(RPwm, sagmotorpwm);
    digitalWrite(RDir1, HIGH);
    digitalWrite(RDir2, LOW);
  }
  if (solmotorpwm <= 0) {              // LEFT motor: negative → BACKWARD
    solmotorpwm = abs(solmotorpwm);
    analogWrite(LPwm, solmotorpwm);
    digitalWrite(LDir1, LOW);
    digitalWrite(LDir2, HIGH);
  } else {                             // LEFT motor: positive → FORWARD
    analogWrite(LPwm, solmotorpwm);
    digitalWrite(LDir1, HIGH);
    digitalWrite(LDir2, LOW);
  }
}

// =====================================================
// TCSRead() — reads the color sensor and lights up LEDs
// =====================================================
void TCSRead() {
  float red, green, blue;
  tcs.setInterrupt(false);
  tcs.getRGB(&red, &green, &blue);     // get red, green, blue values
  tcs.setInterrupt(true);

  //   Serial.print("R:\t"); Serial.print(int(red));
  //  Serial.print("\tG:\t"); Serial.print(int(green));
  //  Serial.print("\tB:\t"); Serial.print(int(blue));

  if (red < 100 && green < 100 && blue > 100) {        // mostly blue → BLUE
    LedOff = 0;
    color = 1;  // mavi   (= blue)
    R = 0;
    G = 0;
    B = 250;
  } else if (red > 100 && green < 100 && blue < 100) { // mostly red → RED
    LedOff = 0;
    color = 2;  // kırmızı (= red)
    R = 250;
    G = 0;
    B = 0;
  } else if (red < 100 && green > 100 && blue < 100) { // mostly green → GREEN
    LedOff = 0;
    color = 3;  // yeşil  (= green)
    R = 0;
    G = 250;
    B = 0;
  } else if (LedOff == 1) {                            // nothing → OFF
    color = 0;  // sön    (= off / extinguish)
    R = 0;
    G = 0;
    B = 0;
  }
  // Light all 8 LEDs with whatever color we picked
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(R, G, B));
    pixels.show();
    //delay(0.1);
  }
}
