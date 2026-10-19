// LEGACY: the IBT-2 / BTS7960 driver this sketch used before the L298N
// (September 2026), kept here in case the big bridges come back. Everything
// between the #if 0 and #endif is skipped by the compiler, so this tab costs
// nothing and defines nothing. To go back: replace the pin block in config.h
// with the one below, replace wheels.ino with the functions below, and delete
// this tab (or leave it -- it is inert either way).

#if 0

// ---- config.h, the pin block ------------------------------------------------

// Board: 30-pin ESP32 DevKit v1 (WROOM-32, CH340). The comment after each line
// is the silkscreen label to land the wire on -- note GPIO 36 and 39 are
// printed "VP" and "VN", not as numbers.
//
// If you soldered EN/IS elsewhere, this block is the only place to change. EN
// must be output-capable; IS is read-only, so 34-39 suit it -- those are
// input-only (the ESP32 cannot drive them even by mistake) and all on ADC1,
// which keeps reading while Wi-Fi is up.
int RIGHT_R_PWM = 25;   // D25
int RIGHT_L_PWM = 26;   // D26
int RIGHT_R_EN = 18;    // D18
int RIGHT_L_EN = 19;    // D19
int RIGHT_R_IS = 36;    // VP  <- not labelled "36"
int RIGHT_L_IS = 39;    // VN  <- not labelled "39"

int LEFT_R_PWM = 32;    // D32
int LEFT_L_PWM = 33;    // D33
int LEFT_R_EN = 21;     // D21
int LEFT_L_EN = 22;     // D22
int LEFT_R_IS = 34;     // D34
int LEFT_L_IS = 35;     // D35

// ---- wheels.ino --------------------------------------------------------------

// The two IBT-2 (BTS7960) bridges. This is the only tab that touches a pin:
// setupLeftWheel() / setupRightWheel() configure them, enableWheel() is the
// only function that moves EN, and rotateWheel() is the only one that writes
// PWM. Nothing here knows about the stick, the page or the network.

void driveWheel(char wheel, int speed) {
  int dir = 0;
  if (speed > 0) {
    dir = 1;
  } else if (speed < 0) {
    dir = -1;
  }
  rotateWheel(wheel, dir, abs(speed));
}

void stopWheels() {
  rotateWheel('L', 0, 0);
  rotateWheel('R', 0, 0);
}

// EN starts LOW: the bridge is gated off before anything else runs.
// IS are the driver's own current-sense outputs -- they are INPUTs here. Never
// drive them, or the ESP32 and the BTS7960 fight over the same net.
void setupLeftWheel() {
  pinMode(LEFT_R_PWM, OUTPUT);
  pinMode(LEFT_L_PWM, OUTPUT);
  pinMode(LEFT_R_EN, OUTPUT);
  pinMode(LEFT_L_EN, OUTPUT);
  digitalWrite(LEFT_R_EN, LOW);
  digitalWrite(LEFT_L_EN, LOW);
  pinMode(LEFT_R_IS, INPUT);
  pinMode(LEFT_L_IS, INPUT);
}

void setupRightWheel() {
  pinMode(RIGHT_R_PWM, OUTPUT);
  pinMode(RIGHT_L_PWM, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT);
  pinMode(RIGHT_L_EN, OUTPUT);
  digitalWrite(RIGHT_R_EN, LOW);
  digitalWrite(RIGHT_L_EN, LOW);
  pinMode(RIGHT_R_IS, INPUT);
  pinMode(RIGHT_L_IS, INPUT);
}

// The only function that touches EN. Both halves of a bridge move together,
// and the PWM is always brought down first so neither edge can hand the driver
// a live duty cycle.
void enableWheel(char wheel, bool on) {
  rotateWheel(wheel, 0, 0);

  if (wheel == 'R') {
    rightSpeed = 0;
    rightEnabled = on;
  } else {
    leftSpeed = 0;
    leftEnabled = on;
  }

  int level = on ? HIGH : LOW;
  digitalWrite((wheel == 'R') ? RIGHT_R_EN : LEFT_R_EN, level);
  digitalWrite((wheel == 'R') ? RIGHT_L_EN : LEFT_L_EN, level);
}

void rotateWheel(char wheel, int dir, int speed) {
  int rPin = (wheel == 'R') ? RIGHT_R_PWM : LEFT_R_PWM;
  int lPin = (wheel == 'R') ? RIGHT_L_PWM : LEFT_L_PWM;
  speed = constrain(speed, 0, 255);

  if (dir >= 0) {
    analogWrite(lPin, 0);
  }
  if (dir <= 0) {
    analogWrite(rPin, 0);
  }
  if (dir == 1) {
    analogWrite(rPin, speed);
  }
  if (dir == -1) {
    analogWrite(lPin, speed);
  }
}

#endif
