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
