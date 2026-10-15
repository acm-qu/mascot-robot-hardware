// The L298N: one board, both wheels. This is the only tab that touches a
// wheel pin: setupLeftWheel() / setupRightWheel() configure them,
// enableWheel() is the only function that changes whether a wheel may run, and
// rotateWheel() is the only one that writes the pins. Nothing here knows about
// the stick, the page or the network.
//
// Per wheel the L298N takes two direction inputs and one enable: IN1/IN2 and
// ENA for the right wheel (OUT1/OUT2), IN3/IN4 and ENB for the left
// (OUT3/OUT4). The direction pins choose which way current flows -- HIGH/LOW
// one way, LOW/HIGH the other, LOW/LOW lets the motor coast -- and the enable
// pin carries the PWM that sets the speed. So "disarmed" is simply ENA or ENB
// held at 0: no duty cycle, no drive, whatever the IN pins say.

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

// Direction pins LOW and the enable at 0 before anything else runs: the motor
// coasts and no duty cycle can reach it.
void setupLeftWheel() {
  pinMode(LEFT_IN3, OUTPUT);
  pinMode(LEFT_IN4, OUTPUT);
  digitalWrite(LEFT_IN3, LOW);
  digitalWrite(LEFT_IN4, LOW);
  pinMode(LEFT_EN, OUTPUT);
  analogWrite(LEFT_EN, 0);
}

void setupRightWheel() {
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  pinMode(RIGHT_EN, OUTPUT);
  analogWrite(RIGHT_EN, 0);
}

// The only function that arms or disarms a wheel. The PWM is brought to 0 and
// the stored speed zeroed on both edges, so re-arming cannot resume at whatever
// the wheel was doing, and disarming cannot leave a duty cycle on the pin.
void enableWheel(char wheel, bool on) {
  rotateWheel(wheel, 0, 0);

  if (wheel == 'R') {
    rightSpeed = 0;
    rightEnabled = on;
  } else {
    leftSpeed = 0;
    leftEnabled = on;
  }
}

// The only function that writes a wheel pin. A disarmed wheel gets 0 on its
// enable pin no matter what it is asked for -- that is the whole of the
// arm/disarm safety on an L298N, so it is enforced here, at the pin, and not
// left to the callers.
void rotateWheel(char wheel, int dir, int speed) {
  int aPin = (wheel == 'R') ? RIGHT_IN1 : LEFT_IN3;
  int bPin = (wheel == 'R') ? RIGHT_IN2 : LEFT_IN4;
  int enPin = (wheel == 'R') ? RIGHT_EN : LEFT_EN;
  bool enabled = (wheel == 'R') ? rightEnabled : leftEnabled;
  speed = constrain(speed, 0, 255);
  if (!enabled || dir == 0) {
    speed = 0;
  }

  // Duty off first, then the direction, then the duty back: the motor never
  // sees a direction change under power.
  analogWrite(enPin, 0);
  digitalWrite(aPin, (dir == 1) ? HIGH : LOW);
  digitalWrite(bPin, (dir == -1) ? HIGH : LOW);
  analogWrite(enPin, speed);
}
