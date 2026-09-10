// Mascot robot - two IBT-2 (BTS7960) drivers steered by an HW-504 joystick.
//
// !! REWIRE BEFORE UPLOADING !!  The old sketch put the four PWM wires on
// A0/0/A3/A4.  The ATmega328P has no PWM hardware on the analog pins, so
// analogWrite() there was really just digitalWrite() - the wheels were full
// speed above 128 and off below it.  The PWM wires have to move to real
// PWM pins (5, 6, 3, 11) for the joystick to control speed at all.
//
//   signal        old pin   new pin
//   LEFT  RPWM      A4        D5      <- moved
//   LEFT  LPWM      A3        D6      <- moved
//   LEFT  R_EN      D2        D2
//   LEFT  L_EN      D4        D4
//   LEFT  R_IS      D6        A4      <- moved
//   LEFT  L_IS      D7        A3      <- moved
//   RIGHT RPWM      A1        D3      <- moved
//   RIGHT LPWM      A0        D11     <- moved
//   RIGHT R_EN      D3        D7      <- moved
//   RIGHT L_EN      D5        D8      <- moved
//   RIGHT R_IS      D9        A2      <- moved
//   RIGHT L_IS      D8        A1      <- moved
//   joystick VRX    -         A2
//   joystick VRY    -         A5
//   joystick SW     -         D13
//   joystick +5V    -         5V
//   joystick GND    -         GND
//
// Each IS wire takes over the analog pin its own PWM wire just left, so the
// pins still bolted to the drivers' PWM inputs are always driven LOW.  If the
// sketch gets uploaded before the rewiring is done the wheels stay still.
// Keep the motor supply switched off while uploading anyway.
//
// Controls: push the stick to arm, push it again to stop.  Forward/back on
// the Y axis, steering on the X axis.

//Right Wheel BTS7960 motor driver
// int RIGHT_R_IS = A4;
// int RIGHT_L_IS = A3;
int RIGHT_R_PWM = 2;
int RIGHT_L_PWM = 3;
int RIGHT_R_EN = 4;
int RIGHT_L_EN = 5;

//Left Wheel BTS7960 motor driver
// int LEFT_R_IS = A2;
// int LEFT_L_IS = A1;
int LEFT_R_PWM = 8;
int LEFT_L_PWM = 9;
int LEFT_R_EN = 10;
int LEFT_L_EN = 11;

//HW-504 joystick
int JOY_X = A0;
int JOY_Y = A1;
int JOY_SW = 13;

// Flip any of these to -1 if something drives the wrong way.  RIGHT_INVERT
// starts at -1 because the two motors face opposite ways on the chassis, so
// "forward" is clockwise for one wheel and counter-clockwise for the other.
int LEFT_INVERT = 1;
int RIGHT_INVERT = -1;
int INVERT_X = 1;
int INVERT_Y = 1;

int DEADZONE = 60;    // joystick counts around centre that count as "centred"
int MIN_PWM = 40;     // below this the motors only buzz, so start here
int MAX_PWM = 255;
int SLEW = 12;        // biggest PWM change allowed per loop, softens the kick
int LOOP_MS = 20;
int DEBOUNCE_MS = 50;

int centerX = 512;    // measured in setup(), the stick rarely sits at 512
int centerY = 512;

int leftSpeed = 0;    // signed speed being applied right now, -255 -> 255
int rightSpeed = 0;
bool armed = false;

int swReading = HIGH;
int swState = HIGH;
unsigned long swChangedAt = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(9600);

  setupRightWheel();
  setupLeftWheel();
  stopWheels();

  pinMode(JOY_SW, INPUT_PULLUP);
  calibrateJoystick();

  Serial.println(F("Mascot robot ready - push the joystick to arm."));
}

void loop() {
  readButton();

  int x = INVERT_X * (analogRead(JOY_X) - centerX);
  int y = INVERT_Y * (analogRead(JOY_Y) - centerY);
  x = applyDeadzone(x);
  y = applyDeadzone(y);

  int targetLeft = 0;
  int targetRight = 0;

  if (armed) {
    // Arcade mixing: forward/back from Y, steering from X.
    long l = (long)y + (long)x;
    long r = (long)y - (long)x;

    // Both can reach +-1024 in the corners.  Scale them back down together
    // instead of clipping, so a diagonal keeps its left/right ratio.
    long biggest = max(abs(l), abs(r));
    if (biggest > 512) {
      l = (l * 512) / biggest;
      r = (r * 512) / biggest;
    }

    targetLeft = toPwm(l);
    targetRight = toPwm(r);
  }

  leftSpeed = slew(leftSpeed, targetLeft);
  rightSpeed = slew(rightSpeed, targetRight);

  driveWheel('L', LEFT_INVERT * leftSpeed);
  driveWheel('R', RIGHT_INVERT * rightSpeed);

  report(x, y);
  delay(LOOP_MS);
}

// Averages the resting position so a stick that idles at 497 still reads as
// centred.  Clamped in case someone is holding it while the board boots.
void calibrateJoystick() {
  long sumX = 0;
  long sumY = 0;
  int samples = 32;

  for (int i = 0; i < samples; i++) {
    sumX = sumX + analogRead(JOY_X);
    sumY = sumY + analogRead(JOY_Y);
    delay(2);
  }

  centerX = constrain(sumX / samples, 412, 612);
  centerY = constrain(sumY / samples, 412, 612);

  Serial.print(F("joystick centre X="));
  Serial.print(centerX);
  Serial.print(F(" Y="));
  Serial.println(centerY);
}

// Zeroes the centre of the stick and stretches what is left back out over the
// full -512 -> 512 range, so the speed does not jump when you leave the
// deadzone.
int applyDeadzone(int v) {
  if (abs(v) <= DEADZONE) {
    return 0;
  }

  int magnitude = constrain(abs(v), DEADZONE, 512);
  magnitude = map(magnitude, DEADZONE, 512, 0, 512);
  return (v > 0) ? magnitude : -magnitude;
}

// -512 -> 512 of stick travel becomes a signed PWM value that skips the dead
// band at the bottom where the motors will not turn.
int toPwm(long v) {
  if (v == 0) {
    return 0;
  }

  long magnitude = constrain(abs(v), 1, 512);
  magnitude = map(magnitude, 1, 512, MIN_PWM, MAX_PWM);
  return (v > 0) ? (int)magnitude : -(int)magnitude;
}

int slew(int current, int target) {
  if (target > current + SLEW) {
    return current + SLEW;
  }
  if (target < current - SLEW) {
    return current - SLEW;
  }
  return target;
}

void readButton() {
  int reading = digitalRead(JOY_SW);

  if (reading != swReading) {
    swReading = reading;
    swChangedAt = millis();
  }

  if (millis() - swChangedAt > (unsigned long)DEBOUNCE_MS && reading != swState) {
    swState = reading;

    if (swState == LOW) {  // SW pulls to ground when pressed
      armed = !armed;

      if (!armed) {
        leftSpeed = 0;
        rightSpeed = 0;
        stopWheels();
      }

      enableDrivers(armed);
      Serial.println(armed ? F("ARMED") : F("STOPPED"));
    }
  }
}

void report(int x, int y) {
  if (millis() - lastPrint < 250) {
    return;
  }
  lastPrint = millis();

  Serial.print(armed ? F("ARMED ") : F("STOP  "));
  Serial.print(F("x="));
  Serial.print(x);
  Serial.print(F(" y="));
  Serial.print(y);
  Serial.print(F(" L="));
  Serial.print(leftSpeed);
  Serial.print(F(" R="));
  Serial.println(rightSpeed);
}

// speed is signed, -255 -> 255
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

// The EN pins gate the drivers in hardware, so a disarmed robot cannot move
// even if a PWM pin glitches.
void enableDrivers(bool on) {
  int level = on ? HIGH : LOW;
  digitalWrite(LEFT_R_EN, level);
  digitalWrite(LEFT_L_EN, level);
  digitalWrite(RIGHT_R_EN, level);
  digitalWrite(RIGHT_L_EN, level);
}

void setupLeftWheel() {
  // pinMode(LEFT_R_IS, OUTPUT);
  pinMode(LEFT_R_EN, OUTPUT);
  pinMode(LEFT_R_PWM, OUTPUT);
  // pinMode(LEFT_L_IS, OUTPUT);
  pinMode(LEFT_L_EN, OUTPUT);
  pinMode(LEFT_L_PWM, OUTPUT);
  // digitalWrite(LEFT_R_IS, LOW);
  // digitalWrite(LEFT_L_IS, LOW);
  digitalWrite(LEFT_R_EN, LOW);  // stays off until the stick arms it
  digitalWrite(LEFT_L_EN, LOW);
}

void setupRightWheel() {
  // pinMode(RIGHT_R_IS, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT);
  pinMode(RIGHT_R_PWM, OUTPUT);
  // pinMode(RIGHT_L_IS, OUTPUT);
  pinMode(RIGHT_L_EN, OUTPUT);
  pinMode(RIGHT_L_PWM, OUTPUT);
  // digitalWrite(RIGHT_R_IS, LOW);
  // digitalWrite(RIGHT_L_IS, LOW);
  digitalWrite(RIGHT_R_EN, LOW);
  digitalWrite(RIGHT_L_EN, LOW);
}

// wheel='R' / 'L'
// dir = 1 clockwise / -1 counter-clockwise / 0 stop
// speed = 0 -> 255
void rotateWheel(char wheel, int dir, int speed) {
  int rPin = (wheel == 'R') ? RIGHT_R_PWM : LEFT_R_PWM;
  int lPin = (wheel == 'R') ? RIGHT_L_PWM : LEFT_L_PWM;
  speed = constrain(speed, 0, 255);

  // Drop the idle half to zero before raising the other one - a BTS7960 must
  // never see both PWM inputs driven at the same time.
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
