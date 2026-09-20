// THIS IS THE FILE YOU OPEN AND UPLOAD. An Arduino sketch's entry point is the
// .ino named after its folder -- there is no index.ino -- and opening this one
// in the Arduino IDE brings every other file in the folder along as a tab.
// Upload compiles all of them together. See the README for board settings.

#include <WiFi.h>
#include <WebServer.h>

#include "config.h"

// The sketch is split across the tabs the Arduino IDE shows along the top:
//
//   wheel-control.ino   this file: shared globals, setup(), loop(), failsafe()
//   config.h            pins, tuning, Wi-Fi credentials -- the one file to edit
//   mixing.ino          the math: stick position -> a signed PWM per wheel
//   moves.ino           named moves: forward, spin_right, slight_left, ...
//   web.ino             the access point and every HTTP route
//   page.h              the control page the browser loads
//   wheels.ino          the two IBT-2 bridges: pins, enable, PWM writes
//
// The IDE compiles all the .ino tabs as one file -- this one first, the rest
// in alphabetical order, with a prototype for every function inserted up
// here. Two rules keep that working: a global that more than one tab uses is
// defined in this file (a tab may keep state only its own functions touch),
// and no struct of ours appears in a function signature.

WebServer server(80);

int webX = 0;
int webY = 0;
unsigned long lastCommand = 0;

// millis() of the last time somebody actually pushed the knob -- a deflected
// /drive, or the page STOP button -- and 0 if nobody has yet. The page idle
// heartbeat does not touch it. voice-activated.ino reads it through
// stickActive() to decide whether the joystick currently outranks the voice
// system; nothing else depends on it.
unsigned long lastStickInput = 0;

int leftSpeed = 0;
int rightSpeed = 0;

bool leftEnabled = false;
bool rightEnabled = false;

unsigned long lastStep = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);

  // Motors first, and stopped, before the radio draws its start-up current.
  setupRightWheel();
  setupLeftWheel();
  stopWheels();
  enableWheel('R', START_ENABLED);
  enableWheel('L', START_ENABLED);

  startAccessPoint();
  startServer();

  Serial.println(F("Mascot robot ready."));
}

void loop() {
  server.handleClient();

  // The voice system is a serial line, and a command off it can block here
  // for the length of a whole routine. It runs every pass rather than per
  // control step so a line is never left sitting in the UART buffer.
  serviceVoice();

  unsigned long now = millis();
  if (now - lastStep < (unsigned long)LOOP_MS) {
    return;
  }
  lastStep = now;

  controlStep(now);
}

// One control step, the whole of it. It is a function of its own only so that
// voice-activated.ino can run it: a routine blocks loop() for its own length,
// so it has to keep this step running or its moves would never reach a wheel.
void controlStep(unsigned long now) {
  // A running move stamps lastCommand itself, so this has to come before the
  // failsafe check or the timeout would cut every move at 500 ms.
  serviceMove(now);

  if (now - lastCommand > COMMAND_TIMEOUT_MS) {
    failsafe();
    return;
  }

  int x = applyDeadzone(INVERT_X * webX);
  int y = applyDeadzone(INVERT_Y * webY);

  long l = 0;
  long r = 0;
  arcadeMix(x, y, l, r);

  // EN already has the bridge off, but ramping the stored speed down keeps the
  // state honest so re-arming cannot resume at whatever it was doing.
  int targetLeft = leftEnabled ? toPwm(l) : 0;
  int targetRight = rightEnabled ? toPwm(r) : 0;

  leftSpeed = slew(leftSpeed, targetLeft);
  rightSpeed = slew(rightSpeed, targetRight);

  driveWheel('L', LEFT_INVERT * leftSpeed);
  driveWheel('R', RIGHT_INVERT * rightSpeed);

  report(x, y);
}

// The link is gone and there is no telling where the robot is pointed, so cut
// the drive outright instead of ramping it down. Letting slew() handle it would
// add another ~440 ms of travel on top of the timeout. A released knob still
// ramps down normally -- this path is only for a link that stopped answering.
void failsafe() {
  webX = 0;
  webY = 0;

  if (leftSpeed == 0 && rightSpeed == 0) {
    return;
  }

  leftSpeed = 0;
  rightSpeed = 0;
  stopWheels();
  Serial.println(F("link lost -- motors cut"));
}

void report(int x, int y) {
  if (millis() - lastPrint < 250) {
    return;
  }
  lastPrint = millis();

  Serial.print(F("x="));
  Serial.print(x);
  Serial.print(F(" y="));
  Serial.print(y);
  Serial.print(F(" L="));
  Serial.print(leftSpeed);
  Serial.print(F(" R="));
  Serial.println(rightSpeed);
}
