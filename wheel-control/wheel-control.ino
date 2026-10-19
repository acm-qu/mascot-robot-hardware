// THIS IS THE FILE YOU OPEN AND UPLOAD. An Arduino sketch's entry point is the
// .ino named after its folder -- there is no index.ino -- and opening this one
// in the Arduino IDE brings every other file in the folder along as a tab.
// Upload compiles all of them together. See the README for board settings.

#include <WiFiS3.h>
#include <stdarg.h>

#include "config.h"
#include "mini-server.h"

// The sketch is split across the tabs the Arduino IDE shows along the top:
//
//   wheel-control.ino   this file: shared globals, logf(), setup(), bootDance(), loop(), failsafe()
//   config.h            pins, tuning, Wi-Fi credentials -- the one file to edit
//   mini-server.h       the tiny WebServer stand-in over WiFiS3 (UNO R4 has none)
//   mixing.ino          the math: stick position -> a signed PWM per wheel
//   moves.ino           named moves: forward, spin_right, slight_left, ...
//   faces.ino           the old /face gestures -- fully commented out; the tablet now sends bare labels handled in serial-link.ino
//   routines.ino        the spin, dance and emotion routines, and who gets the wheels
//   routines.h          what a routine is and promises; the reference for it and the serial link
//   serial-link.ino     reads commands off VOICE_LINK and calls the routines
//   web.ino             the access point and every HTTP route
//   page.h              the control page the browser loads
//   servo.ino           the servo: attach, angle -> pulse (Servo library)
//   wheels.ino          the L298N: direction pins, enable/PWM writes
//
// The IDE compiles all the .ino tabs as one file -- this one first, the rest
// in alphabetical order, with a prototype for every function inserted up
// here. Two rules keep that working: a global that more than one tab uses is
// defined in this file (a tab may keep state only its own functions touch),
// and no struct of ours appears in a function signature.

MiniServer server(80);

// printf() for the log. The UNO R4 core's Serial has no printf(), so
// the sketch's log lines route through this instead -- one buffer, formatted
// with vsnprintf and printed. Defined here in the main tab so every tab can
// call it; the explicit prototype keeps the variadic signature unambiguous.
void logf(const char *fmt, ...);
void logf(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
}

int webX = 0;
int webY = 0;
unsigned long lastCommand = 0;

// millis() of the last time somebody actually pushed the knob -- a deflected
// /drive, or the page STOP button -- and 0 if nobody has yet. The page idle
// heartbeat does not touch it. routines.ino reads it through
// stickActive() to decide whether the joystick currently outranks the serial
// link; nothing else depends on it.
unsigned long lastStickInput = 0;

int leftSpeed = 0;
int rightSpeed = 0;

bool leftEnabled = false;
bool rightEnabled = false;

// Where the servo was last told to go, 0-180. servo.ino writes it, web.ino
// reports it in every status line so the page's slider tracks the board.
int servoDeg = 0;

unsigned long lastStep = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);

  // The serial link (serial-link.ino) reads VOICE_LINK. When that is Serial
  // it is already open; when config.h points it at Serial1 it needs its own
  // begin(), at the same 115200 the sender is told to use.
  if ((void *)&VOICE_LINK != (void *)&Serial) {
    VOICE_LINK.begin(115200);
  }

  // Motors first, and stopped, before the radio draws its start-up current.
  setupRightWheel();
  setupLeftWheel();
  stopWheels();
  enableWheel('R', START_ENABLED);
  enableWheel('L', START_ENABLED);

  // After the wheels. On the UNO R4 the Servo library and analogWrite() both
  // draw on the RA4M1's general-purpose timers; the servo pin is kept off the
  // wheel EN pins so they do not contend -- see servo.ino.
  setupServo();

  startAccessPoint();
  startServer();

  // Disabled for now -- uncomment to have the robot dance on every boot.
  // bootDance();

  Serial.println(F("Mascot robot ready."));
}

// When it is enabled in setup() (it ships commented out), the first thing the
// robot does after a flash -- and after every reset, since setup() cannot tell
// the two apart -- is the dance routine from routines.ino. It goes after startServer() on purpose: a routine pumps
// server.handleClient() while it blocks, and the page STOP button is the only
// HTTP route that can end one, so a dance started before the server was up
// would have no software stop at all.
//
// beginRoutine() refuses unless both wheels are armed, and START_ENABLED ships
// false, so the wheels are armed for exactly the length of the dance and put
// back to START_ENABLED afterwards, whatever dance() returned. enableWheel()
// zeroes the stored speed on both edges, so the board still lands in the
// documented state -- disarmed unless config.h says otherwise -- with nothing
// left to resume. Put the chassis on blocks before uploading: the knob cannot
// interrupt a running routine.
void bootDance() {
  enableWheel('R', true);
  enableWheel('L', true);

  dance();

  enableWheel('R', START_ENABLED);
  enableWheel('L', START_ENABLED);
}

void loop() {
  server.handleClient();

  // A command off the serial line can block here for the length of a whole
  // routine. This runs every pass rather than per control step so a line is
  // never left sitting in the UART buffer.
  readSerial();

  unsigned long now = millis();
  if (now - lastStep < (unsigned long)LOOP_MS) {
    return;
  }
  lastStep = now;

  controlStep(now);
}

// One control step, the whole of it. It is a function of its own only so that
// routines.ino can run it: a routine blocks loop() for its own length,
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
