// LEGACY: the 30-pin ESP32 DevKit v1 + L298N build of this robot (WiFi.h +
// WebServer + LEDC servo), kept after the port to the Arduino UNO R4 WiFi on
// 2026-09-23. The ESP32 started acting up, so the sketch moved to the UNO R4;
// this tab is the ESP32-specific code the port replaced, so the move can be
// undone.
//
// This tab is the ONLY record of that ESP32-L298N build. It was never committed
// -- git HEAD is the even older IBT-2/BTS7960 ESP32 build (different pins,
// INVERT_Y=1, servo on D27, an IBT-2 wheels.ino), so `git checkout` does NOT
// bring this back. The four sections below are it.
//
// Everything between the #if 0 and #endif is skipped by the compiler, so this
// tab costs nothing and defines nothing. The #include lines inside are also
// commented out: the Arduino library scanner reads includes even inside a dead
// #if, and an uncommented <WiFi.h> here would drag the ESP32 WiFi library into
// a UNO R4 build.
//
// To go back to the ESP32-L298N build: install the esp32 core, select the ESP32
// DevKit v1, and restore the four sections below over config.h /
// wheel-control.ino / web.ino / servo.ino (uncommenting their includes). Then,
// in the tabs the port left in place:
//   - KEEP wheels.ino as it is -- it is the L298N driver, platform-neutral, and
//     the port did not touch it (do NOT revert it to git; HEAD is the IBT-2 one);
//   - in moves.ino and voice-activated.ino, change logf( back to Serial.printf(
//     and VOICE_LINK back to Serial;
//   - delete mini-server.h (the ESP32 core's WebServer replaces it).
// mixing.ino and page.h are unchanged either way.

#if 0

// ==========================================================================
// config.h -- the ESP32 pin block, LEDC servo range, and board notes
// ==========================================================================
// Every number you might need to change lives here: pins, tuning, the Wi-Fi
// network and the safety limits. Nothing else in the sketch defines any of
// them, and only wheel-control.ino includes this file.
#pragma once

// Board: 30-pin ESP32 DevKit v1 (WROOM-32, CH340). The comment after each line
// is the silkscreen label to land the wire on. One L298N drives both wheels:
// OUT1/OUT2 is the right wheel, OUT3/OUT4 the left. The IN pins set the
// direction and the EN pins carry the PWM, so a wheel with its EN at 0 is
// dead whatever its IN pins say -- that is what arming a wheel controls.
//
// The pins follow the usual ESP32 + L298N diagram (image.png in this folder).
// If you wired it differently, this block is the only place to change.
int RIGHT_IN1 = 27;     // D27  IN1
int RIGHT_IN2 = 26;     // D26  IN2
int RIGHT_EN = 14;      // D14  ENA  (remove the ENA jumper on the board)

int LEFT_IN3 = 25;      // D25  IN3
int LEFT_IN4 = 33;      // D33  IN4
int LEFT_EN = 32;       // D32  ENB  (remove the ENB jumper on the board)

int SERVO_CONTROL = 18; // D13  (was D27, which the L298N's IN1 now takes)

// The servo on SERVO_CONTROL: a standard hobby servo on a 50 Hz pulse, its
// angle set by the pulse width -- SERVO_MIN_US at 0 degrees, SERVO_MAX_US at
// 180. Most take 500-2500 us; narrow the range to 1000-2000 if yours buzzes
// or strains at either end. SERVO_START_DEG is where it goes at boot.
int SERVO_MIN_US = 500;
int SERVO_MAX_US = 2400;
int SERVO_START_DEG = 90;

int LEFT_INVERT = 1;
int RIGHT_INVERT = 1;
int INVERT_X = 1;
// -1 since the move to the L298N: with the wiring above, forward on the stick
// drove the robot backward. Flip back to 1 if yours goes the other way.
int INVERT_Y = 1;

int DEADZONE = 60;
int MIN_PWM = 40;
int MAX_PWM = 255;
int SLEW = 12;
int LOOP_MS = 20;

// CHANGE ME. WPA2 needs 8 characters or more -- with a shorter one softAP()
// silently brings the network up with no password at all.
const char *AP_SSID = "mascot-robot";
const char *AP_PASS = "mascotbot";

unsigned long COMMAND_TIMEOUT_MS = 500;

// Both wheels come up gated off, so after boot nothing can drive them until
// someone deliberately arms them in the page. The one exception is the boot
// dance in wheel-control.ino, which arms both wheels for its own length and
// puts them back to this value when it ends. Set true for the old behaviour of
// being live the moment the board boots.
bool START_ENABLED = false;

// Named moves (moves.ino) hold a stick position on the board's own clock. A
// /move request that gives no ms= holds for MOVE_DEFAULT_MS, and no request
// can hold for longer than MOVE_MAX_MS -- while a move runs the board feeds
// its own link failsafe, so this cap is what bounds a robot driving with
// nobody steering. See the README, section 9.
unsigned long MOVE_DEFAULT_MS = 500;
unsigned long MOVE_MAX_MS = 3000;

// Voice-activated routines (voice-activated.ino). Nothing on this robot
// measures how far it went -- there is no encoder on either wheel -- so a
// routine is open-loop and every number below is a stopwatch value for YOUR
// robot, YOUR floor and the battery voltage you run at. The values here are
// starting guesses, not measurements. Calibrate one move at a time:
//
//   curl "http://192.168.4.1/enable?l=1&r=1"
//   curl "http://192.168.4.1/move?name=spin_left&ms=1800&speed=70"
//
// then raise or lower ms until the robot lands where you want it, and copy
// the number here. Two things to expect while you do: distance is not
// proportional to time at these lengths, because SLEW spends the first ~0.4 s
// of every move ramping the wheels up, so doubling ms rather more than
// doubles the distance; and a flat battery undershoots every one of them.
//
// SPIN_360_MS has to stay at or below MOVE_MAX_MS. startMove() caps every
// hold there, silently, and a capped spin stops short of a full turn.
unsigned long SPIN_360_MS = 1800;   // one full turn on the spot
int SPIN_SPEED = 70;                // percent of full stick deflection

unsigned long NUDGE_MS = 400;       // roughly 10 cm forward or back
int NUDGE_SPEED = 50;

unsigned long WIGGLE_MS = 250;      // a small turn on the spot, either way
int WIGGLE_SPEED = 60;

// The joystick outranks the voice system, and this is how long it keeps the
// wheels after the last time somebody actually pushed the knob. A voice
// routine asked for inside this window is refused outright.
//
// This window is only about STARTING a routine, and it is the whole of the
// joystick priority rather than half of it. Once a routine is running it owns
// the wheels outright: the knob does not interrupt one and neither does a
// /move, and only a deliberate stop -- GET /stop, or "stop" on the voice
// link, both of which call cancelMove() -- ends it early. voice-activated.h
// sets out the whole arrangement.
//
// The window is what makes this a priority rather than a race. Nobody drives
// by deflecting the stick every millisecond: they cross centre, they pause
// between pushes, and letting go sends three zeros. Too short and the voice
// system can jump into a gap mid-drive; too long and "dance" stops working
// for a while after anyone touches the page. A second and a half is a pause,
// not a gap.
//
// Only a deflected knob and the page's STOP button count. The page's idle
// 0,0 heartbeat deliberately does not -- it arrives at 5 Hz for as long as
// anybody has the page open, so counting it would mean voice never worked
// with a phone connected.
unsigned long STICK_PRIORITY_MS = 1500;

// ==========================================================================
// wheel-control.ino -- WiFi.h / WebServer.h includes and the WebServer instance
// ==========================================================================
// THIS IS THE FILE YOU OPEN AND UPLOAD. An Arduino sketch's entry point is the
// .ino named after its folder -- there is no index.ino -- and opening this one
// in the Arduino IDE brings every other file in the folder along as a tab.
// Upload compiles all of them together. See the README for board settings.
// 
// #include <WiFi.h>
// #include <WebServer.h>
// 
// #include "config.h"

// The sketch is split across the tabs the Arduino IDE shows along the top:
//
//   wheel-control.ino   this file: shared globals, setup(), bootDance(), loop(), failsafe()
//   config.h            pins, tuning, Wi-Fi credentials -- the one file to edit
//   mixing.ino          the math: stick position -> a signed PWM per wheel
//   moves.ino           named moves: forward, spin_right, slight_left, ...
//   voice-activated.ino the spin and dance routines and the serial voice link
//   voice-activated.h   what a routine is and promises; the reference for it
//   web.ino             the access point and every HTTP route
//   page.h              the control page the browser loads
//   servo.ino           the servo: attach, angle -> pulse
//   wheels.ino          the L298N: direction pins, enable/PWM writes
//   legacy.ino          the old IBT-2 driver, inside #if 0 -- compiles to nothing
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

// Where the servo was last told to go, 0-180. servo.ino writes it, web.ino
// reports it in every status line so the page's slider tracks the board.
int servoDeg = 0;

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

  // After the wheels: their first analogWrite() (in setup*Wheel() above) took
  // the low LEDC channels, and the servo's own channel is chosen not to share
  // a timer with them -- see setupServo().
  // setupServo();

  startAccessPoint();
  startServer();

  // bootDance();

  Serial.println(F("Mascot robot ready."));
}

// The first thing the robot does after a flash -- and after every reset, since
// setup() cannot tell the two apart -- is the dance routine from
// voice-activated.ino. It goes after startServer() on purpose: a routine pumps
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

// ==========================================================================
// web.ino -- the softAP access point and the WebServer route table
// ==========================================================================
// The access point and every HTTP route. Each handler runs from inside
// server.handleClient(), on the same task as loop(), which is why none of the
// globals it writes need locking.
// 
// #include "page.h"

void startAccessPoint() {
  WiFi.mode(WIFI_AP);

  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println(F("softAP() failed -- no network, nothing can drive the robot"));
    return;
  }

  Serial.print(F("access point \""));
  Serial.print(AP_SSID);
  Serial.print(F("\" up, open http://"));
  Serial.println(WiFi.softAPIP());
}

void startServer() {
  server.on("/", handleRoot);
  server.on("/drive", handleDrive);
  server.on("/stop", handleStop);
  server.on("/enable", handleEnable);
  server.on("/move", handleMove);
  server.on("/moves", handleMoves);
  server.on("/servo", handleServo);
  server.onNotFound(handleNotFound);

  server.enableDelay(false);   // handleClient() must not sleep, loop() is the control loop
  server.begin();
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// "<leftSpeed>,<rightSpeed>,<leftEnabled>,<rightEnabled>,<servoDeg>" -- every
// route answers with it, so the page's readouts, buttons and slider track the
// board no matter which request came back last, or how many people have it
// open.
String statusBody() {
  return String(leftSpeed) + "," + String(rightSpeed) + ","
       + (leftEnabled ? "1" : "0") + "," + (rightEnabled ? "1" : "0") + ","
       + String(servoDeg);
}

void handleEnable() {
  // An absent argument leaves that wheel alone, so one button cannot disarm
  // the other by omission.
  if (server.hasArg("l")) {
    enableWheel('L', server.arg("l").toInt() != 0);
  }
  if (server.hasArg("r")) {
    enableWheel('R', server.arg("r").toInt() != 0);
  }

  Serial.printf("enable: L=%d R=%d\n", leftEnabled, rightEnabled);
  server.send(200, "text/plain", statusBody());
}

void handleDrive() {
  // A missing argument reads as "", and "".toInt() is 0 -- a missing axis is
  // centred, never the previous value.
  int x = constrain(server.arg("x").toInt(), -512, 512);
  int y = constrain(server.arg("y").toInt(), -512, 512);
  lastCommand = millis();

  // A deflected knob is somebody driving, and that outranks the voice system
  // for STICK_PRIORITY_MS -- see stickActive() in voice-activated.ino. The
  // 0,0 heartbeat below must not stamp it: that arrives at 5 Hz whenever the
  // page is open, so it would hold the priority forever.
  if (x != 0 || y != 0) {
    lastStickInput = millis();
  }

  // A routine owns the wheels for its whole length: the knob does not
  // interrupt one, it waits it out. Ending a routine early is a
  // deliberate act -- /stop, or "stop" on the voice link, both of which call
  // cancelMove(). The stamp above still happens while we wait, so the moment
  // the routine does end, the driver already outranks whatever the voice
  // system says next.
  if (routineRunning()) {
    server.send(200, "text/plain", statusBody());
    return;
  }

  // Past this point no routine is running, so "a move" here is a plain /move
  // request. The page heartbeats x=0,y=0 whenever it is open and idle, and
  // that must not overwrite one. A deflected knob does -- whoever is holding
  // the phone wins over whatever asked for the move.
  bool idleHeartbeat = moveRunning() && x == 0 && y == 0;
  if (!idleHeartbeat) {
    cancelMove();
    webX = x;
    webY = y;
  }

  server.send(200, "text/plain", statusBody());
}

void handleStop() {
  cancelMove();
  webX = 0;
  webY = 0;
  lastCommand = millis();

  // STOP is somebody at the page deciding the robot should not be moving, so
  // it takes the wheels off the voice system too -- otherwise a word heard a
  // moment earlier could start a routine right on top of the stop.
  lastStickInput = millis();

  server.send(200, "text/plain", statusBody());
}

// GET /move?name=<name>&ms=<hold>&speed=<0..100>. Only name is required: the
// defaults are MOVE_DEFAULT_MS and full speed, and startMove() caps the hold
// at MOVE_MAX_MS. A move already running is replaced.
void handleMove() {
  // Same rule as the knob, and for a sharper reason: a move started in the
  // middle of a routine does not cancel it. It replaces the step that is
  // running, runMove() waits out the new hold instead, and the routine then
  // carries on from the next step with its own timing already spent. Turning
  // it away is the only honest answer while a routine owns the wheels.
  if (routineRunning()) {
    server.send(409, "text/plain", "a routine is running -- /stop first");
    return;
  }

  String name = server.arg("name");
  long ms = server.arg("ms").toInt();
  int speed = server.hasArg("speed") ? server.arg("speed").toInt() : 100;

  if (!startMove(name, ms, speed)) {
    server.send(400, "text/plain", "unknown move \"" + name + "\", try one of: " + listMoves());
    return;
  }

  server.send(200, "text/plain", statusBody());
}

// GET /moves -- the names in the table, so a client can discover them rather
// than hard-code the list.
void handleMoves() {
  server.send(200, "text/plain", listMoves());
}

// GET /servo?a=<0..180>. The servo is its own thing: it does not feed the
// failsafe, count as stick input or wait for a routine -- it just goes there.
void handleServo() {
  if (!server.hasArg("a")) {
    server.send(400, "text/plain", "servo needs a=<0..180>");
    return;
  }

  setServo(server.arg("a").toInt());
  Serial.printf("servo: %d\n", servoDeg);
  server.send(200, "text/plain", statusBody());
}

void handleNotFound() {
  server.send(404, "text/plain", "not found");
}

// ==========================================================================
// servo.ino -- the servo driven straight off the ESP32 LEDC peripheral
// ==========================================================================
// The servo on SERVO_CONTROL, driven straight from the ESP32's LEDC
// peripheral at 50 Hz -- no library to install. Only this tab touches the
// pin, and setServo() is the only function that writes it.

// LEDC channels share timers in pairs (0-1, 2-3, 4-5, 6-7), and a timer has
// one frequency. The two wheel enable pins take channels 0-1 on their first
// analogWrite(), at analogWrite's 1 kHz; the servo needs 50 Hz, so it takes
// channel 7 -- timer 3, which nothing else uses -- whatever order the pins
// came up in.
void setupServo() {
  if (!ledcAttachChannel(SERVO_CONTROL, 50, 16, 7)) {
    Serial.println(F("servo: LEDC attach failed -- no servo"));
    return;
  }
  setServo(SERVO_START_DEG);
}

// 0-180 degrees -> a SERVO_MIN_US..SERVO_MAX_US pulse in a 20 ms period, as
// a 16-bit duty. Out-of-range angles are clamped, never refused.
void setServo(int deg) {
  deg = constrain(deg, 0, 180);
  servoDeg = deg;

  long us = SERVO_MIN_US + (long)(SERVO_MAX_US - SERVO_MIN_US) * deg / 180;
  ledcWrite(SERVO_CONTROL, (uint32_t)(us * 65535L / 20000L));
}

#endif
