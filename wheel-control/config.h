// Every number you might need to change lives here: pins, tuning, the Wi-Fi
// network and the safety limits. Nothing else in the sketch defines any of
// them, and only wheel-control.ino includes this file.
#pragma once

// Board: Arduino UNO R4 WiFi (Renesas RA4M1 + on-board ESP32-S3 radio). The
// numbers below are the Dn / silkscreen labels to land each wire on. One L298N
// drives both wheels: OUT1/OUT2 is the right wheel, OUT3/OUT4 the left. The IN
// pins set the direction and the EN pins carry the PWM, so a wheel with its EN
// at 0 is dead whatever its IN pins say -- that is what arming a wheel controls.
//
// Unlike the old ESP32, the UNO R4's logic is 5 V, which suits the L298N's
// inputs better than 3.3 V did. Only D3, D5, D6, D9, D10 and D11 do PWM
// (analogWrite), so the two EN pins have to be among those; the four IN pins
// are plain digital and can be any free pin. D0/D1 are left for Serial1 (a
// wired voice module -- see VOICE_LINK), and D13 is the on-board LED.
//
// If you wired it differently, this block is the only place to change.
int RIGHT_EN = 9;       // D9~  ENA  (PWM; remove the ENA jumper on the board)
int RIGHT_IN1 = 7;      // D7   IN1
int RIGHT_IN2 = 8;      // D8   IN2

int LEFT_IN3 = 4;       // D4   IN3
int LEFT_IN4 = 2;       // D2   IN4
int LEFT_EN = 10;       // D10~ ENB  (PWM; remove the ENB jumper on the board)

int SERVO_CONTROL = 11;  // D11  servo signal (driven by the Servo library)

// The servo on SERVO_CONTROL: a standard hobby servo on a 50 Hz pulse, its
// angle set by the pulse width -- SERVO_MIN_US at 0 degrees, SERVO_MAX_US at
// 180. Most take 500-2500 us; narrow the range to 1000-2000 if yours buzzes
// or strains at either end. SERVO_START_DEG is where it goes at boot.
int SERVO_MIN_US = 500;
int SERVO_MAX_US = 2400;
int SERVO_START_DEG = 90;

// Face-triggered body language (faces.ino). The app sends GET /face?name=<expr>
// whenever ACMO's expression changes, and the board answers with a tiny nudge
// and/or a servo gesture. The nudges go through the normal move pipeline, so
// they are gated by the wheel arm state -- nothing moves until the wheels are
// armed. Tiny by design; tune here.
unsigned long FACE_NUDGE_MS = 300;   // "a tiny bit"
int FACE_NUDGE_SPEED = 35;           // happy / passionate / angry / excited
int FACE_SAD_SPEED = 20;             // sad backs off, more gently
int FACE_ANNOYED_SERVO = 120;        // annoyed swings the servo here from 90
int FACE_SHAKE_LO = 85;              // angry / excited head shake: 85 -> 95 -> 90
int FACE_SHAKE_HI = 95;
unsigned long FACE_SHAKE_MS = 120;   // dwell at each shake position

// Flip any of these to -1 if that wheel, the steering or the throttle comes
// out backwards on your wiring -- README section 2.7.
int LEFT_INVERT = 1;
int RIGHT_INVERT = 1;
int INVERT_X = 1;
int INVERT_Y = 1;

int DEADZONE = 60;
int MIN_PWM = 40;
int MAX_PWM = 255;      // analogWrite is 8-bit on the UNO R4, so 255 is full duty
int SLEW = 12;
int LOOP_MS = 20;

// CHANGE ME. WPA2 needs 8 characters or more -- with a shorter one WiFiS3's
// beginAP() can bring the network up open, with no password at all.
const char *AP_SSID = "mascot-robot";
const char *AP_PASS = "mascotbot";

// Which serial port the serial link (serial-link.ino) reads commands from.
// Serial is the USB-C port -- on the UNO R4 WiFi a UART bridged to USB by the
// on-board ESP32-S3 -- which is what the tablet and the Arduino Serial Monitor
// use; open it at 115200. A voice module wired to the board's own UART on
// D0/D1 instead wants Serial1: change this one line and re-upload, and
// setup() opens it. The board's own logging always goes to Serial (USB)
// whatever this is set to.
#define VOICE_LINK Serial

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

// Routines (routines.ino). Nothing on this robot measures how far it went --
// there is no encoder on either wheel -- so a routine is open-loop and every
// number below is a stopwatch value for YOUR
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

unsigned long SURPRISED_MS = 300;   // roughly 10 cm straight back, fast (surprised())
int SURPRISED_SPEED = 100;

// The joystick outranks the serial link, and this is how long it keeps the
// wheels after the last time somebody actually pushed the knob. A routine
// requested over serial inside this window is refused outright.
//
// This window is only about STARTING a routine, and it is the whole of the
// joystick priority rather than half of it. Once a routine is running it owns
// the wheels outright: the knob does not interrupt one and neither does a
// /move, and only a deliberate stop -- GET /stop, or "stop" on the serial
// link, both of which call cancelMove() -- ends it early. routines.h sets out
// the whole arrangement.
//
// The window is what makes this a priority rather than a race. Nobody drives
// by deflecting the stick every millisecond: they cross centre, they pause
// between pushes, and letting go sends three zeros. Too short and a serial
// command can jump into a gap mid-drive; too long and "dance" stops working
// for a while after anyone touches the page. A second and a half is a pause,
// not a gap.
//
// Only a deflected knob and the page's STOP button count. The page's idle
// 0,0 heartbeat deliberately does not -- it arrives at 5 Hz for as long as
// anybody has the page open, so counting it would mean serial commands never
// worked with a phone connected.
unsigned long STICK_PRIORITY_MS = 1500;
