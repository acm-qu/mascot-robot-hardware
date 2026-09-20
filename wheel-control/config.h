// Every number you might need to change lives here: pins, tuning, the Wi-Fi
// network and the safety limits. Nothing else in the sketch defines any of
// them, and only wheel-control.ino includes this file.
#pragma once

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

int LEFT_INVERT = 1;
int RIGHT_INVERT = -1;
int INVERT_X = 1;
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

// Both wheels come up gated off, so a reset or a fresh upload cannot drive
// anything until someone deliberately arms it in the page. Set true for the
// old behaviour of being live the moment the board boots.
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
