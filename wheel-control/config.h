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
