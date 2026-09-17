// Named moves. A move is a stick position the board holds by itself for a
// while, so "spin right for half a second" is one request instead of a stream
// of /drive commands -- which is what a tablet app, or a curl, wants to send.
//
// A move is nothing more than a value in webX/webY, so it goes through exactly
// the same pipeline as the knob: inversion, deadzone, arcade mix, PWM mapping,
// slew and the per-wheel enables all apply, and a disarmed wheel stays dead.
// The one thing that differs is who keeps the link failsafe fed -- see
// serviceMove().

// To add a move, add a row. x and y are stick positions, -512..512, in the
// same frame the page sends: +y is forward, +x steers right.
struct Move {
  const char *name;
  int x;
  int y;
};

const Move MOVES[] = {
  { "forward",        0,  512 },
  { "backward",       0, -512 },
  { "spin_left",   -512,    0 },   // wheels opposed: turns on the spot
  { "spin_right",   512,    0 },
  { "turn_left",   -360,  512 },   // inside wheel nearly stops: a tight turn
  { "turn_right",   360,  512 },
  { "slight_left", -150,  512 },   // a gentle arc, both wheels still driving
  { "slight_right", 150,  512 },
  { "stop",           0,    0 },   // centre the stick and hold it there
};

const int MOVE_COUNT = sizeof(MOVES) / sizeof(MOVES[0]);

// millis() at which the running move stops holding the stick, 0 when no move
// is running. Only the functions in this tab touch it.
unsigned long moveUntil = 0;

// Index into MOVES, or -1. Names are case-sensitive: the table is the spec.
int findMove(const String &name) {
  for (int i = 0; i < MOVE_COUNT; i++) {
    if (name == MOVES[i].name) {
      return i;
    }
  }
  return -1;
}

// Start holding a move, replacing one already running. ms <= 0 means
// MOVE_DEFAULT_MS, and nothing can ask for more than MOVE_MAX_MS: for the
// length of the hold the board is its own commander, so this clamp is what
// bounds a robot driving with nobody steering. speed is a percent of full
// deflection. Returns false for a name that is not in the table, and then
// changes nothing.
bool startMove(const String &name, long ms, int speed) {
  int i = findMove(name);
  if (i < 0) {
    return false;
  }

  if (ms <= 0) {
    ms = MOVE_DEFAULT_MS;
  }
  unsigned long hold = min((unsigned long)ms, MOVE_MAX_MS);
  speed = constrain(speed, 0, 100);

  webX = (int)((long)MOVES[i].x * speed / 100);
  webY = (int)((long)MOVES[i].y * speed / 100);
  lastCommand = millis();
  moveUntil = millis() + hold;

  Serial.printf("move: %s for %lu ms at %d%%\n", MOVES[i].name, hold, speed);
  return true;
}

// Called once per control step, before the failsafe check. While the hold
// lasts this stamps lastCommand, which is what stops the link failsafe from
// cutting a move at 500 ms: for that long the board is the commander, not the
// browser. When the hold ends the stick is centred and the wheels ramp down
// through slew() like a released knob -- and the stamping continues until
// they are at rest, so a designed stop is never cut short and logged as a
// lost link, whatever SLEW is set to.
void serviceMove(unsigned long now) {
  if (moveUntil == 0) {
    return;
  }

  if ((long)(now - moveUntil) < 0) {   // still holding; wrap-safe
    lastCommand = now;
    return;
  }

  webX = 0;
  webY = 0;
  if (leftSpeed != 0 || rightSpeed != 0) {
    lastCommand = now;
    return;
  }

  moveUntil = 0;
}

// Drop the running move, if any, without touching the stick. The caller
// decides what the stick does next: handleDrive() writes the knob into it,
// handleStop() zeroes it.
void cancelMove() {
  moveUntil = 0;
}

bool moveRunning() {
  return moveUntil != 0;
}

// Comma-separated names, for GET /moves and for the 400 an unknown name gets.
String listMoves() {
  String names;
  for (int i = 0; i < MOVE_COUNT; i++) {
    if (i > 0) {
      names += ",";
    }
    names += MOVES[i].name;
  }
  return names;
}
