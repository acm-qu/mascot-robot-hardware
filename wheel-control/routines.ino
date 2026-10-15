// Routines -- the sequences of moves the robot can play back on its own.
//
// A routine is a fixed sequence of one or more of the named moves in MOVES[]
// (moves.ino), played back to back -- and only those, never a raw stick
// position -- so "dance" is one call instead of six timed requests from
// whatever sent the command. readSerial() in serial-link.ino is the other
// half: it reads one plain-text command per line off Serial and calls the
// routine for it.
//
// Nothing here talks to a wheel. Every step goes through moves.ino --
// runMove() hands a name to startMove() and then runs the control loop itself
// until that move is done -- so a routine gets the whole of the normal
// pipeline (inversion, deadzone, arcade mix, PWM, slew, the per-wheel
// enables), the MOVE_MAX_MS cap on every step, the serial log line, and the
// slew ramp-down at the end of each step.
//
// THE TWO PRIORITY RULES POINT IN OPPOSITE DIRECTIONS, on purpose:
//
//   - while somebody is driving, a routine cannot START. handleDrive() stamps
//     the time whenever the knob is actually deflected, and for
//     STICK_PRIORITY_MS afterwards stickActive() has beginRoutine() refuse.
//   - once a routine HAS started it owns the wheels. A deflected knob no
//     longer interrupts it, and neither does a /move: both are turned away in
//     web.ino by routineRunning(). The only things that end a routine early
//     are the ones that call cancelMove() outright -- GET /stop, which is the
//     page's STOP button, and "stop" on the serial link -- plus disarming a
//     wheel, which is hardware and cannot be argued with.
//
// So the knob wins before a routine begins and the routine wins once it has,
// and taking the wheels back from a running one is a deliberate act rather
// than something a stray touch on the knob can do.
//
// The one thing a routine cannot do is measure. There is no encoder on either
// wheel, so "10 cm" and "360 degrees" are stopwatch numbers in config.h, not
// feedback -- see the comment there before trusting either.

#include "routines.h"

// True while a routine is running. A routine pumps
// server.handleClient() and readSerial(), so a request or a serial command
// arriving mid-routine can call back in here; without this, "dance" received
// twice would nest one routine inside the other and the two would fight over
// the stick. Only this tab writes it.
bool inRoutine = false;

// Whether a routine currently owns the wheels. web.ino asks through this
// rather than reading inRoutine directly, for the same reason moves.ino has
// moveRunning() rather than exposing moveUntil: the build gives every function
// a prototype, so a function can be called from any tab, while a variable can
// only be read by a tab that sorts after the one defining it. This tab happens
// to sort before web.ino today; renaming a file should not be able to break
// the build.
bool routineRunning() {
  return inRoutine;
}

// Whether the joystick currently has the wheels. lastStickInput is stamped by
// handleDrive() when the knob is actually deflected, and by handleStop(); the
// page's idle 0,0 heartbeat deliberately does not stamp it, or serial commands
// would be locked out for as long as anybody had the page open, which is most of the
// time.
//
// This decides whether a routine may START, and nothing else -- a routine
// already running is not subject to it, or to anything else the knob does.
// STICK_PRIORITY_MS is what makes it a priority and not a race; config.h says
// why the window has to be there at all. The lastStickInput == 0 test is for
// the first seconds after boot, when millis() is small enough that the
// subtraction would otherwise read as "somebody just drove".
bool stickActive() {
  if (lastStickInput == 0) {
    return false;
  }
  return (millis() - lastStickInput) <= STICK_PRIORITY_MS;
}

// Refuse, and say why on serial, unless the board can actually perform a
// routine right now.
//
// The joystick check is here rather than in readSerial() for the same
// reason startMove() holds the MOVE_MAX_MS cap rather than handleMove() does:
// put it at the gate every caller has to come through, and a /spin route
// added later inherits it instead of having to remember it.
//
// Both wheels have to be armed: every step drives both of them, and a routine
// with one wheel dead is not a smaller version of itself. Arming is the
// operator's deliberate choice (README section 9), so this refuses rather
// than arming anything itself.
bool beginRoutine(const char *name) {
  if (inRoutine) {
    logf("routine: %s refused -- one is already running\n", name);
    return false;
  }
  if (stickActive()) {
    logf("routine: %s refused -- the joystick has the wheels\n", name);
    return false;
  }
  if (!leftEnabled || !rightEnabled) {
    logf("routine: %s refused -- arm both wheels first\n", name);
    return false;
  }

  inRoutine = true;
  logf("routine: %s\n", name);
  return true;
}

void endRoutine(const char *name, bool finished) {
  inRoutine = false;
  if (!finished) {
    logf("routine: %s cut short\n", name);
  }
}

// One step of a routine: start the move, then run the control loop here until
// it has finished holding and the wheels have ramped back down.
//
// This loop is loop(), deliberately. loop() is not running while a routine
// blocks, and loop() is the only thing that turns webX/webY into PWM, so the
// routine has to take over its jobs for its own length: service HTTP as fast
// as it can, read the serial link, and run one controlStep() per LOOP_MS.
//
// Both pumps earn their place. handleClient() is what lets /stop reach the
// board and cancel a routine that is already moving -- a deflected knob comes
// in through the same call and is deliberately turned away, but the STOP
// button has to get through, and it is the only HTTP route that can end a
// routine. readSerial() is the same story for a serial "stop", besides
// keeping the UART buffer from filling up over a five-second dance. A "spin"
// or "dance" arriving through either pump is refused by the inRoutine guard
// above, so neither can nest.
//
// Returns false for a name that is not in MOVES[], and false if the move was
// cancelled before its hold ran out -- the caller drops the rest of the
// routine in both cases.
bool runMove(const char *name, unsigned long ms, int speed) {
  if (!startMove(String(name), (long)ms, speed)) {
    logf("routine: no move named \"%s\"\n", name);
    return false;
  }

  // startMove() has already applied MOVE_DEFAULT_MS and the MOVE_MAX_MS cap,
  // so moveUntil is the deadline actually granted, not the one asked for.
  // Reaching it means the hold ran out; leaving the loop before it means
  // something cancelled us.
  unsigned long deadline = moveUntil;

  // serviceMove() keeps the move alive past the deadline until both wheels
  // are back at rest, which takes MAX_PWM / SLEW control steps at worst. The
  // bound is only here so that a ramp that somehow never lands ends the step
  // instead of hanging the sketch.
  unsigned long rampSteps = (unsigned long)(MAX_PWM / max(SLEW, 1) + 2);
  unsigned long giveUp = deadline + rampSteps * (unsigned long)LOOP_MS + 200;

  while (moveRunning()) {
    server.handleClient();
    readSerial();

    unsigned long now = millis();
    if ((long)(now - giveUp) >= 0) {   // wrap-safe, as everywhere else
      Serial.println(F("routine: step overran its ramp -- dropping it"));
      cancelMove();
      break;
    }
    if (now - lastStep < (unsigned long)LOOP_MS) {
      continue;
    }
    lastStep = now;
    controlStep(now);
  }

  // Two ways out, and only one of them is the step finishing. serviceMove()
  // clears the move once the hold has run out AND both wheels are back at
  // rest, so that pair of conditions is what "finished" means. Anything else
  // -- cleared early, or cleared with the wheels still turning, which is a
  // cancel landing in the ramp-down -- is somebody stopping the routine.
  return (long)(millis() - deadline) >= 0 && leftSpeed == 0 && rightSpeed == 0;
}

// The turn itself, without the routine guard, so that dance() can end on
// exactly the turn spin() performs: one definition of "a full turn", one
// number to calibrate.
//
// spin_left is x = -512, which the arcade mix turns into the left wheel back
// and the right wheel forward -- counter-clockwise seen from above.
bool fullSpin() {
  if (SPIN_360_MS > MOVE_MAX_MS) {
    Serial.println(F("spin: SPIN_360_MS is above MOVE_MAX_MS -- startMove() will cut the turn short"));
  }
  return runMove("spin_left", SPIN_360_MS, SPIN_SPEED);
}

bool spin() {
  if (!beginRoutine("spin")) {
    return false;
  }

  bool finished = fullSpin();

  endRoutine("spin", finished);
  return finished;
}

// Back, forward, back -- each about NUDGE_MS worth of travel -- then a small
// turn right and the same turn back left, which leaves the robot on its
// starting heading, and then the full turn.
//
// && is doing real work here: a cancelled step returns false and short-
// circuits the rest, so a stop part-way through a dance ends the dance rather
// than just the step it landed in.
bool dance() {
  if (!beginRoutine("dance")) {
    return false;
  }

  bool finished = runMove("backward",   NUDGE_MS,  NUDGE_SPEED)
               && runMove("forward",    NUDGE_MS,  NUDGE_SPEED)
               && runMove("backward",   NUDGE_MS,  NUDGE_SPEED)
               && runMove("spin_right", WIGGLE_MS, WIGGLE_SPEED)
               && runMove("spin_left",  WIGGLE_MS, WIGGLE_SPEED)
               && fullSpin();

  endRoutine("dance", finished);
  return finished;
}

// --- Emotion routines ------------------------------------------------------
//
// Placeholders, one per emotion, with no moves yet: each returns true
// straight away and never touches a wheel. To fill one in, follow dance() --
// beginRoutine(), an && chain of runMove() calls on names from MOVES[], then
// endRoutine(). Only surprised() is on the serial link so far.

// Both wheels reverse at full speed for a short hop back, about 10 cm -- the
// "jump back" of a startled robot. Same distance as a NUDGE, in a fraction of
// the time, so SURPRISED_MS is shorter than NUDGE_MS and needs calibrating on
// its own.
bool surprised() {
  if (!beginRoutine("surprised")) {
    return false;
  }

  bool finished = runMove("backward", SURPRISED_MS, SURPRISED_SPEED);

  endRoutine("surprised", finished);
  return finished;
}

bool sad() {
  return true;
}

bool happy() {
  return true;
}

bool angry() {
  return true;
}

bool passionate() {
  return true;
}

bool annoyed() {
  return true;
}

bool excited() {
  if (!beginRoutine("excited")) {
    return false;
  }

  setServo(SERVO_START_DEG + 25);
  delay(FACE_SHAKE_MS);
  setServo(SERVO_START_DEG - 25);
  delay(FACE_SHAKE_MS);
  setServo(SERVO_START_DEG);

  endRoutine("excited", true);
  return true;
}
