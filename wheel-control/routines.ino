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
//
// Once those pass it also plays the ROUTINE_START_DELAY_MS pause, so every
// routine gets the same beat before it moves. A stop during the pause ends the
// routine here; it is logged as cut short and the caller just gets false.
bool beginRoutine(const char *name) {
  return openRoutine(name, true);
}

// The same gate for a routine that only moves the head. It skips the arming
// check -- no wheel turns, so there is nothing to arm -- and keeps the rest:
// no nesting, the joystick priority, and the start pause. It still holds the
// stick at centre for its length, because it blocks loop() the same way a
// wheel routine does, and centred is the only safe thing for the wheels to be
// doing while nobody can steer them.
bool beginHeadRoutine(const char *name) {
  return openRoutine(name, false);
}

bool openRoutine(const char *name, bool needsWheels) {
  if (inRoutine) {
    logf("routine: %s refused -- one is already running\n", name);
    return false;
  }
  if (stickActive()) {
    logf("routine: %s refused -- the joystick has the wheels\n", name);
    return false;
  }
  if (needsWheels && (!leftEnabled || !rightEnabled)) {
    logf("routine: %s refused -- arm both wheels first\n", name);
    return false;
  }

  inRoutine = true;
  logf("routine: %s\n", name);

  // A short pause before the first step. It is the "stop" move -- the stick
  // held at centre -- rather than a delay(), so the pumps in runMove() keep
  // running through it and a STOP that lands in the pause ends the routine
  // before anything moves.
  if (!pauseFor(ROUTINE_START_DELAY_MS)) {
    endRoutine(name, false);
    return false;
  }
  return true;
}

// Stand still for ms, as a step of the routine: the "stop" move holds the
// stick at centre while runMove() keeps the pumps running, so a stop that
// lands in the pause cancels it like any other step. 0 is no pause at all
// rather than startMove()'s MOVE_DEFAULT_MS.
bool pauseFor(unsigned long ms) {
  return ms == 0 || runMove("stop", ms, 0);
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
//
// It also runs the head shake, when angry() has switched it on: the shake has
// to happen inside these pumps rather than in a delay() loop of its own, or
// STOP could not reach it.
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
    shakeTick(now);
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
// One per emotion, each on the serial link by bare name. surprised(), sad(),
// happy(), passionate() and annoyed() each make one gesture, hold it for
// UNDO_DELAY_MS, and then undo it, so the robot ends where it started with the
// head centred. angry() and excited() are head-only: they move the servo and
// nothing else, have no undo, and go through beginHeadRoutine(), so they work
// with the wheels disarmed.
//
// A stop part-way through ends the routine where it is, undo included, so a
// cancelled emotion leaves the robot wherever the stop caught it.

// The PWM a straight move at speed% settles at: the same deadzone and toPwm()
// steps the control loop runs, so this follows DEADZONE, MIN_PWM and MAX_PWM.
int straightPwm(int speed) {
  int stick = (int)(512L * constrain(speed, 0, 100) / 100);
  return abs(toPwm(applyDeadzone(stick)));
}

// How far a straight move goes, in PWM x control steps -- a proxy for
// distance, not centimetres. The wheels slew up at SLEW per step and back down
// the same way once the hold ends. If they reach the target, the ground lost
// ramping up is exactly what the ramp-down gains, so the answer is target x
// steps; if the hold is too short to reach it (surprised(), passionate()),
// they ramp to a peak and straight back, which comes to peak x steps.
long straightTravel(unsigned long ms, int speed) {
  long steps = (long)(ms / (unsigned long)LOOP_MS);
  long target = straightPwm(speed);
  long peak = (long)max(SLEW, 1) * steps;
  return min(target, peak) * steps;
}

// The hold, at returnSpeed, that covers the same ground as ms at speed --
// straightTravel() solved backwards. This is a model of the motors, not a
// measurement: it assumes ground covered is proportional to PWM, which a real
// gearmotor only roughly obeys near MIN_PWM. If an undo lands short or long,
// UNDO_SPEED is the one knob that moves them all.
unsigned long matchingMs(unsigned long ms, int speed, int returnSpeed) {
  long travel = straightTravel(ms, speed);
  long target = straightPwm(returnSpeed);
  long slewStep = max(SLEW, 1);
  if (target == 0 || travel == 0) {
    return 0;
  }

  long steps;
  if (travel >= target * target / slewStep) {
    steps = travel / target;                          // reaches returnSpeed
  } else {
    steps = (long)sqrt((double)travel / slewStep);    // ramps up and straight back
  }
  return (unsigned long)steps * (unsigned long)LOOP_MS;
}

// The undo for a straight gesture: wait UNDO_DELAY_MS, centre the head, then
// drive the other way at UNDO_SPEED for as long as it takes to cover the
// ground the gesture covered. Every straight emotion returns at the same
// speed; only the length of the return differs.
bool undoStraight(const char *backTo, unsigned long ms, int speed) {
  if (!pauseFor(UNDO_DELAY_MS)) {
    return false;
  }
  setServo(SERVO_START_DEG);

  unsigned long back = matchingMs(ms, speed, UNDO_SPEED);
  if (back > MOVE_MAX_MS) {
    Serial.println(F("undo: return is above MOVE_MAX_MS -- startMove() will cut it short; raise UNDO_SPEED"));
  }
  return back == 0 || runMove(backTo, back, UNDO_SPEED);
}

// Both wheels reverse at full speed for a short hop back, about 10 cm -- the
// "jump back" of a startled robot. Same distance as a NUDGE, in a fraction of
// the time, so SURPRISED_MS is shorter than NUDGE_MS and needs calibrating on
// its own. Then, after the pause, back forward to where it started.
bool surprised() {
  if (!beginRoutine("surprised")) {
    return false;
  }

  bool finished = runMove("backward", SURPRISED_MS, SURPRISED_SPEED)
               && undoStraight("forward", SURPRISED_MS, SURPRISED_SPEED);

  endRoutine("surprised", finished);
  return finished;
}

// A slow back-off -- slower than surprised(), so it reads as shrinking away
// rather than jumping back -- and then forward again to where it started.
bool sad() {
  if (!beginRoutine("sad")) {
    return false;
  }

  bool finished = runMove("backward", SAD_MS, SAD_SPEED)
               && undoStraight("forward", SAD_MS, SAD_SPEED);

  endRoutine("sad", finished);
  return finished;
}

// A slow roll forward, and then back again to where it started.
bool happy() {
  if (!beginRoutine("happy")) {
    return false;
  }

  bool finished = runMove("forward", HAPPY_MS, HAPPY_SPEED)
               && undoStraight("backward", HAPPY_MS, HAPPY_SPEED);

  endRoutine("happy", finished);
  return finished;
}

// Whether the head shake is on, which way the head is in it, and when it last
// switched. Only shakeTick() and angry() touch them.
bool shaking = false;
bool shakeLeft = false;
unsigned long lastShake = 0;

// Called on every pass of runMove()'s loop. While angry() has the shake on,
// flips the head to the other side of centre every ANGRY_SHAKE_STEP_MS;
// otherwise does nothing.
void shakeTick(unsigned long now) {
  if (!shaking || now - lastShake < ANGRY_SHAKE_STEP_MS) {
    return;
  }
  lastShake = now;
  shakeLeft = !shakeLeft;
  setServo(SERVO_START_DEG + (shakeLeft ? ANGRY_SHAKE_DEG : -ANGRY_SHAKE_DEG));
}

// Head-only: the head shakes left and right of centre by ANGRY_SHAKE_DEG with
// no hold at either end, for ANGRY_SHAKE_MS, and then faces front. No undo --
// the wheels never move, and the head is centred at the end whichever way the
// routine ends.
//
// "No hold" still means one servo frame per side, not zero: the servo only
// reads a new angle every 20 ms, so flipping faster than that just averages
// the two out and the head sits still.
bool angry() {
  if (!beginHeadRoutine("angry")) {
    return false;
  }

  shakeLeft = false;
  lastShake = millis() - ANGRY_SHAKE_STEP_MS;   // first flip on the first pass
  shaking = true;
  bool finished = pauseFor(ANGRY_SHAKE_MS);
  shaking = false;
  setServo(SERVO_START_DEG);

  endRoutine("angry", finished);
  return finished;
}

// A fast lunge forward -- happy()'s direction at full speed -- and then back
// again at the same return speed happy() uses.
bool passionate() {
  if (!beginRoutine("passionate")) {
    return false;
  }

  bool finished = runMove("forward", PASSIONATE_MS, PASSIONATE_SPEED)
               && undoStraight("backward", PASSIONATE_MS, PASSIONATE_SPEED);

  endRoutine("passionate", finished);
  return finished;
}

// The body turns right on the spot while the head turns left, so the robot
// ends up looking away. The servo is set before the move starts and puts
// itself there on its own, so the two happen together. It holds that for
// UNDO_DELAY_MS, then the head comes back to centre and the body turns back
// left by the same turn -- the same move mirrored, same ms and speed, so the
// angle matches without any model. The head is centred whichever way the
// routine ends, so a cancelled annoyed() never leaves it turned.
bool annoyed() {
  if (!beginRoutine("annoyed")) {
    return false;
  }

  setServo(ANNOYED_SERVO_DEG);
  bool finished = runMove("spin_right", ANNOYED_MS, ANNOYED_SPEED)
               && pauseFor(UNDO_DELAY_MS);
  setServo(SERVO_START_DEG);
  finished = finished && runMove("spin_left", ANNOYED_MS, ANNOYED_SPEED);

  endRoutine("annoyed", finished);
  return finished;
}

// Head-only: the head turns EXCITED_SERVO_DEG to the right, holds for
// EXCITED_HOLD_MS, swings straight across to the same angle on the left in
// one move -- one setServo(), so it passes centre without stopping there --
// holds again, and then faces front. No undo; the wheels never move. The
// holds are pauseFor(), not delay(), so STOP still gets through, and the head
// is centred whichever way the routine ends.
//
// Left is SERVO_START_DEG + degrees, the same side ANNOYED_SERVO_DEG treats as
// left; if annoyed() turns your head the wrong way, this does too.
bool excited() {
  if (!beginHeadRoutine("excited")) {
    return false;
  }

  setServo(SERVO_START_DEG - EXCITED_SERVO_DEG);       // right
  bool finished = pauseFor(EXCITED_HOLD_MS);
  if (finished) {
    setServo(SERVO_START_DEG + EXCITED_SERVO_DEG);     // across to the left
    finished = pauseFor(EXCITED_HOLD_MS);
  }
  setServo(SERVO_START_DEG);

  endRoutine("excited", finished);
  return finished;
}
