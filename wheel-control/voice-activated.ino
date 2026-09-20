// Voice-activated routines, and the serial link the voice system speaks over.
//
// A routine is a fixed sequence of named moves, played back to back, so
// "dance" is one call instead of six timed requests from whatever heard the
// word. serviceVoice() at the bottom of this tab is the other half: it reads
// one plain-text command per line off Serial and calls the routine for it.
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
//     page's STOP button, and "stop" on the voice link -- plus disarming a
//     wheel, which is hardware and cannot be argued with.
//
// So the knob wins before a routine begins and the routine wins once it has,
// and taking the wheels back from a running one is a deliberate act rather
// than something a stray touch on the knob can do.
//
// The one thing a routine cannot do is measure. There is no encoder on either
// wheel, so "10 cm" and "360 degrees" are stopwatch numbers in config.h, not
// feedback -- see the comment there before trusting either.

#include "voice-activated.h"

// True while spin() or dance() is running. A routine pumps
// server.handleClient() and serviceVoice(), so a request or a spoken word
// arriving mid-routine can call back in here; without this, "dance" heard
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
// page's idle 0,0 heartbeat deliberately does not stamp it, or voice would be
// locked out for as long as anybody had the page open, which is most of the
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
// The joystick check is here rather than in serviceVoice() for the same
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
    Serial.printf("routine: %s refused -- one is already running\n", name);
    return false;
  }
  if (stickActive()) {
    Serial.printf("routine: %s refused -- the joystick has the wheels\n", name);
    return false;
  }
  if (!leftEnabled || !rightEnabled) {
    Serial.printf("routine: %s refused -- arm both wheels first\n", name);
    return false;
  }

  inRoutine = true;
  Serial.printf("routine: %s\n", name);
  return true;
}

void endRoutine(const char *name, bool finished) {
  inRoutine = false;
  if (!finished) {
    Serial.printf("routine: %s cut short\n", name);
  }
}

// One step of a routine: start the move, then run the control loop here until
// it has finished holding and the wheels have ramped back down.
//
// This loop is loop(), deliberately. loop() is not running while a routine
// blocks, and loop() is the only thing that turns webX/webY into PWM, so the
// routine has to take over its jobs for its own length: service HTTP as fast
// as it can, read the voice link, and run one controlStep() per LOOP_MS.
//
// Both pumps earn their place. handleClient() is what lets /stop reach the
// board and cancel a routine that is already moving -- a deflected knob comes
// in through the same call and is deliberately turned away, but the STOP
// button has to get through, and it is the only HTTP route that can end a
// routine. serviceVoice() is the same story for a spoken "stop", besides
// keeping the UART buffer from filling up over a five-second dance. A "spin"
// or "dance" arriving through either pump is refused by the inRoutine guard
// above, so neither can nest.
//
// Returns false for a name that is not in MOVES[], and false if the move was
// cancelled before its hold ran out -- the caller drops the rest of the
// routine in both cases.
bool runMove(const char *name, unsigned long ms, int speed) {
  if (!startMove(String(name), (long)ms, speed)) {
    Serial.printf("routine: no move named \"%s\"\n", name);
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
    serviceVoice();

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

// --- The voice link --------------------------------------------------------
//
// One command per line of plain text on Serial, at the 115200 setup() already
// opens. The line the voice system sends is its own word, not a move name out
// of MOVES[]: those are the robot's vocabulary for the HTTP API, and nothing
// says the two lists have to be the same.

// The line being assembled, and whether it has already run past the length
// any real command has. Dropping an over-long line matters -- truncating it
// instead would let "dancefloor" arrive as "dance". Only this tab's functions
// touch either.
String voiceLine;
bool voiceTooLong = false;

const unsigned int VOICE_MAX_LEN = 32;

// One complete command. Lower-cased and trimmed by the caller, so this only
// has to match words.
//
// "stop" is answered before anything else and is never refused. It is one of
// only two ways a running routine can be ended at all now that the knob does
// not do it, and refusing a request to stop is never the safe answer whatever
// the priority rules say. Everything else goes through spin() and dance(),
// which is where beginRoutine() applies the joystick priority and prints the
// reason when it bites.
void runVoiceCommand(const String &command) {
  if (command == "stop") {
    cancelMove();
    webX = 0;
    webY = 0;
    lastCommand = millis();
    Serial.println(F("voice: stop"));
    return;
  }

  if (command == "spin") {
    spin();
    return;
  }

  if (command == "dance") {
    dance();
    return;
  }

  Serial.printf("voice: ignored \"%s\" -- say spin, dance or stop\n", command.c_str());
}

// Read whatever has arrived and act on any complete line in it. Never blocks
// waiting for one: loop() calls this every pass and there is no telling when
// the rest of a line will turn up, so a part-line stays in voiceLine until the
// newline that finishes it.
//
// Note that runVoiceCommand() can block here for the length of a whole
// routine, and that the buffer is cleared before it is called -- a routine
// pumps this function again from inside runMove(), and it has to start from
// an empty line rather than from half of the one that launched it.
void serviceVoice() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;               // CR LF or bare LF, both end a line
    }

    if (c != '\n') {
      if (voiceLine.length() >= VOICE_MAX_LEN) {
        voiceTooLong = true;
      } else {
        voiceLine += c;
      }
      continue;
    }

    String command = voiceLine;
    bool tooLong = voiceTooLong;
    voiceLine = "";
    voiceTooLong = false;

    if (tooLong) {
      Serial.println(F("voice: line too long -- dropped"));
      continue;
    }

    command.trim();
    command.toLowerCase();
    if (command.length() > 0) {
      runVoiceCommand(command);
    }
  }
}
