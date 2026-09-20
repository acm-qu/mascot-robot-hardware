// Voice-activated routines -- the two moves a spoken command asks for, the
// serial link the voice system sends them over, and the rules that decide who
// gets the wheels when the joystick and the voice system both want them.
//
// The definitions are in voice-activated.ino; the numbers that decide how far,
// how long and how long the joystick holds priority are in config.h. This
// header is the reference for whoever wires up the microphone: the one place
// that says what a routine is, what calling one costs, and what can refuse or
// stop it.
//
// The Arduino build concatenates every .ino in the folder and inserts a
// prototype for each function, so no tab has to include this for the sketch to
// compile -- voice-activated.ino includes it anyway, which is what makes these
// declarations the ones that have to agree with the definitions. The helpers
// beside them in that tab (runMove(), beginRoutine(), fullSpin(),
// stickActive(), runVoiceCommand()) are deliberately not here: they are
// internal, and the tab model already lets any tab call them if it must.
// routineRunning() is the exception web.ino needs, and it is declared below.
#pragma once

#include <Arduino.h>


// === How the voice system talks to the board ===============================
//
// One command per line of plain text on Serial, at the 115200 setup() already
// opens. Either line ending works. serviceVoice() reads it, trims it,
// lower-cases it and matches it against three words:
//
//   spin    one full turn on the spot, counter-clockwise
//   dance   the six-step routine below
//   stop    cancel whatever is running and centre the stick
//
// Anything else is ignored, with a line on serial saying so. Lines longer than
// 32 characters are dropped whole rather than truncated -- truncating would
// let "dancefloor" arrive as "dance".
//
// That vocabulary is this tab's own. It is deliberately not the MOVES[] table
// from moves.ino: those names are the HTTP API's vocabulary, they are
// case-sensitive, and nothing says a thing a person says out loud has to be
// one of them.
//
// Two practical notes. The board's own logging goes out of the same UART, so
// whatever is listening at the other end has to tolerate lines it did not ask
// for -- every routine reports itself there. And the Arduino Serial Monitor is
// a complete test harness for all of this: set its line ending to Newline,
// type dance, press enter.


// === Who gets the wheels ===================================================
//
// There are two rules and they point in opposite directions, so it is worth
// being clear about which one is acting.
//
// BEFORE A ROUTINE STARTS, THE JOYSTICK WINS. handleDrive() stamps the time
// whenever the knob is actually deflected, and for STICK_PRIORITY_MS
// afterwards -- 1.5 s as shipped -- beginRoutine() refuses and says so on
// serial. The page's STOP button stamps it too. So a spoken "dance" that
// arrives while somebody is driving is simply dropped, and the driving
// continues uninterrupted.
//
// ONCE A ROUTINE IS RUNNING, THE ROUTINE WINS. A deflected knob does not
// interrupt it -- handleDrive() answers with the status line and changes
// nothing at all -- and neither does /move, which gets a 409. The routine
// keeps the wheels until it finishes or until somebody stops it deliberately:
//
//   - GET /stop, which is the STOP button on the page;
//   - "stop" on the voice link;
//   - GET /enable?l=0&r=0, which gates the bridges off in hardware and does
//     not care what the software thinks.
//
// The first two are both cancelMove(), which is the one door left open, and
// they reach the board because a routine keeps pumping handleClient() and
// serviceVoice() while it blocks. Note what this costs: the knob is no longer
// an emergency stop during a routine. The STOP button is, and it is the only
// HTTP route that can end one.
//
// What does NOT count as driving, for the first rule, is the page's idle 0,0
// heartbeat. It arrives at 5 Hz for as long as anybody has the page open, so
// counting it would mean the voice system never worked with a phone connected.
// The window exists for the same reason: somebody driving crosses centre,
// pauses between pushes, and sends three zeros on release, and none of those
// gaps should be a chance for a routine to jump in.
//
// A deflected knob still stamps the priority while a routine runs, even though
// it is ignored, so the moment a routine ends the person who was pushing the
// knob already outranks whatever the voice system says next.
//
// "stop" is the one voice command neither rule applies to. The joystick
// outranking the voice system is about who gets to MOVE the robot; there is no
// reading of that which says a request to stop should be ignored.


// === What a routine is =====================================================
//
// A routine is a fixed sequence of the named moves in moves.ino, played back
// to back, so "dance" is one call instead of six timed requests from whatever
// heard the word.
//
// Nothing in this tab touches a wheel. Each step is a real move, started with
// startMove(), so a routine inherits everything a /move request gets:
//
//   - the whole control pipeline -- INVERT_X/INVERT_Y, the deadzone, the
//     arcade mix, the normalise step, MIN_PWM/MAX_PWM, slew (README s2);
//   - the per-wheel enables: a disarmed wheel is dead in hardware for a
//     routine exactly as it is for the knob;
//   - the MOVE_MAX_MS cap on every single step;
//   - one serial line per step, from startMove() -- "move: spin_left for
//     1800 ms at 70%";
//   - the slew ramp-down at the end of each step, with the link failsafe held
//     off until both wheels are actually at rest.
//
// What a routine adds on top is the sequencing, a guard against two routines
// running at once, the priority rules above, and the blocking below.


// === Calling one blocks ====================================================
//
// spin() and dance() do not return until the routine is over. That is not an
// oversight and it cannot be avoided cheaply: loop() is the only thing that
// turns webX/webY into PWM, and loop() is not running while a routine is, so
// the routine runs loop()'s control step itself -- controlStep(), one call per
// LOOP_MS -- for its whole length.
//
// server.handleClient() and serviceVoice() are pumped in the same inner loop,
// so the page still loads, the HTTP API still answers, and a stop from either
// side can still get in. Everything else on the board waits.
//
// With the defaults in config.h:
//
//   spin()    1800 ms of holding, plus one ramp-down          ~2.1 s
//   dance()   3 x 400 + 2 x 250 + 1800 = 3500 ms of holding,
//             plus a ramp-down after each of the six steps    ~5 s
//
// Retune the constants and both numbers move -- the rule is the sum of the
// holds plus one slew ramp per step. Those are also the lengths somebody has
// to wait out if they take hold of the knob mid-routine and do not want to
// reach for STOP.


// === What stops a routine ==================================================
//
// Only a deliberate stop, and there are three:
//
//   - GET /stop, the page's STOP button;
//   - "stop" on the voice link;
//   - disarming a wheel with GET /enable, which stops that wheel in hardware.
//     The routine itself keeps running and ends at its own pace, because the
//     enables gate the bridges rather than the sequence.
//
// The first two call cancelMove(), which ends the step the routine is in;
// runMove() returns false, and the && chain in dance() drops every step after
// it, so one stop ends the whole routine rather than just the step it landed
// in.
//
// Everything else is refused for the routine's length: a deflected knob, the
// idle heartbeat, /move, and a second spin() or dance(). A routine never drops
// EN and never arms a wheel -- arming stays the operator's explicit choice
// (README section 9) -- so it cannot arm the robot, and losing an arm does not
// abort it.


// === Return value ==========================================================
//
// true   the routine ran to the end and both wheels are back at rest.
// false  one of:
//          - the joystick had the wheels (this one did not start);
//          - a routine was already running (this one did not start);
//          - either wheel was disarmed (this one did not start);
//          - a stop ended it part-way through.
//
// Each case prints a line on serial saying which, so false is a signal to the
// caller rather than an error it has to diagnose:
//
//   routine: dance
//   routine: dance refused -- the joystick has the wheels
//   routine: dance refused -- one is already running
//   routine: dance refused -- arm both wheels first
//   routine: dance cut short
//
// A false from a refusal means nothing moved at all. A false from a stop means
// the robot is wherever the stop left it, with the wheels centred -- so a
// caller that simply ignores the return value still behaves correctly.


// === There is no feedback ==================================================
//
// Neither wheel has an encoder, so a routine is open-loop: it holds a stick
// position for a measured length of time and trusts that the robot moved. The
// distances and the angle live in config.h and are stopwatch values for one
// robot on one floor at one battery voltage -- the defaults shipped there are
// starting guesses, not measurements, and config.h has the calibration recipe.
//
//   SPIN_360_MS   SPIN_SPEED     one full turn on the spot
//   NUDGE_MS      NUDGE_SPEED    roughly 10 cm forward or back
//   WIGGLE_MS     WIGGLE_SPEED   a small turn on the spot, either way
//
// Two traps worth knowing before you tune. Distance is not proportional to
// time at these lengths, because SLEW spends the first ~0.4 s of every step
// ramping the wheels up -- doubling ms rather more than doubles the distance.
// And SPIN_360_MS has to stay at or below MOVE_MAX_MS: startMove() caps every
// hold there silently, so a spin asking for longer just stops short of a full
// turn. spin() warns on serial if you have set it that way.


// === Safety ================================================================
//
// A routine is the longest this board drives with nobody steering it, and it
// now ignores the knob while it does, so it is worth being exact about what is
// left holding it.
//
// The link failsafe is suspended for the routine's length, step by step: each
// step is a move, and serviceMove() stamps lastCommand every control step
// while its hold lasts (README section 9). MOVE_MAX_MS caps each STEP at 3 s,
// not the routine -- six steps is six holds, so with the defaults a dance is
// about five seconds of driving that COMMAND_TIMEOUT_MS will not interrupt and
// the joystick will not interrupt either.
//
// What still holds in that window, in the order you would reach for them:
//
//   - the STOP button on the page, and "stop" on the voice link. Both are
//     software and both depend on the routine's pumps still running.
//   - the per-wheel enables, which gate the bridges in hardware. Disarming
//     works whatever the sketch is doing.
//   - the physical cutoff on the motor supply, which is the only real kill
//     switch and is unaffected by any of this.
//
// Put the chassis on blocks the first time you run either routine. An
// uncalibrated SPIN_360_MS is a robot turning for as long as you told it to,
// and the knob will not talk it out of it.


// Whether a routine currently owns the wheels. web.ino asks before letting a
// /drive or a /move touch the stick; anything else that starts commanding the
// robot should ask too.
bool routineRunning();

// Read the voice link and run any complete command on it. Called from loop()
// every pass, and again from inside a running routine. Returns as soon as
// there is nothing left to read, EXCEPT that a command it recognises runs
// there and then -- so this call blocks for the length of a routine the same
// way spin() and dance() do.
void serviceVoice();

// One full 360-degree turn on the spot, counter-clockwise. Counter-clockwise
// because it is the spin_left move: x = -512 through the arcade mix is the
// left wheel back and the right wheel forward, which is counter-clockwise seen
// from above.
bool spin();

// Six steps: about 10 cm back, the same forward, the same back again, a small
// turn right, the same turn back left (so the heading comes out where it went
// in), and then the same full turn spin() performs.
bool dance();
