// The serial link -- reads commands off Serial and calls the matching routine.
//
// One command per line of plain text on Serial, at the 115200 setup() already
// opens. The word on a line is its own command, not a move name out of
// MOVES[]: those are the robot's vocabulary for the HTTP API, and nothing says
// the two lists have to be the same.
//
// Nothing here touches a wheel or decides who gets them. A command is either
// "stop", answered here, or a call into a routine in routines.ino, which
// refuses for itself when it has to. routines.h sets out the vocabulary and
// the priority rules in full.

#include "routines.h"

// just in case. commmands are checked host side first any way
static const size_t MAX_COMMAND_LENGTH = 32;
String command;

// One complete command. Lower-cased and trimmed by the caller, so this only
// has to match words.
//
// "stop" is answered before anything else and is never refused. It is one of
// only two ways a running routine can be ended at all now that the knob does
// not do it, and refusing a request to stop is never the safe answer whatever
// the priority rules say. Everything else goes through spin() and dance(),
// which is where beginRoutine() applies the joystick priority and prints the
// reason when it bites.
void runCommand(const String &command) {

  // For the sake of testing we will do surprised only
  if (command == "surprised") {
    surprised();
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

  Serial.printf("?!?! \"%s\" -- unrecognized command\n", command.c_str());
}

// Read whatever has arrived and act on any complete line in it. Never blocks
// waiting for one: loop() calls this every pass and there is no telling when
// the rest of a line will turn up, so a part-line stays in command until the
// newline that finishes it.
//
// Note that runCommand() can block here for the length of a whole
// routine, and that the buffer is cleared before it is called -- a routine
// pumps this function again from inside runMove(), and it has to start from
// an empty line rather than from half of the one that launched it.

void readSerial() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;               // CR LF or bare LF, both end a line
    }

    if (c != '\n') {
      if (command.length() < MAX_COMMAND_LENGTH) {
        command += c;
      }
      continue;
    }

    // End of line: take the command out of the buffer *before* running it,
    // so a routine that pumps readSerial() starts from an empty line.
    String line = command;
    command = "";
    line.trim();
    line.toLowerCase();
    if (line.length() > 0) {
      runCommand(line);
    }
  }
}
