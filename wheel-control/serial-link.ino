// The serial link -- reads commands off VOICE_LINK and calls the matching routine.
//
// One command per line of plain text on VOICE_LINK (config.h; Serial as
// shipped), at the 115200 setup() opens. The word on a line is its own
// command, not a move name out of MOVES[]: those are the robot's vocabulary
// for the HTTP API, and nothing says the two lists have to be the same.
//
// On the UNO R4 WiFi, Serial is not native USB. The USB-C socket belongs to
// the on-board ESP32-S3, which bridges it to one of the RA4M1's UARTs, so
// Serial is a real UART and the baud is real: whatever opens the port -- the
// tablet, the Serial Monitor -- has to open it at 115200 too. Serial1 (D0/D1)
// is a plain UART as well. Both are read the same way below, a byte at a time
// with available()/read(); the R4 core has no Serial.printf(), so anything
// formatted goes out through logf() (wheel-control.ino).
//
// Nothing here touches a wheel or decides who gets them. A command is either
// "stop", answered here, or a call into a routine in routines.ino, which
// refuses for itself when it has to. routines.h sets out the vocabulary and
// the priority rules in full.

#include "routines.h"

// Long enough for any command, "face:passionate" included. A longer line is
// dropped whole rather than truncated -- see readSerial().
static const size_t MAX_COMMAND_LENGTH = 32;

// The line being assembled, and whether it has already run past
// MAX_COMMAND_LENGTH. A fixed buffer rather than a String: this is read every
// loop pass, and the RA4M1 has 32 KB of RAM to fragment. Only this tab's
// functions touch either.
static char lineBuf[MAX_COMMAND_LENGTH + 1];
static size_t lineLen = 0;
static bool lineTooLong = false;

// One complete command. Lower-cased and trimmed by the caller, so this only
// has to match words.
//
// "stop" is answered before anything else and is never refused. It is one of
// only two ways a running routine can be ended at all now that the knob does
// not do it, and refusing a request to stop is never the safe answer whatever
// the priority rules say. It does what GET /stop does to the stick, but does
// not stamp lastStickInput: the joystick priority is about the person at the
// page, and this is not them. Everything else goes through the routines,
// which is where beginRoutine() applies the joystick priority and prints the
// reason when it bites.
void runCommand(const String &cmd) {
  if (cmd == "stop") {
    cancelMove();
    webX = 0;
    webY = 0;
    lastCommand = millis();
    Serial.println(F("serial: stop"));
    return;
  }

  if (cmd == "spin") {
    spin();
    return;
  }

  if (cmd == "dance") {
    dance();
    return;
  }

  if (cmd == "surprised") {
    surprised();
    return;
  }

  if (cmd == "excited") {
    excited();
    return;
  }

  // The tablet sends every expression change as its bare label (RobotLink.kt),
  // so the rest of the vocabulary arrives here too. These are placeholders
  // today; "idle" is the resting face and needs nothing.
  if (cmd == "sad")        { sad();        return; }
  if (cmd == "happy")      { happy();      return; }
  if (cmd == "angry")      { angry();      return; }
  if (cmd == "passionate") { passionate(); return; }
  if (cmd == "annoyed")    { annoyed();    return; }
  if (cmd == "idle")       { return; }

  /*
  // "face:<expr>" -- the tablet's expression changes, same gestures as
  // GET /face (faces.ino).
  if (cmd.startsWith("face:")) {
    String name = cmd.substring(5);
    if (runFace(name)) {
      logf("face: %s\n", name.c_str());
    } else {
      logf("serial: unknown face \"%s\"\n", name.c_str());
    }
    return;
  }
*/
  logf("serial: \"%s\" -- unrecognized command\n", cmd.c_str());
}

// How long a part-line may sit with nothing more arriving before it is taken
// as the whole command. The tablet ends every word with '\n', so this only
// matters for a sender that sends no line ending at all -- the Serial Monitor
// on "No line ending", say. At 115200 a word arrives in well under a
// millisecond, so a gap this long means the sender has finished.
static const unsigned long LINE_IDLE_MS = 50;

// When the last byte arrived, for LINE_IDLE_MS.
static unsigned long lastByteAt = 0;

// Take the line out of the buffer, finish cleaning it and hand it to
// runCommand(). The buffer is emptied *before* runCommand() is called -- a
// routine pumps readSerial() again from inside runMove(), and it has to start
// from an empty line rather than from half of the one that launched it.
static void finishLine() {
  lineBuf[lineLen] = '\0';
  String line(lineBuf);
  bool tooLong = lineTooLong;
  lineLen = 0;
  lineTooLong = false;

  if (tooLong) {
    Serial.println(F("serial: line too long -- dropped"));
    return;
  }

  line.trim();
  line.toLowerCase();
  if (line.length() == 0) {
    return;
  }

  // A command is one word. "spin now" is not "spin" -- run neither half.
  if (line.indexOf(' ') >= 0) {
    logf("serial: \"%s\" -- not one word, dropped\n", line.c_str());
    return;
  }

  runCommand(line);
}

// Read whatever has arrived and act on any complete command in it. Never
// blocks waiting for one: loop() calls this every pass and there is no telling
// when the rest of a word will turn up, so a part-word stays in lineBuf until
// a line ending, or LINE_IDLE_MS of silence, finishes it.
//
// Either '\n' or '\r' ends a line, so the Serial Monitor works on any of its
// line-ending settings; the empty line a CR LF pair leaves behind is skipped.
// Cleaning happens a byte at a time: tabs count as spaces, leading whitespace
// is never stored, and any other control byte or non-ASCII byte (a UTF-8 BOM,
// noise from the bridge at boot) is thrown away. What is left is trimmed,
// lower-cased and must be a single word. A line longer than
// MAX_COMMAND_LENGTH is dropped whole -- truncating it could turn a long word
// into a short command.
//
// Note that runCommand() can block here for the length of a whole routine.
void readSerial() {
  while (VOICE_LINK.available() > 0) {
    int c = VOICE_LINK.read();
    if (c < 0) {
      break;
    }
    lastByteAt = millis();

    if (c == '\n' || c == '\r') {
      finishLine();
      continue;
    }

    if (c == ' ' || c == '\t') {
      if (lineLen == 0) {
        continue;
      }
      c = ' ';
    } else if (c < 0x21 || c > 0x7e) {
      continue;
    }

    if (lineLen < MAX_COMMAND_LENGTH) {
      lineBuf[lineLen++] = (char)c;
    } else {
      lineTooLong = true;
    }
  }

  // A word with no line ending after it: once the sender has gone quiet, it
  // is the whole command.
  if (lineLen > 0 && millis() - lastByteAt >= LINE_IDLE_MS) {
    finishLine();
  }
}
