// The serial link -- reads commands off Serial and calls the routine each one
// names. 
//
// One command per line of plain text on Serial, at the 115200 setup() already
// opens. The line the voice system sends is its own word, not a move name out
// of MOVES[]: those are the robot's vocabulary for the HTTP API, and nothing
// says the two lists have to be the same.
//
// Nothing here touches a wheel or decides who gets them. A command is either
// "stop", answered here, or a call into a routine in routines.ino, which
// refuses for itself when it has to. routines.h sets out the vocabulary and
// the priority rules in full.

// TODO: this is a serial reader not a voice reader. update it to be more readable!!
#include "routines.h"

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
