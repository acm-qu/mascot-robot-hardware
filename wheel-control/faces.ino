// No 💖

/*

// Faces: ACMO's body language. The Kotlin app calls GET /face?name=<expr>
// whenever its on-screen expression changes -- the first time it shows one, or
// any switch to another -- and this tab turns that into a small movement and/or
// a servo gesture. One function per expression in the app's Expression enum
// (idle, surprised, sad, happy, angry, passionate, annoyed, excited), plus a
// default resting pose.
//
// The nudges go through startMove() (moves.ino), so they inherit everything a
// /move gets: the per-wheel arm gate -- nothing moves until the wheels are
// armed -- the failsafe feeding while a move runs, and the MOVE_MAX_MS cap.
// They are deliberately tiny; FACE_NUDGE_MS / *_SPEED live in config.h.
//
// Faces arrive fast -- a talking ACMO can change expression every sentence --
// so any face that would MOVE is skipped while a move is still running
// (moveRunning() is also true mid-routine). That is what stops nudges stacking
// and, more importantly, stops two blocking head shakes running back to back
// and overrunning the link failsafe. The servo-only faces are cheap and always
// act.

// Servo 85 -> 95 -> 90: a quick shake of the head. Blocking, but short and well
// under COMMAND_TIMEOUT_MS. The caller runs startMove() FIRST, so lastCommand is
// freshly stamped before this blocks and the failsafe cannot fire during it.
static void shakeHead() {
  setServo(FACE_SHAKE_LO);
  delay(FACE_SHAKE_MS);
  setServo(FACE_SHAKE_HI);
  delay(FACE_SHAKE_MS);
  setServo(90);
}

void faceDefault() {
  setServo(90);
}

void faceIdle() {
  setServo(90);   // nothing else
}

void faceSurprised() {
  setServo(90);   // nothing else
}

void faceSad() {
  if (moveRunning()) return;
  setServo(90);
  startMove("backward", FACE_NUDGE_MS, FACE_SAD_SPEED);   // slowly, a tiny bit
}

void faceHappy() {
  if (moveRunning()) return;
  setServo(90);
  startMove("forward", FACE_NUDGE_MS, FACE_NUDGE_SPEED);   // a tiny bit
}

void faceAngry() {
  if (moveRunning()) return;
  startMove("backward", FACE_NUDGE_MS, FACE_NUDGE_SPEED);   // first: stamps lastCommand
  shakeHead();                                              // then the blocking shake
}

void facePassionate() {
  if (moveRunning()) return;
  setServo(90);
  startMove("forward", FACE_NUDGE_MS, FACE_NUDGE_SPEED);   // a tiny bit
}

void faceAnnoyed() {
  setServo(FACE_ANNOYED_SERVO);   // 90 -> 120
}

void faceExcited() {
  faceAngry();   // same as angry
}

// Run the gesture for a lower-cased expression name. Returns false for a name
// that is not one of ACMO's expressions. Both transports come through here: the
// USB serial link (a "face:<name>" line, the way the tablet drives it) and the
// /face HTTP route.
bool runFace(const String &name) {
  if (name == "idle") faceIdle();
  else if (name == "surprised") faceSurprised();
  else if (name == "sad") faceSad();
  else if (name == "happy") faceHappy();
  else if (name == "angry") faceAngry();
  else if (name == "passionate") facePassionate();
  else if (name == "annoyed") faceAnnoyed();
  else if (name == "excited") faceExcited();
  else if (name == "default") faceDefault();
  else return false;
  return true;
}

// GET /face?name=<expr>, for a WiFi client or a curl. The tablet does not use
// this -- it sends "face:<expr>" over the USB serial link (serial-link.ino).
// Unknown names get a 400, like /move.
void handleFace() {
  String name = server.arg("name");
  if (!runFace(name)) {
    server.send(400, "text/plain", "unknown face \"" + name + "\"");
    return;
  }
  logf("face: %s\n", name.c_str());
  server.send(200, "text/plain", statusBody());
}

*/