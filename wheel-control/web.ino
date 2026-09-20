// The access point and every HTTP route. Each handler runs from inside
// server.handleClient(), on the same task as loop(), which is why none of the
// globals it writes need locking.

#include "page.h"

void startAccessPoint() {
  WiFi.mode(WIFI_AP);

  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println(F("softAP() failed -- no network, nothing can drive the robot"));
    return;
  }

  Serial.print(F("access point \""));
  Serial.print(AP_SSID);
  Serial.print(F("\" up, open http://"));
  Serial.println(WiFi.softAPIP());
}

void startServer() {
  server.on("/", handleRoot);
  server.on("/drive", handleDrive);
  server.on("/stop", handleStop);
  server.on("/enable", handleEnable);
  server.on("/move", handleMove);
  server.on("/moves", handleMoves);
  server.onNotFound(handleNotFound);

  server.enableDelay(false);   // handleClient() must not sleep, loop() is the control loop
  server.begin();
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// "<leftSpeed>,<rightSpeed>,<leftEnabled>,<rightEnabled>" -- every route
// answers with it, so the page's readouts and buttons track the board no
// matter which request came back last, or how many people have it open.
String statusBody() {
  return String(leftSpeed) + "," + String(rightSpeed) + ","
       + (leftEnabled ? "1" : "0") + "," + (rightEnabled ? "1" : "0");
}

void handleEnable() {
  // An absent argument leaves that wheel alone, so one button cannot disarm
  // the other by omission.
  if (server.hasArg("l")) {
    enableWheel('L', server.arg("l").toInt() != 0);
  }
  if (server.hasArg("r")) {
    enableWheel('R', server.arg("r").toInt() != 0);
  }

  Serial.printf("enable: L=%d R=%d\n", leftEnabled, rightEnabled);
  server.send(200, "text/plain", statusBody());
}

void handleDrive() {
  // A missing argument reads as "", and "".toInt() is 0 -- a missing axis is
  // centred, never the previous value.
  int x = constrain(server.arg("x").toInt(), -512, 512);
  int y = constrain(server.arg("y").toInt(), -512, 512);
  lastCommand = millis();

  // A deflected knob is somebody driving, and that outranks the voice system
  // for STICK_PRIORITY_MS -- see stickActive() in voice-activated.ino. The
  // 0,0 heartbeat below must not stamp it: that arrives at 5 Hz whenever the
  // page is open, so it would hold the priority forever.
  if (x != 0 || y != 0) {
    lastStickInput = millis();
  }

  // A routine owns the wheels for its whole length: the knob does not
  // interrupt one, it waits it out. Ending a routine early is a
  // deliberate act -- /stop, or "stop" on the voice link, both of which call
  // cancelMove(). The stamp above still happens while we wait, so the moment
  // the routine does end, the driver already outranks whatever the voice
  // system says next.
  if (routineRunning()) {
    server.send(200, "text/plain", statusBody());
    return;
  }

  // Past this point no routine is running, so "a move" here is a plain /move
  // request. The page heartbeats x=0,y=0 whenever it is open and idle, and
  // that must not overwrite one. A deflected knob does -- whoever is holding
  // the phone wins over whatever asked for the move.
  bool idleHeartbeat = moveRunning() && x == 0 && y == 0;
  if (!idleHeartbeat) {
    cancelMove();
    webX = x;
    webY = y;
  }

  server.send(200, "text/plain", statusBody());
}

void handleStop() {
  cancelMove();
  webX = 0;
  webY = 0;
  lastCommand = millis();

  // STOP is somebody at the page deciding the robot should not be moving, so
  // it takes the wheels off the voice system too -- otherwise a word heard a
  // moment earlier could start a routine right on top of the stop.
  lastStickInput = millis();

  server.send(200, "text/plain", statusBody());
}

// GET /move?name=<name>&ms=<hold>&speed=<0..100>. Only name is required: the
// defaults are MOVE_DEFAULT_MS and full speed, and startMove() caps the hold
// at MOVE_MAX_MS. A move already running is replaced.
void handleMove() {
  // Same rule as the knob, and for a sharper reason: a move started in the
  // middle of a routine does not cancel it. It replaces the step that is
  // running, runMove() waits out the new hold instead, and the routine then
  // carries on from the next step with its own timing already spent. Turning
  // it away is the only honest answer while a routine owns the wheels.
  if (routineRunning()) {
    server.send(409, "text/plain", "a routine is running -- /stop first");
    return;
  }

  String name = server.arg("name");
  long ms = server.arg("ms").toInt();
  int speed = server.hasArg("speed") ? server.arg("speed").toInt() : 100;

  if (!startMove(name, ms, speed)) {
    server.send(400, "text/plain", "unknown move \"" + name + "\", try one of: " + listMoves());
    return;
  }

  server.send(200, "text/plain", statusBody());
}

// GET /moves -- the names in the table, so a client can discover them rather
// than hard-code the list.
void handleMoves() {
  server.send(200, "text/plain", listMoves());
}

void handleNotFound() {
  server.send(404, "text/plain", "not found");
}
