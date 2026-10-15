// The access point and every HTTP route. Each handler runs from inside
// server.handleClient(), on the same task as loop(), which is why none of the
// globals it writes need locking. The server is MiniServer (mini-server.h), a
// small WebServer stand-in over WiFiS3 -- the UNO R4's Wi-Fi library has no
// WebServer of its own.

#include "page.h"

void startAccessPoint() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("no Wi-Fi module found -- nothing can drive the robot"));
    return;
  }

  Serial.print(F("Wi-Fi firmware "));
  Serial.println(WiFi.firmwareVersion());

  // Pin the AP to 192.168.4.1 so the page, the README and any tablet client
  // keep the address they had on the ESP32.
  WiFi.config(IPAddress(192, 168, 4, 1));

  if (WiFi.beginAP(AP_SSID, AP_PASS) != WL_AP_LISTENING) {
    Serial.println(F("beginAP() failed -- no network, nothing can drive the robot"));
    return;
  }

  Serial.print(F("access point \""));
  Serial.print(AP_SSID);
  Serial.print(F("\" up, open http://"));
  Serial.println(WiFi.localIP());
}

void startServer() {
  server.on("/", handleRoot);
  server.on("/drive", handleDrive);
  server.on("/stop", handleStop);
  server.on("/enable", handleEnable);
  server.on("/move", handleMove);
  server.on("/moves", handleMoves);
  server.on("/servo", handleServo);
  server.on("/face", handleFace);
  server.onNotFound(handleNotFound);

  server.begin();
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// "<leftSpeed>,<rightSpeed>,<leftEnabled>,<rightEnabled>,<servoDeg>" -- every
// route answers with it, so the page's readouts, buttons and slider track the
// board no matter which request came back last, or how many people have it
// open.
String statusBody() {
  return String(leftSpeed) + "," + String(rightSpeed) + ","
       + (leftEnabled ? "1" : "0") + "," + (rightEnabled ? "1" : "0") + ","
       + String(servoDeg);
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

  logf("enable: L=%d R=%d\n", leftEnabled, rightEnabled);
  server.send(200, "text/plain", statusBody());
}

void handleDrive() {
  // A missing argument reads as "", and "".toInt() is 0 -- a missing axis is
  // centred, never the previous value.
  int x = constrain(server.arg("x").toInt(), -512, 512);
  int y = constrain(server.arg("y").toInt(), -512, 512);
  lastCommand = millis();

  // A deflected knob is somebody driving, and that outranks the serial link
  // for STICK_PRIORITY_MS -- see stickActive() in routines.ino. The
  // 0,0 heartbeat below must not stamp it: that arrives at 5 Hz whenever the
  // page is open, so it would hold the priority forever.
  if (x != 0 || y != 0) {
    lastStickInput = millis();
  }

  // A routine owns the wheels for its whole length: the knob does not
  // interrupt one, it waits it out. Ending a routine early is a
  // deliberate act -- /stop, or "stop" on the serial link, both of which call
  // cancelMove(). The stamp above still happens while we wait, so the moment
  // the routine does end, the driver already outranks whatever arrives on
  // serial next.
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
  // it takes the wheels off the serial link too -- otherwise a command that
  // arrived a moment earlier could start a routine right on top of the stop.
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

// GET /servo?a=<0..180>. The servo is its own thing: it does not feed the
// failsafe, count as stick input or wait for a routine -- it just goes there.
void handleServo() {
  if (!server.hasArg("a")) {
    server.send(400, "text/plain", "servo needs a=<0..180>");
    return;
  }

  setServo(server.arg("a").toInt());
  logf("servo: %d\n", servoDeg);
  server.send(200, "text/plain", statusBody());
}

void handleNotFound() {
  server.send(404, "text/plain", "not found");
}
