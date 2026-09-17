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

  // The page heartbeats x=0,y=0 whenever it is open and idle, and that must
  // not overwrite a running move. A deflected knob does -- whoever is holding
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

  server.send(200, "text/plain", statusBody());
}

// GET /move?name=<name>&ms=<hold>&speed=<0..100>. Only name is required: the
// defaults are MOVE_DEFAULT_MS and full speed, and startMove() caps the hold
// at MOVE_MAX_MS. A move already running is replaced.
void handleMove() {
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
