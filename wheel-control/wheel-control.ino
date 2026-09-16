#include <WiFi.h>
#include <WebServer.h>

// Board: 30-pin ESP32 DevKit v1 (WROOM-32, CH340). The comment after each line
// is the silkscreen label to land the wire on -- note GPIO 36 and 39 are
// printed "VP" and "VN", not as numbers.
//
// If you soldered EN/IS elsewhere, this block is the only place to change. EN
// must be output-capable; IS is read-only, so 34-39 suit it -- those are
// input-only (the ESP32 cannot drive them even by mistake) and all on ADC1,
// which keeps reading while Wi-Fi is up.
int RIGHT_R_PWM = 25;   // D25
int RIGHT_L_PWM = 26;   // D26
int RIGHT_R_EN = 18;    // D18
int RIGHT_L_EN = 19;    // D19
int RIGHT_R_IS = 36;    // VP  <- not labelled "36"
int RIGHT_L_IS = 39;    // VN  <- not labelled "39"

int LEFT_R_PWM = 32;    // D32
int LEFT_L_PWM = 33;    // D33
int LEFT_R_EN = 21;     // D21
int LEFT_L_EN = 22;     // D22
int LEFT_R_IS = 34;     // D34
int LEFT_L_IS = 35;     // D35

int LEFT_INVERT = 1;
int RIGHT_INVERT = -1;
int INVERT_X = 1;
int INVERT_Y = 1;

int DEADZONE = 60;
int MIN_PWM = 40;
int MAX_PWM = 255;
int SLEW = 12;
int LOOP_MS = 20;

// CHANGE ME. WPA2 needs 8 characters or more -- with a shorter one softAP()
// silently brings the network up with no password at all.
const char *AP_SSID = "mascot-robot";
const char *AP_PASS = "mascotbot";

unsigned long COMMAND_TIMEOUT_MS = 500;

// Both wheels come up gated off, so a reset or a fresh upload cannot drive
// anything until someone deliberately arms it in the page. Set true for the
// old behaviour of being live the moment the board boots.
bool START_ENABLED = false;

WebServer server(80);

int webX = 0;
int webY = 0;
unsigned long lastCommand = 0;

int leftSpeed = 0;
int rightSpeed = 0;

bool leftEnabled = false;
bool rightEnabled = false;

unsigned long lastStep = 0;
unsigned long lastPrint = 0;

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Mascot Robot</title>
<style>
  :root {
    --bg: #14161a;
    --panel: #1b1e24;
    --line: #2c313a;
    --text: #e6e8ec;
    --dim: #7c8493;
    --ok: #4ade80;
    --bad: #f87171;
  }
  * { 
    box-sizing: border-box;
  }
  html, body {
    margin: 0;
    height: 100%;
    background: var(--bg);
    color: var(--text);
    font: 15px/1.45 -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    touch-action: none;
    overscroll-behavior: none;
    -webkit-user-select: none;
    user-select: none;
    -webkit-tap-highlight-color: transparent;
  }
  body {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 20px;
    padding: 18px;
  }
  h1 {
    margin: 0;
    font-size: 12px;
    font-weight: 600;
    letter-spacing: .14em;
    text-transform: uppercase;
    color: var(--dim);
  }

  #base {
    position: relative;
    width: min(76vw, 74vh, 310px);
    aspect-ratio: 1;
    border-radius: 50%;
    border: 1px solid var(--line);
    background: radial-gradient(circle at 50% 40%, #262b34, #181b21 72%);
    touch-action: none;
  }
  #base::before, #base::after {
    content: "";
    position: absolute;
    background: var(--line);
    opacity: .55;
  }
  #base::before { left: 8%; right: 8%; top: 50%; height: 1px; }
  #base::after { top: 8%; bottom: 8%; left: 50%; width: 1px; }

  #knob {
    position: absolute;
    left: 50%;
    top: 50%;
    width: 36%;
    aspect-ratio: 1;
    border-radius: 50%;
    background: linear-gradient(180deg, #7aa2ff, #3f6fe0);
    box-shadow: 0 8px 20px rgba(0,0,0,.5), inset 0 1px 0 rgba(255,255,255,.35);
    transform: translate(-50%, -50%);
    transition: transform .13s ease-out;
  }
  #knob.live { transition: none; }

  #stats {
    width: min(76vw, 310px);
    display: grid;
    grid-template-columns: auto 1fr;
    gap: 6px 14px;
    padding: 12px 14px;
    border: 1px solid var(--line);
    border-radius: 10px;
    background: var(--panel);
    font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
    font-size: 13px;
  }
  #stats .k { color: var(--dim); }
  #stats .v { text-align: right; white-space: pre; }
  .ok { color: var(--ok); }
  .bad { color: var(--bad); }

  #arm {
    display: flex;
    gap: 10px;
    width: min(76vw, 310px);
  }
  #arm button {
    flex: 1;
    padding: 14px 6px;
    border: 1px solid var(--line);
    border-radius: 10px;
    background: #24272e;
    color: var(--dim);
    font: inherit;
    font-weight: 700;
    letter-spacing: .08em;
    touch-action: none;
  }
  #arm button.on {
    background: #14532d;
    border-color: #16a34a;
    color: #bbf7d0;
  }

  #stop {
    width: min(76vw, 310px);
    padding: 15px;
    border: 0;
    border-radius: 10px;
    background: #7f1d1d;
    color: #fee2e2;
    font: inherit;
    font-weight: 700;
    letter-spacing: .14em;
    touch-action: none;
  }
  #stop:active { background: #991b1b; }
</style>
</head>
<body>

<h1>Mascot Robot</h1>

<div id="base"><div id="knob"></div></div>

<div id="stats">
  <span class="k">link</span><span class="v bad" id="link">no link</span>
  <span class="k">axes</span><span class="v" id="axes">x 0   y 0</span>
  <span class="k">wheels</span><span class="v" id="wheels">L 0   R 0</span>
</div>

<div id="arm">
  <button id="enL">LEFT OFF</button>
  <button id="enR">RIGHT OFF</button>
</div>

<button id="stop">STOP</button>

<script>
(function () {
  var base = document.getElementById('base');
  var knob = document.getElementById('knob');
  var linkEl = document.getElementById('link');
  var axesEl = document.getElementById('axes');
  var wheelsEl = document.getElementById('wheels');
  var enLEl = document.getElementById('enL');
  var enREl = document.getElementById('enR');

  var TICK_MS = 50;        // how often a held knob is sampled and sent (20 Hz)
  var IDLE_TICKS = 4;      // heartbeat every 4th tick when not driving (5 Hz)

  var tx = 0, ty = 0;      // what the knob is asking for, -512..512
  var active = false;      // finger or mouse is down
  var pid = null;          // pointer we captured
  var burst = 0;           // extra zero sends queued after a release
  var idle = 0;            // ticks since the last idle heartbeat
  var inFlight = false;    // one request at a time, always

  var enabled = { l: false, r: false };
  var pendingEnable = null;   // an arm/disarm waiting for its turn on the wire

  function place(dx, dy) {
    knob.style.transform = 'translate(-50%, -50%) translate(' + dx + 'px, ' + dy + 'px)';
  }

  function track(e) {
    var r = base.getBoundingClientRect();
    var max = r.width / 2;
    var dx = e.clientX - (r.left + max);
    var dy = e.clientY - (r.top + r.height / 2);
    var d = Math.sqrt(dx * dx + dy * dy);
    if (d > max) { dx = dx * max / d; dy = dy * max / d; }
    place(dx, dy);
    tx = Math.round(dx / max * 512);
    ty = Math.round(-dy / max * 512);   // screen y grows downward, forward is up
  }

  function release() {
    if (!active) return;
    active = false;
    knob.classList.remove('live');
    if (pid !== null) {
      try { base.releasePointerCapture(pid); } catch (err) {}
      pid = null;
    }
    tx = 0; ty = 0;
    place(0, 0);
    burst = 3;   // the failsafe is the backstop, these are the primary stop
  }

  base.addEventListener('pointerdown', function (e) {
    active = true;
    pid = e.pointerId;
    knob.classList.add('live');
    try { base.setPointerCapture(pid); } catch (err) {}
    track(e);
    e.preventDefault();
  });

  base.addEventListener('pointermove', function (e) {
    if (!active || e.pointerId !== pid) return;
    track(e);
    e.preventDefault();
  });

  base.addEventListener('pointerup', release);
  base.addEventListener('pointercancel', release);
  window.addEventListener('blur', release);
  document.addEventListener('visibilitychange', function () {
    if (document.hidden) release();
  });

  document.getElementById('stop').addEventListener('click', function () {
    release();
    tx = 0; ty = 0;
    place(0, 0);
    burst = 3;
    fetch('/stop', { cache: 'no-store' })
      .then(function (r) { return r.text(); })
      .then(applyStatus)
      .catch(function () {});
  });

  function renderEnable() {
    enLEl.textContent = 'LEFT ' + (enabled.l ? 'ON' : 'OFF');
    enREl.textContent = 'RIGHT ' + (enabled.r ? 'ON' : 'OFF');
    enLEl.className = enabled.l ? 'on' : '';
    enREl.className = enabled.r ? 'on' : '';
  }

  // Flip locally so the button responds to the tap at once, then queue the
  // request. The reply is authoritative and will correct this if the board
  // disagrees.
  function toggle(side) {
    enabled[side] = !enabled[side];
    renderEnable();
    pendingEnable = { l: enabled.l, r: enabled.r };
  }

  enLEl.addEventListener('click', function () { toggle('l'); });
  enREl.addEventListener('click', function () { toggle('r'); });

  // "<L>,<R>,<leftEnabled>,<rightEnabled>" from any route.
  function applyStatus(txt) {
    var p = txt.split(',');
    if (p.length < 4) return;
    wheelsEl.textContent = 'L ' + p[0] + '   R ' + p[1];
    enabled.l = (p[2] === '1');
    enabled.r = (p[3] === '1');
    renderEnable();
  }

  function setLink(ok) {
    linkEl.textContent = ok ? 'linked' : 'no link';
    linkEl.className = 'v ' + (ok ? 'ok' : 'bad');
  }

  // Returns true only if a request actually went out, so callers that are
  // counting sends (the release burst) do not count skipped ticks.
  function send() {
    if (inFlight) return false;
    inFlight = true;

    var url;
    if (pendingEnable) {
      url = '/enable?l=' + (pendingEnable.l ? 1 : 0) + '&r=' + (pendingEnable.r ? 1 : 0);
      pendingEnable = null;
    } else {
      url = '/drive?x=' + tx + '&y=' + ty;
    }

    var ctrl = new AbortController();
    var bail = setTimeout(function () { ctrl.abort(); }, 250);

    fetch(url, { signal: ctrl.signal, cache: 'no-store' })
      .then(function (r) { return r.text(); })
      .then(function (txt) { applyStatus(txt); setLink(true); })
      .catch(function () { setLink(false); })
      .then(function () { clearTimeout(bail); inFlight = false; });

    return true;
  }

  // 20 Hz while driving, 5 Hz otherwise. Pointer events fire far faster than
  // this; the tick coalesces them so requests cannot pile up.
  setInterval(function () {
    axesEl.textContent = 'x ' + tx + '   y ' + ty;
    if (pendingEnable) {
      send();                   // never consumes a burst slot; if the wire is
                                // busy it simply waits for the next tick
    } else if (active) {
      send();
    } else if (burst > 0) {
      if (send()) burst--;      // a tick skipped because a request was still
                                // outstanding must not consume a zero
    } else if (++idle >= IDLE_TICKS) {
      idle = 0;
      send();
    }
  }, TICK_MS);

  place(0, 0);
  renderEnable();
})();
</script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);

  // Motors first, and stopped, before the radio draws its start-up current.
  setupRightWheel();
  setupLeftWheel();
  stopWheels();
  enableWheel('R', START_ENABLED);
  enableWheel('L', START_ENABLED);

  startAccessPoint();
  startServer();

  Serial.println(F("Mascot robot ready."));
}

void loop() {
  server.handleClient();

  unsigned long now = millis();
  if (now - lastStep < (unsigned long)LOOP_MS) {
    return;
  }
  lastStep = now;

  if (now - lastCommand > COMMAND_TIMEOUT_MS) {
    failsafe();
    return;
  }

  int x = applyDeadzone(INVERT_X * webX);
  int y = applyDeadzone(INVERT_Y * webY);

  long l = (long)y + (long)x;
  long r = (long)y - (long)x;

  long biggest = max(abs(l), abs(r));
  if (biggest > 512) {
    l = (l * 512) / biggest;
    r = (r * 512) / biggest;
  }

  // EN already has the bridge off, but ramping the stored speed down keeps the
  // state honest so re-arming cannot resume at whatever it was doing.
  int targetLeft = leftEnabled ? toPwm(l) : 0;
  int targetRight = rightEnabled ? toPwm(r) : 0;

  leftSpeed = slew(leftSpeed, targetLeft);
  rightSpeed = slew(rightSpeed, targetRight);

  driveWheel('L', LEFT_INVERT * leftSpeed);
  driveWheel('R', RIGHT_INVERT * rightSpeed);

  report(x, y);
}

// The link is gone and there is no telling where the robot is pointed, so cut
// the drive outright instead of ramping it down. Letting slew() handle it would
// add another ~440 ms of travel on top of the timeout. A released knob still
// ramps down normally -- this path is only for a link that stopped answering.
void failsafe() {
  webX = 0;
  webY = 0;

  if (leftSpeed == 0 && rightSpeed == 0) {
    return;
  }

  leftSpeed = 0;
  rightSpeed = 0;
  stopWheels();
  Serial.println(F("link lost -- motors cut"));
}

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
  webX = constrain(server.arg("x").toInt(), -512, 512);
  webY = constrain(server.arg("y").toInt(), -512, 512);
  lastCommand = millis();

  server.send(200, "text/plain", statusBody());
}

void handleStop() {
  webX = 0;
  webY = 0;
  lastCommand = millis();

  server.send(200, "text/plain", statusBody());
}

void handleNotFound() {
  server.send(404, "text/plain", "not found");
}

int applyDeadzone(int v) {
  if (abs(v) <= DEADZONE) {
    return 0;
  }

  int magnitude = constrain(abs(v), DEADZONE, 512);
  magnitude = map(magnitude, DEADZONE, 512, 0, 512);
  return (v > 0) ? magnitude : -magnitude;
}

int toPwm(long v) {
  if (v == 0) {
    return 0;
  }

  long magnitude = constrain(abs(v), 1, 512);
  magnitude = map(magnitude, 1, 512, MIN_PWM, MAX_PWM);
  return (v > 0) ? (int)magnitude : -(int)magnitude;
}

int slew(int current, int target) {
  if (target > current + SLEW) {
    return current + SLEW;
  }
  if (target < current - SLEW) {
    return current - SLEW;
  }
  return target;
}

void report(int x, int y) {
  if (millis() - lastPrint < 250) {
    return;
  }
  lastPrint = millis();

  Serial.print(F("x="));
  Serial.print(x);
  Serial.print(F(" y="));
  Serial.print(y);
  Serial.print(F(" L="));
  Serial.print(leftSpeed);
  Serial.print(F(" R="));
  Serial.println(rightSpeed);
}

void driveWheel(char wheel, int speed) {
  int dir = 0;
  if (speed > 0) {
    dir = 1;
  } else if (speed < 0) {
    dir = -1;
  }
  rotateWheel(wheel, dir, abs(speed));
}

void stopWheels() {
  rotateWheel('L', 0, 0);
  rotateWheel('R', 0, 0);
}

// EN starts LOW: the bridge is gated off before anything else runs.
// IS are the driver's own current-sense outputs -- they are INPUTs here. Never
// drive them, or the ESP32 and the BTS7960 fight over the same net.
void setupLeftWheel() {
  pinMode(LEFT_R_PWM, OUTPUT);
  pinMode(LEFT_L_PWM, OUTPUT);
  pinMode(LEFT_R_EN, OUTPUT);
  pinMode(LEFT_L_EN, OUTPUT);
  digitalWrite(LEFT_R_EN, LOW);
  digitalWrite(LEFT_L_EN, LOW);
  pinMode(LEFT_R_IS, INPUT);
  pinMode(LEFT_L_IS, INPUT);
}

void setupRightWheel() {
  pinMode(RIGHT_R_PWM, OUTPUT);
  pinMode(RIGHT_L_PWM, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT);
  pinMode(RIGHT_L_EN, OUTPUT);
  digitalWrite(RIGHT_R_EN, LOW);
  digitalWrite(RIGHT_L_EN, LOW);
  pinMode(RIGHT_R_IS, INPUT);
  pinMode(RIGHT_L_IS, INPUT);
}

// The only function that touches EN. Both halves of a bridge move together,
// and the PWM is always brought down first so neither edge can hand the driver
// a live duty cycle.
void enableWheel(char wheel, bool on) {
  rotateWheel(wheel, 0, 0);

  if (wheel == 'R') {
    rightSpeed = 0;
    rightEnabled = on;
  } else {
    leftSpeed = 0;
    leftEnabled = on;
  }

  int level = on ? HIGH : LOW;
  digitalWrite((wheel == 'R') ? RIGHT_R_EN : LEFT_R_EN, level);
  digitalWrite((wheel == 'R') ? RIGHT_L_EN : LEFT_L_EN, level);
}

void rotateWheel(char wheel, int dir, int speed) {
  int rPin = (wheel == 'R') ? RIGHT_R_PWM : LEFT_R_PWM;
  int lPin = (wheel == 'R') ? RIGHT_L_PWM : LEFT_L_PWM;
  speed = constrain(speed, 0, 255);

  if (dir >= 0) {
    analogWrite(lPin, 0);
  }
  if (dir <= 0) {
    analogWrite(rPin, 0);
  }
  if (dir == 1) {
    analogWrite(rPin, speed);
  }
  if (dir == -1) {
    analogWrite(lPin, speed);
  }
}
