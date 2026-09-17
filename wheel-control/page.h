// The control page the browser loads, as one string compiled into flash.
// web.ino serves it straight from here with send_P(), so there is nothing to
// upload to a filesystem: change the page here and re-upload the sketch.
#pragma once

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
