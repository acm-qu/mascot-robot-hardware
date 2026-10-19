// A tiny stand-in for the ESP32 core's WebServer, built on WiFiS3's WiFiServer
// and WiFiClient. It exists because the UNO R4's Wi-Fi library has no WebServer
// class, and it deliberately implements only the sliver this sketch used:
// on() / onNotFound() / begin() / handleClient(), hasArg() / arg() over the URL
// query, and send() / send_P(). web.ino's handlers are otherwise unchanged.
//
// handleClient() is TIME-BOUNDED, and that is the whole point of the class.
// It runs inline on loop()'s thread, and every WiFiS3 call is a round-trip to
// the on-board ESP32-S3 radio, so a slow or half-open request must never stall
// the control loop: if it did, controlStep() -- and with it failsafe() -- would
// stop running and the wheels would hold their last PWM with nobody steering. A
// request that does not complete within REQUEST_MS is dropped and the loop
// carries on; the failsafe then cuts the motors as it always would.
//
// It is defined in a header, not a tab, because more than one tab names the
// MiniServer type and the Arduino build only auto-prototypes functions, not
// classes -- the same reason page.h and routines.h are headers.
#pragma once

#include <Arduino.h>
#include <WiFiS3.h>

class MiniServer {
 public:
  typedef void (*Handler)();

  explicit MiniServer(uint16_t port) : server(port) {}

  void begin() { server.begin(); }

  // Register a handler for an exact path (the part before '?'). Extra routes
  // past MAX_ROUTES are dropped rather than overflowing the table.
  void on(const char *path, Handler h) {
    if (routeCount < MAX_ROUTES) {
      paths[routeCount] = path;
      handlers[routeCount] = h;
      routeCount++;
    }
  }

  void onNotFound(Handler h) { notFound = h; }

  // Accept at most one request, parse it, dispatch, and close. Returns at once
  // when nothing is waiting, so calling it every loop pass costs almost nothing.
  void handleClient() {
    client = server.available();
    if (!client) return;
    path = "";
    query = "";
    if (readRequest()) dispatch();
    // Whatever happened, do not leave a client hanging: flush, a beat for the
    // radio to drain (WiFiS3 truncates output if stopped too fast), then close.
    if (client) {
      client.flush();
      delay(1);
      client.stop();
    }
  }

  bool hasArg(const char *name) {
    int s, e;
    return findArg(name, s, e);
  }

  String arg(const char *name) {
    int s, e;
    if (!findArg(name, s, e)) return String("");
    return query.substring(s, e);
  }

  void send(int code, const char *contentType, const String &body) {
    writeStatus(code);
    writeHeaders(contentType, body.length());
    client.print(body);
  }

  // For the big HTML page: an ordinary char array (PROGMEM is a no-op on the
  // R4's flat address space), streamed in chunks so a copy of it never has to
  // exist in this board's 32 KB of RAM.
  void send_P(int code, const char *contentType, const char *body) {
    size_t len = strlen(body);
    writeStatus(code);
    writeHeaders(contentType, len);
    size_t off = 0;
    while (off < len) {
      size_t n = len - off;
      if (n > CHUNK) n = CHUNK;
      client.write(reinterpret_cast<const uint8_t *>(body) + off, n);
      off += n;
    }
  }

 private:
  static const int MAX_ROUTES = 12;
  static const unsigned long REQUEST_MS = 150;
  static const size_t CHUNK = 512;
  static const int LINE_MAX = 220;

  WiFiServer server;
  WiFiClient client;
  const char *paths[MAX_ROUTES] = {0};
  Handler handlers[MAX_ROUTES] = {0};
  int routeCount = 0;
  Handler notFound = nullptr;
  String path;
  String query;

  // Read the request line ("GET /path?query HTTP/1.1"), split path from query,
  // then drain the headers to the blank line -- all under one deadline so a
  // client that stops sending cannot hold the loop.
  bool readRequest() {
    unsigned long deadline = millis() + REQUEST_MS;
    char line[LINE_MAX];
    if (readLine(line, sizeof(line), deadline) <= 0) return false;

    String req(line);
    int sp1 = req.indexOf(' ');
    int sp2 = (sp1 < 0) ? -1 : req.indexOf(' ', sp1 + 1);
    if (sp1 < 0 || sp2 < 0) return false;

    String target = req.substring(sp1 + 1, sp2);
    int q = target.indexOf('?');
    if (q < 0) {
      path = target;
    } else {
      path = target.substring(0, q);
      query = target.substring(q + 1);
    }

    // Read past the remaining headers so the client has finished sending before
    // we reply. Stop at the blank line (readLine returns 0) or the deadline.
    char hdr[LINE_MAX];
    while (millis() < deadline) {
      if (readLine(hdr, sizeof(hdr), deadline) <= 0) break;
    }
    return true;
  }

  // One line up to '\n', with any '\r' stripped and a NUL appended. Returns the
  // length, 0 for a blank line, or -1 on timeout / a closed connection with
  // nothing buffered. Bounded by the shared deadline.
  int readLine(char *buf, int cap, unsigned long deadline) {
    int n = 0;
    while (millis() < deadline) {
      if (!client.available()) {
        if (!client.connected()) {
          buf[n] = 0;
          return n > 0 ? n : -1;
        }
        continue;
      }
      char c = client.read();
      if (c == '\n') {
        buf[n] = 0;
        return n;
      }
      if (c == '\r') continue;
      if (n < cap - 1) buf[n++] = c;
    }
    buf[n] = 0;
    return -1;
  }

  void dispatch() {
    for (int i = 0; i < routeCount; i++) {
      if (path == paths[i]) {
        handlers[i]();
        return;
      }
    }
    if (notFound) notFound();
  }

  void writeStatus(int code) {
    client.print(F("HTTP/1.1 "));
    client.print(code);
    client.print(' ');
    client.print(reason(code));
    client.print(F("\r\n"));
  }

  void writeHeaders(const char *contentType, size_t length) {
    client.print(F("Content-Type: "));
    client.print(contentType);
    client.print(F("\r\nContent-Length: "));
    client.print(length);
    client.print(F("\r\nConnection: close\r\n\r\n"));
  }

  const __FlashStringHelper *reason(int code) {
    switch (code) {
      case 400: return F("Bad Request");
      case 404: return F("Not Found");
      case 409: return F("Conflict");
      default: return F("OK");
    }
  }

  // Find [name] as a whole key in the query and set [s,e) to its value bounds.
  // Whole-key matching is what stops "x" from matching inside "max".
  bool findArg(const char *name, int &s, int &e) {
    int nlen = (int)strlen(name);
    int L = query.length();
    int i = 0;
    while (i < L) {
      int amp = query.indexOf('&', i);
      int tokEnd = (amp < 0) ? L : amp;
      int eq = query.indexOf('=', i);
      if (eq >= i && eq < tokEnd) {
        if (eq - i == nlen && strncmp(query.c_str() + i, name, nlen) == 0) {
          s = eq + 1;
          e = tokEnd;
          return true;
        }
      } else {  // a bare key with no '='
        if (tokEnd - i == nlen && strncmp(query.c_str() + i, name, nlen) == 0) {
          s = tokEnd;
          e = tokEnd;
          return true;
        }
      }
      i = tokEnd + 1;
    }
    return false;
  }
};
