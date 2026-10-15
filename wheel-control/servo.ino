// The servo on SERVO_CONTROL, driven by the Arduino Servo library. The UNO R4
// has no LEDC peripheral (that was the ESP32's), so this is a hobby-servo
// library rather than a hand-built 50 Hz signal. Only this tab touches the
// pin, and setServo() is the only function that writes it.
//
// The library uses one of the RA4M1's general-purpose timers; analogWrite()
// on the wheel EN pins uses others. If a servo and a wheel EN ever share a
// timer they can interfere -- if the servo twitches or a wheel loses PWM when
// the servo is attached, move SERVO_CONTROL to a different pin in config.h.

#include <Servo.h>

// File-local: nothing outside this tab drives the pin, and the tab model lets
// setServo() be called from anywhere by name.
static Servo servoObj;
static bool servoReady = false;

void setupServo() {
  // attach() with an explicit pulse range: writeMicroseconds() is then clamped
  // to [SERVO_MIN_US, SERVO_MAX_US] by the library, matching the old LEDC map.
  if (servoObj.attach(SERVO_CONTROL, SERVO_MIN_US, SERVO_MAX_US) == 255) {
    Serial.println(F("servo: attach failed -- no servo"));
    return;
  }
  servoReady = true;
  setServo(SERVO_START_DEG);
}

// 0-180 degrees -> a SERVO_MIN_US..SERVO_MAX_US pulse. Out-of-range angles are
// clamped, never refused. servoDeg is recorded even if no servo attached, so
// the page's slider still tracks what was asked for.
void setServo(int deg) {
  deg = constrain(deg, 0, 180);
  servoDeg = deg;

  if (!servoReady) {
    return;
  }

  long us = SERVO_MIN_US + (long)(SERVO_MAX_US - SERVO_MIN_US) * deg / 180;
  servoObj.writeMicroseconds((int)us);
}
