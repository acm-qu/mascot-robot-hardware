// The math, in the order loop() runs it: applyDeadzone() -> arcadeMix() ->
// toPwm() -> slew(). Every function here is pure -- numbers in, numbers out,
// touching nothing but the constants in config.h -- which is what makes the
// README's worked example something you can check by hand.

int applyDeadzone(int v) {
  if (abs(v) <= DEADZONE) {
    return 0;
  }

  int magnitude = constrain(abs(v), DEADZONE, 512);
  magnitude = map(magnitude, DEADZONE, 512, 0, 512);
  return (v > 0) ? magnitude : -magnitude;
}

// One stick, two wheels. y drives both wheels the same way and x drives them
// against each other. On a diagonal the sum can reach twice what toPwm()
// accepts, so both values are scaled down together: clipping them separately
// would turn a curve into a straight line.
void arcadeMix(int x, int y, long &l, long &r) {
  l = (long)y + (long)x;
  r = (long)y - (long)x;

  long biggest = max(abs(l), abs(r));
  if (biggest > 512) {
    l = (l * 512) / biggest;
    r = (r * 512) / biggest;
  }
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
