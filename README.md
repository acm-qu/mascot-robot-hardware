# Mascot Robot — Hardware

Firmware for the mascot robot's physical subsystems. Each subsystem is one
Arduino sketch folder, flashed straight from the Arduino IDE — there is no
build system here and nothing to install beyond the board packages.

| Subsystem | What it does | Board |
| --- | --- | --- |
| [`wheel-control/`](wheel-control/) | Drives the two wheels from an on-screen joystick the board serves itself, over its own Wi-Fi access point, plus named moves (`forward`, `spin_right`, …) over the same HTTP API | ESP32 DevKit v1 |

`docs/` holds board reference photos shared across subsystems.

## Layout

```
mascot-robot-hardware/
├── docs/                     board photos, referenced from subsystem READMEs
└── wheel-control/            one Arduino sketch folder
    ├── README.md             the wiring, the math, the safety model
    ├── wheel-control.ino     shared globals, setup(), loop(), the failsafe
    ├── config.h              pins, tuning, Wi-Fi credentials -- the file to edit
    ├── mixing.ino            stick position -> wheel speeds
    ├── moves.ino             named moves: forward, spin_right, slight_left, ...
    ├── web.ino               access point + HTTP routes
    ├── page.h                the control page the browser loads
    └── wheels.ino            the two H-bridges: pins, enable, PWM
```

A sketch folder has to be named after the main `.ino` inside it — that is an
Arduino rule, not a preference — and every other `.ino` and `.h` in the folder
is compiled along with it, so each subsystem stays a folder of its own, code
only. **To flash a subsystem, open the `.ino` that shares its folder's name**
(`wheel-control/wheel-control.ino`); the IDE loads the rest as tabs.

## License

MIT — see [`LICENSE`](LICENSE).

## The other half

The tablet-side Android app lives in a sibling repository,
[mascot-robot-software](https://github.com/acm-qu/mascot-robot-software). The two
are independent: nothing here imports from there, and the wheel control is driven
from a browser rather than from the app.
