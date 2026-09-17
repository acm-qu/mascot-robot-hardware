# Mascot Robot — Hardware

Firmware for the mascot robot's physical subsystems. Each subsystem is one
Arduino sketch folder, flashed straight from the Arduino IDE — there is no
build system here and nothing to install beyond the board packages.

| Subsystem | What it does | Board |
| --- | --- | --- |
| [`wheel-control/`](wheel-control/) | Drives the two wheels from an on-screen joystick the board serves itself, over its own Wi-Fi access point | ESP32 DevKit v1 |

`docs/` holds board reference photos shared across subsystems.

## Layout

```
mascot-robot-hardware/
├── docs/                     board photos, referenced from subsystem READMEs
└── wheel-control/            one Arduino sketch folder
    ├── README.md             the wiring, the math, the safety model
    └── wheel-control.ino
```

A sketch folder has to be named after the `.ino` inside it — that is an Arduino
rule, not a preference — so each subsystem stays a folder of its own, code only.

## The other half

The tablet-side Android app lives in a sibling repository,
[mascot-robot-software](https://github.com/acm-qu/mascot-robot-software). The two
are independent: nothing here imports from there, and the wheel control is driven
from a browser rather than from the app.
