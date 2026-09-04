# Pops Autonomous Mower

ESP32-based four-wheel autonomous mower project with manual FlySky RC control and UWB yard positioning.

## Current hardware

- ESP32 main controller
- FlySky RC transmitter/receiver
- 4 x BTS7960 motor drivers
- 4 x 755 brushed DC drive motors
- 5 x Ai-Thinker BU04-KIT UWB modules using the DW3000
  - 4 fixed yard anchors: A1-A4
  - 1 mower-mounted ranging unit
- 12 V battery/power system
- 12 V to 5 V BEC for logic power
- Front wheels slightly smaller than rear wheels

## Repository layout

- `firmware/esp32/` - mower ESP32 firmware
- `docs/hardware.md` - component inventory and architecture
- `docs/wiring.md` - wiring plan and pin assignments
- `docs/uwb-positioning.md` - BU04/DW3000 anchor/tag plan
- `docs/calibration.md` - FlySky and wheel calibration
- `docs/roadmap.md` - remaining autonomous-navigation work

## Current control architecture

The ESP32 owns propulsion. It accepts FlySky throttle/steering in manual mode and autonomous forward/turn commands in autonomous mode, but never both at the same time.

Safety behavior already represented in firmware:

- RC signal loss stops propulsion.
- Autonomous command timeout stops propulsion.
- Mode changes stop motors before re-enabling the selected source.
- A physical emergency stop should independently remove motor power.

## Important UWB correction

The BU04/DW3000 mower interface is treated as SPI, not UART. The module exposes SPI clock, MOSI, MISO, chip select, IRQ, reset and wakeup signals.

## Status

The four-motor propulsion, RC mode switching, calibration hooks, wheel scaling and UWB integration hook are in place. Actual DW3000 two-way-ranging, four-anchor trilateration, heading/navigation and mowing-path planning are the next major pieces.
