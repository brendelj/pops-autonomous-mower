# Hardware Inventory and Architecture

## Propulsion

- 1 x ESP32 main controller
- 4 x BTS7960 H-bridge motor drivers
- 4 x 755 brushed DC motors
- 4 wheels
  - front pair slightly smaller
  - rear pair larger
- 12 V battery/power source
- Power distribution to the four motor drivers
- 12 V to 5 V BEC for ESP32, RC receiver and other logic loads

## Manual control

- FlySky RC transmitter
- FlySky RC receiver
- Receiver channels currently allocated as:
  - CH1: throttle / forward-reverse
  - CH2: steering / left-right
  - CH3: mode selection / auxiliary

## Autonomous positioning

5 x Ai-Thinker BU04-KIT UWB modules using DW3000:

- A1 - fixed yard anchor
- A2 - fixed yard anchor
- A3 - fixed yard anchor
- A4 - fixed yard anchor
- Mower - mobile ranging unit

The four anchors provide geometry and redundancy for two-dimensional positioning. The mower unit measures ranges to the fixed anchors.

## Controller responsibility

The ESP32 is intended to handle:

1. RC input acquisition.
2. RC/manual drive mixing.
3. Autonomous-mode command acceptance.
4. Four BTS7960 motor outputs.
5. Wheel-size compensation.
6. Mode interlock so RC and autonomous control cannot drive simultaneously.
7. UWB range acquisition.
8. Later: trilateration, heading, path tracking and obstacle/safety inputs.

## Safety requirement

A physical emergency-stop path should remove motor power independently of ESP32 software. Software stop logic is not a substitute for a hardware E-stop.
