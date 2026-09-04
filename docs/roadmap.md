# Development Roadmap

## Phase 1 - propulsion bench test

- Verify all four BTS7960 connections.
- Verify each motor direction.
- Verify enable behavior.
- Confirm no motor moves at ESP32 startup.
- Confirm physical E-stop removes motor power.
- Measure actual front/rear wheel diameters.

## Phase 2 - FlySky manual control

- Capture actual pulse widths.
- Enter throttle and steering endpoints.
- Confirm deadband.
- Confirm RC signal loss stops the mower.
- Tune wheel direction and scaling.
- Tune maximum drive power.

## Phase 3 - UWB ranging

- Bring up mower BU04/DW3000 over SPI.
- Bring up A1-A4 anchors.
- Measure reliable ranges to all four anchors.
- Calibrate antenna delays.
- Record anchor coordinates.

## Phase 4 - localization

- Implement four-anchor trilateration/least-squares.
- Reject outliers.
- Smooth X/Y position.
- Establish yard coordinate frame.
- Add heading source/estimation.

## Phase 5 - autonomous drive

- Convert target waypoint into desired heading.
- Add heading controller.
- Add speed controller.
- Implement waypoint arrival behavior.
- Verify manual override always wins safely.

## Phase 6 - mowing behavior

- Define legal mowing boundary.
- Define keep-out zones.
- Generate mowing lanes.
- Handle turns between lanes.
- Return-to-home behavior.
- Lost-position behavior.
- Low-battery behavior.

## Phase 7 - obstacle and machine safety

- Add physical bumper/obstacle sensing as selected.
- Add blade/mower safety interlocks separately from propulsion.
- Add watchdog and fault logging.
- Require safe states for sensor/ranging failures.
