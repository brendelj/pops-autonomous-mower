# Calibration

## FlySky receiver

Open the ESP32 Serial Monitor at 115200 baud.

Type:

```text
C
```

The controller will print live receiver pulse widths for throttle, steering and mode.

Record these six positions:

| Control | Position | Firmware setting |
|---|---|---|
| Throttle | full reverse | `RC_THROTTLE_REVERSE` |
| Throttle | neutral | `RC_THROTTLE_NEUTRAL` |
| Throttle | full forward | `RC_THROTTLE_FORWARD` |
| Steering | full left | `RC_STEERING_LEFT` |
| Steering | center | `RC_STEERING_CENTER` |
| Steering | full right | `RC_STEERING_RIGHT` |

Also observe the mode-channel pulse width in manual and autonomous positions and set `RC_MODE_THRESHOLD_US` between those values.

## Wheel direction

Bench-test with the mower supported so all wheels are off the ground.

For each wheel, if a positive forward command makes that wheel rotate backward, change its corresponding direction value from `1` to `-1`.

## Front/rear wheel size compensation

The front wheels are slightly smaller than the rear wheels.

Current starting values:

```text
Front Left  = 1.08
Front Right = 1.08
Rear Left   = 1.00
Rear Right  = 1.00
```

These are only starting values. The correct scale depends on actual rolling circumference, motor speed, loading and traction.

A geometric starting point is:

```text
front scale = rear wheel circumference / front wheel circumference
```

Then tune each side experimentally so the mower tracks straight at low and medium speed.

## Bench commands

With the RC mode switch selecting autonomous mode:

```text
A 0.25 0.00
```

requests 25% forward.

```text
A 0.00 0.25
```

requests a right turn.

```text
A 0.00 -0.25
```

requests a left turn.

```text
A -0.20 0.00
```

requests reverse.

```text
X
```

clears the autonomous command and stops propulsion.
