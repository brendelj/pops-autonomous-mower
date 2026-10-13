# ESP32-S3 Grass Milling Navigation (commissioning v0.1)

This branch adds a **non-driving** navigation prototype without modifying the existing Revision 10 cart controller. Autonomous movement is NOT implemented or enabled. The sketch sends STOP every 100 ms and never sends DRV.

## Measured/estimated geometry

- Cart outside: 40 x 40 inches (1016 x 1016 mm)
- Front/rear axle center spacing: 35 inches (889 mm)
- Cutting footprint: 20 x 20 inches (508 x 508 mm), centered laterally
- BU04 antenna: at front center of cutting footprint, about **9 inches behind cart front** (configurable; expected 8–10 inches)
- Cutting footprint extends 20 inches behind BU04
- Front and rear ultrasonic sensors: cart front/rear center, respectively
- Battery and both electronics enclosures: at cart front
- Mowing lane spacing example: 18 inches (20-inch cut less 2-inch overlap)
- Cart and cutting geometry must be transformed by heading; no valid cart footprint can be inferred from a single UWB point alone.

## Wiring — proposed, verify S3 board labels

Cart Rev10 already uses GPIO16 RX2 and GPIO17 TX2, at 115200 8N1.

| Cart ESP32 | Navigation ESP32-S3 |
|---|---|
| GPIO17 TX2 | GPIO18 RX1 |
| GPIO16 RX2 | GPIO21 TX1 |
| GND | GND |

Both controllers must use compatible 3.3V logic and a common ground. Do not connect 5V signal levels to ESP32 GPIO. The S3 GPIO16 RX2/GPIO17 TX2 are **experimental BU04 TTL input**; actual BU04 binary data has only been confirmed via BU04 USB-C. Do not connect USB D+/D- to UART pins. The native S3 USB host path, USB class and power arrangement need investigation.

For COM/UART-port serial-monitor testing, USB CDC On Boot may need to be Disabled depending on the S3 board's USB wiring.

## Serial monitor commands

- `HELP` — command list
- `ANCHOR,0,0,0` — set surveyed anchor 0 in millimeters (repeat IDs 1,2,3)
- `RANGE,0,2500` — inject an experimental range in millimeters (repeat as needed)
- `HEADING,90` — set **manual** heading in degrees, test only
- `STATUS` — print pose and counters
- `LEARN` — start boundary capture in RAM
- `POINT` — record current cart-center point if pose valid
- `END` — close boundary capture in RAM (requires at least three points)
- `CLEAR_MAP` — clear RAM boundary
- `STOP` — safe-stop

No live heading sensor, USB host, SSD driver, camera driver, obstacle contouring, persistent map, cutting coverage execution, or autonomous path follower is included yet. Console-set anchors/heading and learned points are volatile. The simple polygon and solver are prototype tools; they are not a validated geofence.

## Cart protocol and limitations

The existing cart Rev10 accepts `DRV,throttle,steering[*HH]` and `STOP[*HH]`, and emits `OBS,FRONT,mm`, `OBS,REAR,mm`, `CLEAR,...`, and `SENSOR_FAULT,...`. This sketch sends only STOP and parses those reports. Its stop heartbeat is NOT a substitute for the cart's RC/autonomous switch, sonar interlocks, communication timeout, independent E-stop, or physical blade disable.

Front and rear center sonar do not see cart corners or side hazards. A 40-inch cart needs 40 inches plus verified clearance to pass through a straight corridor. Turning must check the swept footprint; an illustrative centered spin reaches 28.3 inches from cart center to a corner, before safety margin.

## Planned grass-milling strategy

1. Survey anchors and establish trustworthy XY + heading.
2. RC-teach boundary, record quality-tagged samples, persist raw and simplified polygons.
3. Offset permitted mowing region inward by the full cart footprint and positioning uncertainty.
4. Generate perimeter-inward passes; split complicated polygons into regions and use parallel lanes where appropriate.
5. Track the 20-inch cutting footprint independently of the 40-inch cart collision footprint.
6. On an obstacle, cart stops first. Evaluate safe uncut-side detour, then alternate side; if neither safely fits, retreat to a verified turning area, reverse to another uncut lane, and leave blocked cells pending.
7. Add SSD map versioning, coverage checkpoints, camera detections, IMU/encoders, and robust restart behavior.

## Validation gates before movement

1. Confirm BU04 measurement transport and packet fields against manufacturer protocol.
2. Survey anchors and validate range quality and XY error in multiple yard locations.
3. Fit IMU/heading source and verify antenna-to-center transform.
4. Bench-test bidirectional cart UART and sonar messages with motors physically disabled.
5. Add a cart status/ack protocol and prove RC override, comms loss, sonar fault and independent E-stop.
6. Implement conservative speed-limited path following only after geofence and footprint tests.

**Do not enable autonomous drive or blades based on this prototype.**
