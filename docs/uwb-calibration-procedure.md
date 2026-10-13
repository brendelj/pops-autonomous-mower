# BU04 repeatable UWB calibration and positioning verification

## Status
The positioning solver is implemented in `firmware/esp32/PopsUwbPositioning.h`.
It accepts four timestamped ranges and four surveyed anchor coordinates; it
does **not** yet receive BU04 UART/USB packets. Do not enable autonomous
propulsion based on this unvalidated output.

## Coordinate convention
Use meters. Survey A1 as (0,0), X toward A2, Y toward A4.
Measure all anchor antenna phase-center X/Y/Z coordinates and the mobile
antenna height Z. Measure the antenna-to-mower reference point offset and
apply that transform later when heading is available. Do not treat antenna
X/Y as cutting-deck center.

## Repeatable range calibration (one anchor at a time)
1. Label physical units A1-A4 and M; record module serial/address, firmware,
   antenna type/orientation, mounting height and firmware antenna-delay settings.
2. With motors OFF, put the M antenna and each anchor antenna at the same
   known height, in clear line of sight. Use fixed, nonmetallic supports.
3. At tape-measured separation distances of 2, 4, 6, 8 and 10 meters,
   collect 100 raw range readings per distance for each A1-M through A4-M pair.
   Record the raw samples, not only the displayed mean. Do not change
   antenna-delay settings halfway through a dataset.
4. Reject packet errors and record median, median absolute deviation and
   number of missing measurements at each separation.
5. Use the five medians and five actual tape measurements as inputs to
   `PopsUWB::calibrate(raw, truth, 5, result)`. Store scale and offset
   PER ANCHOR (not one global value). In `Anchor`, `scale=result.scale`
   and `bias=result.offset`. Keep original data for repeatability.
6. Repeat all five distances on a different day. If fitted corrections or
   residual errors vary materially, inspect antenna mounting, multipath and
   firmware calibration before accepting a correction. Prefer radio-level
   antenna-delay calibration using the manufacturer's procedure first.
7. Re-run after moving antennas, changing firmware, changing antenna type,
   or modifying the battery/ELX enclosure.

## Yard coordinate validation
1. Survey A1-A4 X/Y/Z with a tape or laser and mark at least nine known
   mower-antenna X/Y points: center, four near corners, four edge midpoints.
2. Place the mower stationary at each point with drive disabled. Record
   >=100 complete four-anchor range sets, timestamps and calculated fixes.
3. Compute mean X/Y bias, median horizontal error, 95th-percentile error,
   worst-case error, valid-fix percentage, and per-anchor range residuals.
4. Repeat with the battery and ELX installed and electronics energized,
   but motors disabled. Repeat in different orientations.
5. Hold out validation points: NEVER use the same points both to fit
   corrections and to claim final accuracy.
6. Target <=0.10 m median horizontal error and <=0.20 m 95th-percentile
   error as *provisional engineering goals*, not guaranteed BU04 performance.
   Require valid, fresh fixes and a separately verified safety system before
   any autonomous drive tests.

## Runtime rules
- Reject nonfinite/negative/stale ranges. Default age limit: 500 ms.
- Require >=3 geometrically separated anchors; 4 preferred.
- Reject geometrically degenerate layouts and high range residuals.
- Current solver uses a robust Huber nonlinear least-squares refinement
  seeded by linear trilateration; RMS residual >0.30 m marks a fix invalid.
- Residual RMS is NOT absolute coordinate accuracy or a safety guarantee.
- Missing/invalid fix must not be converted to (0,0). Navigation must stop
  on position loss. Never disable independent RC/obstacle/e-stop interlocks.

## Integrating real BU04 readings
Ai-Thinker BU04-KIT has an onboard STM32 handling DW3000 radio functions.
Verify actual firmware measurement-output direction and packet format over
USB/UART before writing a decoder. Do not confuse the module's internal
DW3000 SPI with an external host protocol. Once confirmed, a transport
adapter should map each message to `Range{meters,millis(),true}` by
unique anchor ID and then call `PopsUWB::solve(...)`.
The existing README/wiring provisional SPI assignments require revision
after this hardware validation; notably GPIO22 is already used in RC
sensor-bypass logic, so do not wire a UWB clock there.

## Calibration CSV format
`date,anchor_id,mower_id,distance_truth_m,range_raw_m,antenna_height_m,orientation_deg,firmware,notes`
Store every individual sample, not just the medians.

References:
- https://github.com/Ai-Thinker-Open/STM32F103-BU0x_SDK
- https://docs.ai-thinker.com/en/uwb_1/index.html
