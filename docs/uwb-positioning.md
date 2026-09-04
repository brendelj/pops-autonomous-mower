# BU04 / DW3000 UWB Positioning

## Hardware plan

Five BU04-KIT modules are allocated:

- A1 - fixed anchor
- A2 - fixed anchor
- A3 - fixed anchor
- A4 - fixed anchor
- M - mower-mounted mobile ranging unit

The fixed anchors should be placed around the mowing area with good geometric spread rather than clustered on one side.

## Coordinate system

Choose a yard origin and measure each anchor location in meters.

Example:

```text
A1 = (0.0, 0.0)
A2 = (20.0, 0.0)
A3 = (20.0, 30.0)
A4 = (0.0, 30.0)
```

These are examples only. Real coordinates must be surveyed/measured.

## Data flow

```text
A1 ----A2 -----A3 ------> UWB mower unit -> ranges r1-r4 -> trilateration -> mower X/Y
A4 -----/
```

The ESP32 firmware currently contains an `UWBRanges` structure with one distance for each anchor.

## Interface correction

The BU04/DW3000 integration is treated as SPI rather than the UART wiring shown in the first generated concept diagram.

Signals reserved in the current firmware:

- SPI CLK
- SPI MOSI
- SPI MISO
- SPI CSN
- IRQ
- reset
- wakeup

## Position solution

With known anchor coordinates and measured ranges, solve for the mower position that best fits:

```text
(x-x1)^2 + (y-y1)^2 = r1^2
(x-x2)^2 + (y-y2)^2 = r2^2
(x-x3)^2 + (y-y3)^2 = r3^2
(x-x4)^2 + (y-y4)^2 = r4^2
```

Four anchors permit an overdetermined 2D solution, which is useful for rejecting noisy measurements.

## Work still required

1. Select and validate the DW3000/BU04 ranging library/firmware.
2. Configure unique anchor identities A1-A4.
3. Implement two-way ranging from the mower.
4. Apply antenna-delay/range calibration.
5. Reject stale or implausible ranges.
6. Implement a four-anchor least-squares position solver.
7. Add position filtering.
8. Add heading estimation.
9. Feed position/heading into path following.
