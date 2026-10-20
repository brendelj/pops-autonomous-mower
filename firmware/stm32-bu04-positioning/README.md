# BU04 STM32 UWB Positioning Firmware

This directory is reserved for the **Ai-Thinker BU04 STM32F103 + DW3000** firmware used by Pops' autonomous mower.

## Intended roles

- **MOWER_TAG**: Mobile BU04 attached to the mower. Obtain range measurements to four anchors and transmit timestamped range records over UART to the ESP32 navigation controller.
- **ANCHOR_1 ... ANCHOR_4**: Fixed BU04 units responding to ranging exchanges, each with a unique ID.

## Project status

**Scaffold only. Not compilable and not flashable yet.** The original Ai-Thinker STM32 source project and its build dependencies have not been recovered. Do not flash placeholder code.

Once the vendor project is available, import its actual STM32CubeIDE project (or documented build system) into `vendor-project/` and adapt its ranging implementation. Preserve the vendor's licensing and attribution. Avoid assuming the BU04's STM32 SPI wiring is exposed for direct ESP32 control.

## Firmware contract (planned)

- Anchor IDs: 1–4, tag ID: 0.
- Range records: anchor ID, distance in meters, timestamp, sequence number, quality/validity status.
- Reject stale, invalid, or missing ranges; no fabricated zero distances.
- Navigation controller must fail safe when positioning is unavailable.
- Firmware binaries are produced only by a verified toolchain build, not by this scaffold.

## GitHub build

`.github/workflows/bu04-stm32-build.yml` is a manually triggered validation workflow. It **intentionally fails with an actionable message** until a real vendor project and build command are configured. It does not claim to generate firmware.
