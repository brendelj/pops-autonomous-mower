#!/usr/bin/env python3
"""Apply mower UART range reporting to a clean Ai-Thinker BU04 SDK checkout."""
from pathlib import Path
import sys

root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("firmware/stm32-bu04-positioning/vendor-project")
path = root / "Components/APP/user_out.c"
source = path.read_text(encoding="utf-8")
marker = "bool aiio_output = true;"
assert source.count(marker) == 1, "Upstream user_out.c changed: marker not unique"
injection = r'''
/*
 * Mower UART telemetry (version 1).
 * R,<anchor_id>,<distance_cm>\r\n
 * Valid anchor IDs are 1..4. Zero and 0xFFFF mean no usable measurement.
 * No artificial readings are generated. The ESP32 must time out stale data.
 * The BU04's existing _dbg_printf() output channel is used deliberately.
 */
#define MOWER_ANCHOR_COUNT 4
static void mower_emit_ranges(const uint16_t *distances)
{
    int i;
    if (distances == NULL) return;
    for (i = 0; i < MOWER_ANCHOR_COUNT && i < MAX_ANCHOR_LIST_SIZE; ++i)
    {
        uint16_t cm = distances[i];
        if (cm == 0 || cm == 0xFFFF) continue;
        _dbg_printf("R,%d,%u\r\n", i + 1, (unsigned int)cm);
    }
}
'''
source = source.replace(marker, injection + "\n" + marker, 1)
needle = 'void tag_twr_output_user (uint16_t* dis)\n{\n'
assert source.count(needle) == 1, "Upstream tag callback changed"
source = source.replace(needle, needle + '    mower_emit_ranges(dis);\n', 1)
path.write_text(source, encoding="utf-8")
print("Applied mower range UART patch to", path)
