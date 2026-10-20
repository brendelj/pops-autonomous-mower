# BU04 firmware source findings — 2026-10-20

Verified upstream: https://github.com/Ai-Thinker-AIoT/STM32F103-BU0x_SDK (master).

## Why AT distance reports 0.000000

In `Components/APP/cmd_fn.c`, the source defines `float distance = 0;` and `f_distance()` prints this variable directly via `sprintf(str, "\\r\\ndistance: %f\\r\\n", distance);`. This function **does not initiate a ranging exchange**. A zero result therefore does not prove the DW3000 radio or wiring is broken. The remaining work is to trace which TWR/PDoA callbacks populate this global and whether the current role/configuration starts that measurement path.

## Firmware layout

- `Components/APP/cmd_fn.c`: AT commands and distance output.
- `Components/Examples/examples/ex_05c_ds_twr_init_sts_sdc/ds_twr_sts_sdc_init.c`: initiator example.
- `Components/Examples/examples/ex_05d_ds_twr_resp_sts_sdc/ds_twr_sts_sdc_resp.c`: responder example.
- `Projects/USER/Project.uvprojx`: Keil project targeting STM32F103T8.
- `Components/HAL/DW/twr_pdoa/lib/{aitcmd,os,pdoa,twr}.lib`: precompiled Keil ARMCC libraries.

## Build constraints

Upstream README states Keil ARMCC 5.06 and CMSIS/STM32F1 device packs were used; GCC compilation is **not** demonstrated and these precompiled libraries may prevent a straightforward GNU Arm build. Do not claim a compiled or flashable firmware image until a fresh build has passed.

## Safety and validation

Keep the factory HEX backups. Verify the physical MCU flash capacity and SWD/bootloader recovery method before flashing. Do not interpret invalid/missing ranges as 0 meters; expire stale readings and prevent autonomous motion when positioning is unavailable.
