# STM32 projects — latest sources merged into the laptop CubeIDE projects (2026-10-24)

Base: the user's laptop projects (`~/ESWContest/AIV_rear_stm32`, `AIV_front_stm32`, uploaded as ESWContest_new.zip).
Those projects held OLD sources: rear `main.c` = main_v14 (Oct 9), front `main.c` = front_main_v4 (Oct 9).

What was replaced (everything else — .ioc, Drivers, startup, hal_msp, it, imu_uart, headers — is untouched laptop content):
| Project | Core/Src/main.c | Core/Src/ssd1306.c | Firmware string |
|---|---|---|---|
| AIV_rear_stm32 | main_v18c.c | ssd1306_v18.c | `FW v18c (OLED I2C guard + diag)` |
| AIV_front_stm32 | front_main_v4b.c | front_ssd1306_v4b.c | `FW front v4b (OLED I2C guard + diag)` |

NOT build-verified in CubeIDE (no ARM toolchain was available when this was assembled).
Static check done: v18c uses only HAL modules enabled in the rear project's hal_conf (I2C, SPI, UART, GPIO, RCC, PWR); watchdog is register-level (IWDG->KR), no extra HAL module needed.
`_bins/` = the binaries that were actually built from these sources and flashed (rear 48,748 B). A fresh CubeIDE build will not be byte-identical (different compiler/flags), so compare behavior (FW string, OLEDSTAT lines), not hashes.
`_old_laptop_main/` = the replaced old files, kept for reference.

How to build: STM32CubeIDE → File > Open Projects from File System → select AIV_rear_stm32 → Build. If the build fails, send the full console output.
Excluded from this zip: Debug/ and .metadata/ (build outputs and IDE cache; regenerated).
