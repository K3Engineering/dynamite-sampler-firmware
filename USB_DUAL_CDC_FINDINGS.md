# USB Dual CDC Port — Debug Session Findings

Date: 2026-08-20
Scope: two COM ports over USB on the custom ESP32-S3 board (flash/debug console + ADC data stream), flashing workflow, and the "ports unusable / can't reflash" bug.
Status: **fixed and validated** for flashing + monitoring with the full app (BLE, ADC, I2C, PM all running). Uncommitted changes only.

## Environment

- ESP-IDF v6.0.1, `espressif/esp_tinyusb` **2.2.1** (managed component), target ESP32-S3, Windows host (usbser.sys).
- App USB identity: `303A:4002` (esp_tinyusb composite, 2x CDC-ACM).
  - CDC #1 (Windows interface `MI_00`) = DATA port (future ADC stream) — COM13 in testing.
  - CDC #2 (interface `MI_02`) = CONSOLE + FLASH port — COM12 in testing.
- ROM download identities observed: `303A:1001` (USB-Serial-JTAG download, seen after manual BOOT-strap entry) and `303A:0009` (USB-OTG ROM download, seen after force-download reboots). Which one appears depends on *how* download mode was entered; the wrapper accepts both.

## Hardware constraint (by design, not a bug)

USB-Serial-JTAG and USB-OTG share GPIO19/20 on the S3 — only one can be active at a time. Two COM ports therefore **must** be two CDC-ACM interfaces on the TinyUSB stack; "keep the old USJ port for flashing + TinyUSB for data" is not possible simultaneously.

## Root causes found

1. **CDC was not enabled in config.** `# CONFIG_TINYUSB_CDC_ENABLED is not set` in the merged sdkconfig → `CFG_TUD_CDC == 0` → `tinyusb_cdcacm_init()` fails (with `ESP_ERROR_CHECK`, boot loop); no COM ports at all. The coworker's "first working" flash came from an uncommitted local config.
   - Fix: `sdkconfig.common` adds `CONFIG_TINYUSB_CDC_ENABLED=y` and `CONFIG_TINYUSB_CDC_COUNT=2` (count 2 required for two ports; default is 1).

2. **esptool cannot follow the USB identity switch.** The app's flash/console port is `303A:4002`, but ROM download mode enumerates as a different device (`303A:1001`/`0009`, often a different COM number). `idf.py -p COM12 flash` triggers the reset then waits for *the same port* to come back → fails at sync, leaving the chip parked in download mode (which is what "COM ports in a weird state" was).
   - Fix: `flash_tusb.py` wrapper (see below).

3. **Logging / delays inside TinyUSB callbacks wedge the `tud_task`.** CDC callbacks run in the TinyUSB task inside CDC driver event dispatch; `ESP_LOG*` there re-enters the CDC driver through the redirected console and deadlocks the task. Symptom: the device stays enumerated but stops answering control requests → Windows reports `ERROR_GEN_FAILURE` (31) "A device attached to the system is not functioning" on every open/configure.
   - Fix: callbacks only set a flag (`s_downloadRequested`) and return; all work is deferred to the main task.

4. **The original "wedged ports" mechanism: `esp_restart()` during an active USB session leaves the S3 OTG PHY in a state the next boot cannot recover without a power cycle.** The device either doesn't re-enumerate or enumerates "half-dead" (enumeration OK, first control request fails → error 31) until the cable is replugged. This explains every "unplug fixes it" from day one — including wedges triggered while diagnosing causes 2–3.
   - Fix: graceful teardown before reset — defer to the main task, `tinyusb_driver_uninstall()` (deletes the TinyUSB task, disconnects, powers down the PHY), delay, then force-download reboot.

### Dead end kept for the record

An esptool-compatible DTR/RTS reset state machine (like arduino-esp32) was implemented and then **removed**: Windows usbser.sys nondeterministically drops/coalesces RTS-only control-line requests, so the pattern arrives mangled and can also kill the stack mid-sequence. The **1200-baud "touch"** (a single `SET_LINE_CODING` request — the Arduino convention) proved fully deterministic.

## Final design (validated)

- **Reset trigger**: on the console port, set baud to **1200** (or send `R`). The `flashPortLineCodingCb` / `flashPortRxCb` callbacks only set `s_downloadRequested`.
- **Deferred reboot** (main task, `downloadModeShutdownAndReboot`):
  `vTaskDelay(100ms)` → `tinyusb_driver_uninstall()` → `vTaskDelay(100ms)` → `REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT)` → `esp_restart()`.
- **Flash workflow**: `python flash_tusb.py [debug|release]`
  1. Finds the console port (`303A:4002`, `MI_02`), sends the 1200-baud touch.
  2. Waits for the ROM download port (`303A:1001`/`0009`).
  3. Flashes using `build/<config>/flasher_args.json` (same args as `idf.py flash`).
  4. Reports the console port when the app comes back (auto-return observed; otherwise tap RESET).
- **Monitoring** the console port works normally via `idf.py monitor` / any terminal.

### Empirical validation (this session)

- `set 1200` completes cleanly (no mid-transfer exception anymore), device detaches cleanly, ROM download port appears ~1 s later and stays.
- 20/20 and 15/15 open/poke/baud-change "storm" cycles on the console port with no wedge (full app running).
- Two complete unattended wrapper flash round-trips (`python flash_tusb.py debug`).
- Selective-suspend was disabled on the dev PC during diagnosis; final build was verified robust **without** app console writes racing, but subsystems (BLE/ADC/I2C/PM) are all enabled and healthy.

## Files changed (uncommitted)

| File | Change |
|---|---|
| `sdkconfig.common` | `CONFIG_TINYUSB_CDC_ENABLED=y`, `CONFIG_TINYUSB_CDC_COUNT=2` (+comment) |
| `main/main.cpp` | Callbacks wired on console port (`flashPortRxCb`, `flashPortLineCodingCb`), deferred `requestDownloadMode()`/`downloadModeShutdownAndReboot()` in main task, includes for `soc/rtc_cntl_reg.h`/`soc/soc.h`, `esp_timer.h`; full subsystems restored |
| `flash_tusb.py` | **New** flash wrapper (1200-baud touch → ROM port discovery → esptool via flasher_args.json) |

## Known limitations / notes

- Plain `idf.py -p <console-com> flash` still cannot work one-shot (esptool limitation); use `flash_tusb.py`. Using it by accident can wedge the port state until replug — document in the team onboarding.
- The RTS "hard reset" after flashing does not reach EN on this board; the app usually comes back on its own, else tap RESET.
- Windows hub ports latch an error state when a device vanishes mid-transaction — avoid any path that resets the chip without tearing down USB first.
- On ESP32-P4/S31 in `esp_tinyusb` the default VID/PID strings differ; the wrapper is S3-specific (VID/PID + MI_02 heuristics are Windows-specific).
- `taskHello` (commented demo task) is pre-existing dead demo code, untouched.

## Recommended review focus

High-value items for the next review pass (the automated `/review` sub-agents were declined; do these by hand):

1. **Dead code:** `#include <esp_timer.h>` is likely now unused (was needed only by the removed DTR/RTS state machine). Verify and drop.
2. **Security:** the `R` trigger reboots into ROM download mode on *any* byte stream starting with `R` on the console port — no authentication. Acknowledge as dev-only behavior, or gate it (e.g., debug builds only / require a longer magic sequence). Same for the 1200-baud touch.
3. **Business logic:** `tinyusb_driver_uninstall()` return value is ignored; decide the retry/fail policy (currently `esp_restart` happens regardless — probably acceptable, but call it out).
4. **Business logic:** `s_downloadRequested` is `volatile bool` set in the TinyUSB task and polled in main — fine at this size, but note the single-producer/single-consumer assumption (no atomicity concerns expected).
5. **flash_tusb.py:** port-handle lifecycle on exception paths; MI_02/VID:PID heuristics are Windows-specific; `flash_files` offset ordering assumption; clarify behavior when the board has multiple S3 boards attached.
6. **esp_tinyusb 2.2.1**: consider checking the component registry for a newer release; if the PHY-survives-restart behavior reproduces on a devkit with a minimal repro, file an upstream issue (Espressif would want it).
7. **Regression risk:** anything that logs from TinyUSB callback context will reintroduce the error-31 wedge. Worth a code-comment/AGENTS.md note; a future reviewer should know why the callbacks are bare.
8. **Docs:** add flashing instructions (`python flash_tusb.py`) to the README so `idf.py flash` misuse doesn't recreate the wedge reports.
