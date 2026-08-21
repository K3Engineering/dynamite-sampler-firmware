#!/usr/bin/env python3
"""Flash helper for boards exposing TinyUSB CDC ports.

The running app presents the TinyUSB ports (VID:PID 303A:4002) while the ROM
download mode presents a *different* USB identity (303A:1001 USB-Serial-JTAG
or 303A:0009 USB-OTG). esptool only retries the port it started with, so a
plain `idf.py -p <console_com> flash` fails: the firmware resets into
download mode correctly, but esptool never follows to the new port.

This script bridges the two steps:
  1. Send the 1200-baud touch on the TinyUSB console port. The firmware
     (flashPortLineCodingCb in main.cpp) translates it into a
     forced-download-boot reboot.
  2. Wait for the ROM download port to enumerate.
  3. Flash it with the same arguments `idf.py flash` would use.

NOTE: the RTS-based hard reset after flashing cannot reach EN on this board,
so tap RESET once when prompted to boot the new app.

Run inside the ESP-IDF shell (or with the IDF venv python):

    python flash_tusb.py                # debug config
    python flash_tusb.py release        # release config
    python flash_tusb.py debug COM12    # explicit console port
"""

import json
import os
import subprocess
import sys
import time

from serial.tools import list_ports
import serial

APP_VID_PID = (0x303A, 0x4002)          # esp_tinyusb composite (2x CDC)
ROM_VID_PIDS = {(0x303A, 0x1001), (0x303A, 0x0009)}  # USJ / USB-OTG download
CONSOLE_MI = "MI_02"                    # console/flash port = 2nd CDC function


def find_port(vid_pids):
    for p in list_ports.comports():
        if p.vid is None:
            continue
        for vid, pid in vid_pids:
            if (p.vid, p.pid) == (vid, pid):
                return p
    return None


def find_console_port():
    ports = [p for p in list_ports.comports() if (p.vid, p.pid) == APP_VID_PID]
    if not ports:
        return None
    for p in ports:
        if CONSOLE_MI in p.hwid:
            return p
    return ports[0]


def reset_to_download(console_port):
    """Trigger the firmware's 1200-baud touch (see flashPortLineCodingCb in
    main.cpp): a single SET_LINE_CODING control request at 1200 baud makes the
    app reboot into ROM download mode.

    Do NOT use a DTR/RTS sequence here: rapid control-line toggling wedges the
    esp_tinyusb stack on the device (control endpoint dies until power cycle).
    """
    port = None
    try:
        port = serial.Serial(port=console_port, baudrate=115200, timeout=0.2)
        time.sleep(0.2)
        # The device typically disappears mid-request; the exception is expected.
        port.baudrate = 1200
        time.sleep(0.5)
    except serial.SerialException:
        pass
    finally:
        if port is not None:
            try:
                port.close()
            except serial.SerialException:
                pass


def wait_for_rom_port(timeout_s=15):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        p = find_port(ROM_VID_PIDS)
        if p:
            return p
        time.sleep(0.2)
    return None


def flash_command(build_dir, rom_port):
    args_file = os.path.join(build_dir, "flasher_args.json")
    with open(args_file, encoding="utf-8") as f:
        flasher_args = json.load(f)

    write_args = [str(a) for a in flasher_args.get("write_flash_args", [])]
    flash_files = flasher_args.get("flash_files", {})

    cmd = [
        sys.executable, "-m", "esptool",
        "--chip", "esp32s3",
        "-p", rom_port,
        "--before", "default-reset",
        "--after", "hard-reset",
        "write-flash",
    ]
    cmd += write_args
    for offset in sorted(flash_files, key=lambda x: int(x, 16)):
        cmd += [str(offset), str(flash_files[offset])]
    return cmd


def main():
    config = sys.argv[1] if len(sys.argv) > 1 else "debug"
    build_dir = os.path.join("build", config)

    console = sys.argv[2] if len(sys.argv) > 2 else None
    if console is None:
        p = find_console_port()
        if p:
            console = p.device

    already_download = find_port(ROM_VID_PIDS)

    if console and not already_download:
        print(f"Resetting via console port {console} into download mode...")
        reset_to_download(console)
    elif already_download:
        print(f"Chip already in download mode ({already_download.device}).")

    rom = wait_for_rom_port()
    if rom is None:
        print("ERROR: ROM download port (303A:1001 / 303A:0009) did not appear.")
        sys.exit(1)
    print(f"ROM download port: {rom.device} ({rom.description})")

    cmd = flash_command(build_dir, rom.device)
    print(" ".join(cmd), "\n")
    ret = subprocess.run(cmd, cwd=build_dir).returncode
    if ret != 0:
        sys.exit(ret)

    print("\nFlash complete. Tap RESET on the board to boot the app.")
    deadline = time.time() + 60
    while time.time() < deadline:
        p = find_console_port()
        if p:
            print(f"TinyUSB console port is back: {p.device}")
            break
        time.sleep(0.3)
    else:
        print("App ports did not come back within 60 s; tap RESET on the board.")


if __name__ == "__main__":
    main()
