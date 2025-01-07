#!/usr/bin/env python
"""Flash the calibration data to the dynamite sampler board
"""

import binascii
import struct

import esptool

PARTITION_TABLE_OFFSET = 0x8000
MAX_PARTITION_LENGTH = 0xC00
MAGIC_BYTES = b"\xAA\x50"

## Detect the port, connect to ESP
ser_list = esptool.get_port_list([], [], [])
print("Serial ports found:", ser_list)

esp: esptool = esptool.get_default_connected_device(
    ser_list, port=None, connect_attempts=1, initial_baud=esptool.ESPLoader.ESP_ROM_BAUD
)

## Read the NVS

description = esp.get_chip_description()
print(description)

esp = esp.run_stub()


def flash_progress(progress, length):
    print("Progress:", progress, length)


data = esp.read_flash(PARTITION_TABLE_OFFSET, MAX_PARTITION_LENGTH, flash_progress)
print("got partition table")

STRUCT_FORMAT = b"<2sBBLL16sL"

partition_offset = {}
for vals in struct.iter_unpack(STRUCT_FORMAT, data):
    (magic, type, subtype, offset, size, name, flags) = vals
    if magic != MAGIC_BYTES:
        break
    name = name.strip(b"\x00").decode()
    partition_offset[name] = offset
    print(name, offset, hex(offset))

print(partition_offset)
## Find the correct offset
flash_offset = partition_offset["loadcell_calib"]

## Generate the calibration data to flash
calibration_data = struct.pack("<III", 11, 22, 33)
print("calibration data")
print(binascii.hexlify(calibration_data, " "))

## Flash the calibration data to the offset

total_size = len(calibration_data)
print(f"Binary size: {total_size} bytes")

# Write binary blocks


def progress_callback(percent):
    print(f"Wrote: {int(percent)}%")


esp.flash_begin(total_size, flash_offset)
for i in range(0, total_size, esp.FLASH_WRITE_SIZE):
    block = calibration_data[i : i + esp.FLASH_WRITE_SIZE]
    # Pad the last block
    block = block + bytes([0xFF]) * (esp.FLASH_WRITE_SIZE - len(block))
    esp.flash_block(block, i + flash_offset)
    progress_callback(float(i + len(block)) / total_size * 100)
esp.flash_finish()

# Reset the chip out of bootloader mode
esp.hard_reset()
"""

# The port of the connected ESP
PORT = "/dev/ttyACM0"
# The binary file
BIN_FILE = "./firmware.bin"
# Flash offset to flash the binary to
FLASH_ADDRESS = 0x10000

def progress_callback(percent):
    print(f"Wrote: {int(percent)}%")

with detect_chip(PORT) as esp:
    description = esp.get_chip_description()
    features = esp.get_chip_features()
    print(f"Detected ESP on port {PORT}: {description}")
    print(f"Features: {", ".join(features)}")

    esp = esp.run_stub()
    with open(BIN_FILE, 'rb') as binary:
        # Load the binary
        binary_data = binary.read()
        total_size = len(binary_data)
        print(f"Binary size: {total_size} bytes")

        # Write binary blocks
        esp.flash_begin(total_size, FLASH_ADDRESS)
        for i in range(0, total_size, esp.FLASH_WRITE_SIZE):
            block = binary_data[i:i + esp.FLASH_WRITE_SIZE]
            # Pad the last block
            block = block + bytes([0xFF]) * (esp.FLASH_WRITE_SIZE - len(block))
            esp.flash_block(block, i + FLASH_ADDRESS)
            progress_callback(float(i + len(block)) / total_size * 100)
        esp.flash_finish()

        # Reset the chip out of bootloader mode
        esp.hard_reset()

"""
