#!/usr/bin/env python
"""Flash the calibration data to the dynamite sampler board
"""

import argparse
import binascii
import struct
import inspect
import sys
from collections.abc import Iterator
from typing import Self, Optional

from tqdm import tqdm

import esptool

# These are the defaults.
PARTITION_TABLE_OFFSET = 0x8000
MAX_PARTITION_LENGTH = 0xC00


def generate_calibration_raw(data1: int, data2: int) -> bytes:
    """Generate the raw bytes to be flashed to the calibration partition"""
    # TODO this format needs to be synced between this script and the client.
    return struct.pack("II", data1, data2)


class PartitionRow:
    """Format and parsing of a partition table row on the ESP.
    ESP tools has a class like this, but its really cumbersome, so this is a lite version.
    """

    MAGIC_BYTES = b"\xAA\x50"  # Each row starts with this magic number
    STRUCT_FORMAT = b"<2sBBLL16sL"

    def __init__(self, type, subtype, offset, size, name, flags):
        self.type = type
        self.subtype = subtype
        self.offset = offset
        self.size = size
        self.name = name
        self.flags = flags

    def __str__(self):
        # Since the format is fixed, the object string can be fixed width
        self_str = (
            "PartitionRow object: "
            f"type: {self.type:#04x} "
            f"subtype: {self.subtype:#04x} "
            f"offset: {self.offset:#010x} "
            f"size: {self.size:#010x} "
            f"name: {self.name:16} "
            f"flags: {self.flags:#010b} "
        )
        return self_str

    @classmethod
    def iter_from_raw_table(cls, data: bytes) -> Iterator[Self]:
        """Unpack a raw partition table, and yield each row as a PartitionRow"""
        for vals in struct.iter_unpack(PartitionRow.STRUCT_FORMAT, data):
            (magic, type, subtype, offset, size, name, flags) = vals
            if magic == PartitionRow.MAGIC_BYTES:
                name = name.strip(b"\x00").decode()
                yield cls(type, subtype, offset, size, name, flags)


def connect_to_esp() -> esptool.ESPLoader:
    """Connect to an available ESP."""
    ## Detect the port, connect to ESP
    ser_list = esptool.get_port_list([], [], [])
    print("Serial ports found:", ser_list)

    esp: Optional[esptool.ESPLoader] = esptool.get_default_connected_device(
        ser_list,
        port=None,
        connect_attempts=1,
        initial_baud=esptool.ESPLoader.ESP_ROM_BAUD,
    )
    if esp is None:
        raise Exception("Could not connect to ESP")

    return esp


def main(partition_name: str, data_to_flash: bytes):

    ## Detect the port, connect to ESP
    esp = connect_to_esp()

    ## Print the description of the ESP
    description = esp.get_chip_description()
    print(description)

    ## Not sure what this does, but its required to interface with the esp
    esp = esp.run_stub()

    ## Read the NVS
    print("Reading the NVS partition table")

    def flash_progress(progress, length):
        print(f"Read {progress:#x} bytes out of {length:#x}")

    parition_raw = esp.read_flash(
        PARTITION_TABLE_OFFSET, MAX_PARTITION_LENGTH, flash_progress
    )

    ## Parse the partition table
    print(f"Parsing table and looking for partition: {partition_name}")
    flash_offset = None
    for row in PartitionRow.iter_from_raw_table(parition_raw):
        print(row)
        if row.name == partition_name:
            flash_offset = row.offset
            break
    print(f"Using partition offset {flash_offset:#x}")

    ## Generate the calibration data to flash
    # calibration_data = struct.pack("<III", 11, 22, 33)
    print("Data being flashed to partition:")
    print(binascii.hexlify(data_to_flash, " "))

    ## Flash the calibration data to the offset

    total_size = len(data_to_flash)
    print(f"Size of data being flashed: {total_size} bytes")
    print(f"Size of ESP Flash block: {esp.FLASH_WRITE_SIZE} bytes")
    print("Flashing data")
    esp.flash_begin(total_size, flash_offset)
    for i in tqdm(range(0, total_size, esp.FLASH_WRITE_SIZE)):
        block = data_to_flash[i : i + esp.FLASH_WRITE_SIZE]
        # Pad the last block
        block = block + bytes([0xFF]) * (esp.FLASH_WRITE_SIZE - len(block))
        esp.flash_block(block, i + flash_offset)

    esp.flash_finish()

    print("Reseting the ESP out of bootloader mode.")
    esp.hard_reset()

    print("Complete.")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__
    )
    parser.add_argument(
        "-p",
        "--partition",
        default="loadcell_calib",  # This is from the partitions.csv
        help="Name of partition to flash to.",
    )

    calibration_args = tuple(inspect.getfullargspec(generate_calibration_raw).args)
    if sys.version_info[:2] <= (3, 12):
        parser.add_argument(
            "calib_data",
            metavar="calib-data",
            nargs=len(calibration_args),
            help="The loadcell calibration values: " + ", ".join(calibration_args),
            type=int,
        )
    else:
        parser.add_argument(
            "calib_data",
            metavar=calibration_args,
            nargs=len(calibration_args),
            help="The loadcell calibration values",
            type=int,
        )

    args = parser.parse_args()

    # Generate the calibration data to flash
    calibration_data = generate_calibration_raw(*args.calib_data)

    main(args.partition, calibration_data)
