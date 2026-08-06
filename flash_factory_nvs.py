#!/usr/bin/env python
"""Flash the factory NVS data to the dynamite sampler board.

Generates the Factory namespace (board model and model-derived nominal analog
values) and flashes it to the DynaPersistent partition as an NVS image.
`idf.py flash` never touches the partition; factory data is written only by
this script.

WARNING: this rewrites the entire DynaPersistent partition, erasing the User
namespace (and anything else stored in Factory).

The board is selected with --board, or read from an sdkconfig file otherwise.
Run with the ESP-IDF python environment (requires IDF_PATH to be set).

Scalar values carry a provenance tag: "<value>,<provenance>". Everything written
here is "nominal" (model-derived, not measured); calibration data overwrites
these keys with the same format and its own provenance tag, e.g.
"4.512,calboard_v2".

All logs / warnings go to stderr.
"""

import argparse
import csv
import os
import subprocess
import sys
import tempfile

# Kconfig option name -> Factory namespace keys.
# `board_model` must match the `name` fields in main/board_cfg.h.
# The numeric values are nominal, derived from the board model (not measured).
BOARD_MODELS = {
    "CONFIG_DYNAMITE_HW_REV_V3": {
        "board_model": "v300",
        # EXC+ and AFE gain are unverified guesses for this board.
        "adc_fsr": "1.2",
        "adc_gain": "1.0",
        "exc": "4.53",
        "afe_gain": "50.0",
    },
    "CONFIG_DYNAMITE_HW_REV_V4": {
        "board_model": "v400",
        # EXC+ and AFE gain are unverified guesses for this board.
        "adc_fsr": "1.2",
        "adc_gain": "1.0",
        "exc": "4.53",
        "afe_gain": "50.0",
    },
    "CONFIG_DYNAMITE_HW_REV_V5": {
        "board_model": "v500",
        # EXC+ and AFE gain are unverified guesses for this board.
        "adc_fsr": "1.2",
        "adc_gain": "1.0",
        "exc": "4.53",
        "afe_gain": "50.0",
    },
    "CONFIG_DYNAMITE_HW_REV_V6_LITE": {
        "board_model": "v600L",
        "adc_fsr": "1.2",
        "adc_gain": "1.0",
        "exc": "2.8",
        "afe_gain": "1.0",
    },
    "CONFIG_DYNAMITE_HW_REV_V6_PRO": {
        "board_model": "v600P",
        "adc_fsr": "1.2",
        "adc_gain": "1.0",
        "exc": "4.53",
        "afe_gain": "101.0",
    },
    "CONFIG_DYNAMITE_HW_REV_V7_LITE": {
        "board_model": "v700L",
        "adc_fsr": "1.2",
        "adc_gain": "1.0",
        "exc": "2.8",
        "afe_gain": "1.0",
    },
    "CONFIG_DYNAMITE_HW_REV_V7_PRO": {
        "board_model": "v700P",
        "adc_fsr": "1.2",
        "adc_gain": "1.0",
        "exc": "4.53",
        "afe_gain": "101.0",
    },
}

MODELS_BY_NAME = {k["board_model"]: k for k in BOARD_MODELS.values()}

PROVENANCE_NOMINAL = "nominal"
KVS_VER = "1"
# Scalar keys carrying model-derived nominal values, in csv row order.
SCALAR_KEYS = ("adc_fsr", "adc_gain", "exc", "afe_gain")

# Must match the DynaPersistent partition in partitions.csv.
FACTORY_PARTITION_NAME = "DynaPersistent"
FACTORY_PARTITION_SIZE = "0x4000"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def idf_tool_path(*parts: str) -> str:
    idf_path = os.environ.get("IDF_PATH", "")
    if not idf_path:
        eprint("error: IDF_PATH is not set, run with the ESP-IDF python environment")
        sys.exit(1)
    return os.path.join(idf_path, *parts)


def board_keys_from_sdkconfig(sdkconfig_path: str) -> dict:
    selected = [k for o, k in BOARD_MODELS.items() if _option_enabled(sdkconfig_path, o)]
    if len(selected) != 1:
        eprint(
            f"error: expected exactly one board revision enabled in "
            f"'{sdkconfig_path}', found: "
            f"{', '.join(k['board_model'] for k in selected) or 'none'}"
        )
        sys.exit(1)
    return selected[0]


def _option_enabled(sdkconfig_path: str, option: str) -> bool:
    with open(sdkconfig_path, encoding="utf-8") as f:
        return any(line.strip() == f"{option}=y" for line in f)


def generate_factory_csv(keys: dict, csv_path: str) -> None:
    with open(csv_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["key", "type", "encoding", "value"])
        writer.writerow(["Factory", "namespace", "", ""])
        writer.writerow(["board_model", "data", "string", keys["board_model"]])
        writer.writerow(["kvs_ver", "data", "string", KVS_VER])
        for key in SCALAR_KEYS:
            writer.writerow([key, "data", "string", f"{keys[key]},{PROVENANCE_NOMINAL}"])


def run(cmd: list) -> None:
    r = subprocess.run(cmd)
    if r.returncode != 0:
        eprint(f"error: command failed: {' '.join(cmd)}")
        sys.exit(r.returncode)


def main() -> None:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__
    )
    board_args = parser.add_mutually_exclusive_group()
    board_args.add_argument(
        "--board",
        choices=sorted(MODELS_BY_NAME),
        help="board model to flash, e.g. v700P (default: read from --sdkconfig)",
    )
    board_args.add_argument(
        "--sdkconfig",
        default=os.path.join(SCRIPT_DIR, "sdkconfig"),
        help="path to the sdkconfig file (default: %(default)s)",
    )
    parser.add_argument("--port", help="serial port of the board (default: auto-detect)")
    parser.add_argument(
        "--no-flash",
        action="store_true",
        help="only generate factory_nvs.csv/.bin, do not flash",
    )
    parser.add_argument(
        "--out",
        help="directory for the generated factory_nvs.csv/.bin "
        "(default: temporary directory, or the current directory with --no-flash)",
    )
    args = parser.parse_args()

    keys = MODELS_BY_NAME[args.board] if args.board else board_keys_from_sdkconfig(
        args.sdkconfig
    )

    nvs_gen = idf_tool_path(
        "components", "nvs_flash", "nvs_partition_generator", "nvs_partition_gen.py"
    )

    def work(work_dir: str) -> None:
        csv_path = os.path.join(work_dir, "factory_nvs.csv")
        bin_path = os.path.join(work_dir, "factory_nvs.bin")

        generate_factory_csv(keys, csv_path)
        run([sys.executable, nvs_gen, "generate", csv_path, bin_path, FACTORY_PARTITION_SIZE])
        eprint(f"Generated factory NVS image for {keys['board_model']}: {bin_path}")

        if args.no_flash:
            return

        parttool = idf_tool_path("components", "partition_table", "parttool.py")
        cmd = [sys.executable, parttool]
        if args.port:
            cmd += ["--port", args.port]
        cmd += [
            "write_partition",
            "--partition-name",
            FACTORY_PARTITION_NAME,
            "--input",
            bin_path,
        ]
        run(cmd)

    if args.out or args.no_flash:
        # Keep the artifacts: write them to a known location.
        work_dir = args.out or os.getcwd()
        os.makedirs(work_dir, exist_ok=True)
        work(work_dir)
    else:
        with tempfile.TemporaryDirectory() as work_dir:
            work(work_dir)


if __name__ == "__main__":
    main()
