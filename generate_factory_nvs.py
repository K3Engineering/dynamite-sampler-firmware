#!/usr/bin/env python
"""Generate the factory NVS csv.

The csv describes the Factory namespace: the board model and the model-derived
nominal analog values, taken from the hardware revision selected in sdkconfig.
The build system converts it to an NVS image (via IDF's
nvs_create_partition_image) and flashes it to the DynaPersistent partition (see
the project CMakeLists.txt).

Scalar values carry a provenance tag: "<value>,<provenance>". Everything written
here is "nominal" (model-derived, not measured); calibration data overwrites
these keys with the same format and its own provenance tag, e.g.
"4.512,calboard_v2".

All logs / warnings go to stderr.
"""

import argparse
import csv
import sys

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

PROVENANCE_NOMINAL = "nominal"
KVS_VER = "1"
# Scalar keys carrying model-derived nominal values, in csv row order.
SCALAR_KEYS = ("adc_fsr", "adc_gain", "exc", "afe_gain")


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


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


def main() -> None:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__
    )
    parser.add_argument("--sdkconfig", required=True, help="path to the sdkconfig file")
    parser.add_argument("--out", required=True, help="output .csv file path")
    args = parser.parse_args()

    keys = board_keys_from_sdkconfig(args.sdkconfig)

    with open(args.out, "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["key", "type", "encoding", "value"])
        writer.writerow(["Factory", "namespace", "", ""])
        writer.writerow(["board_model", "data", "string", keys["board_model"]])
        writer.writerow(["kvs_ver", "data", "string", KVS_VER])
        for key in SCALAR_KEYS:
            writer.writerow([key, "data", "string", f"{keys[key]},{PROVENANCE_NOMINAL}"])


if __name__ == "__main__":
    main()
