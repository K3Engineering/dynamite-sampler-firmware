#!/usr/bin/env python
"""Generate the factory NVS partition image.

The image contains the Factory namespace with the board model, taken from the
hardware revision selected in sdkconfig. The build system flashes this image to
the DynaPersistent partition on every `idf.py flash` (see the project
CMakeLists.txt).

All logs / warnings go to stderr.
"""

import argparse
import os
import subprocess
import sys

# Kconfig option name -> board model string.
# Must match the `name` fields in main/board_cfg.h.
BOARD_MODELS = {
    "CONFIG_DYNAMITE_HW_REV_V3": "v300",
    "CONFIG_DYNAMITE_HW_REV_V4": "v400",
    "CONFIG_DYNAMITE_HW_REV_V5": "v500",
    "CONFIG_DYNAMITE_HW_REV_V6_LITE": "v600L",
    "CONFIG_DYNAMITE_HW_REV_V6_PRO": "v600P",
    "CONFIG_DYNAMITE_HW_REV_V7_LITE": "v700L",
    "CONFIG_DYNAMITE_HW_REV_V7_PRO": "v700P",
}

DEFAULT_NVS_GEN = os.path.join(
    os.environ.get("IDF_PATH", ""),
    "components",
    "nvs_flash",
    "nvs_partition_generator",
    "nvs_partition_gen.py",
)


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def board_model_from_sdkconfig(sdkconfig_path: str) -> str:
    selected = [m for o, m in BOARD_MODELS.items() if _option_enabled(sdkconfig_path, o)]
    if len(selected) != 1:
        eprint(
            f"error: expected exactly one board revision enabled in "
            f"'{sdkconfig_path}', found: {', '.join(selected) or 'none'}"
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
    parser.add_argument("--out", required=True, help="output .bin file path")
    parser.add_argument(
        "--size",
        default="0x4000",
        help="NVS partition size, e.g. 0x4000 (default: %(default)s)",
    )
    parser.add_argument(
        "--nvs-gen",
        default=DEFAULT_NVS_GEN,
        help="path to IDF's nvs_partition_gen.py (default: $IDF_PATH/components/...)",
    )
    args = parser.parse_args()

    model = board_model_from_sdkconfig(args.sdkconfig)

    csv_path = args.out + ".csv"
    with open(csv_path, "w", encoding="utf-8", newline="") as f:
        f.write("key,type,encoding,value\n")
        f.write("Factory,namespace,,\n")
        f.write(f"board_model,data,string,{model}\n")

    r = subprocess.run(
        [sys.executable, args.nvs_gen, "generate", csv_path, args.out, args.size]
    )
    if r.returncode != 0:
        eprint("error: nvs_partition_gen failed")
    sys.exit(r.returncode)


if __name__ == "__main__":
    main()
