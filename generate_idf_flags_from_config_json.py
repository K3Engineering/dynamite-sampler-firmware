"""Generate idf.py build flags from esp_idf_project_configuration.json.
This is so that you can manually build from CLI with the same options as your vscode.
"""

import argparse
import os
import sys
import json
from pathlib import Path


# All logs / warnings should be on stderr
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def generate_flags(json_str: str, configuration: str) -> list[str]:
    """Parse the json and generate the flags"""
    # The json might contain ${workspace} variables, and they need to be replaced
    # Json doesn't like single \, which is what happens on windows. So use the posix
    # path, even though this isn't what vscode will actually do.
    workspace_dir = Path(__file__).parent.as_posix()
    eprint("Assuming the following workspace dir:")
    eprint(workspace_dir)

    json_str = json_str.replace("${workspaceFolder}", workspace_dir)

    config_dict = json.loads(json_str)

    if configuration not in config_dict:
        eprint(f"{configuration} not in the json")
        exit(1)

    conf_selected = config_dict[configuration]

    flags = []
    # currently only handle the "build" section
    conf_build = conf_selected["build"]
    flags.extend(conf_build.get("compileArgs", []))
    flags.extend(conf_build.get("ninjaArgs", []))
    if conf_build.get("buildDirectoryPath"):
        flags.append("-B")
        flags.append(conf_build.get("buildDirectoryPath"))
    if conf_build.get("sdkconfigFilePath"):
        flags.append(f"-DSDKCONFIG={conf_build['sdkconfigFilePath']}")
    if conf_build.get("sdkconfigDefaults"):
        flags.append(
            f"-DSDKCONFIG_DEFAULTS='{';'.join(conf_build['sdkconfigDefaults'])}'"
        )

    return flags


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__
    )
    parser.add_argument("-f", "--file", default="esp_idf_project_configuration.json")
    parser.add_argument("configuration")

    args = parser.parse_args()

    eprint("reading configuration file:")
    eprint(args.file)
    with open(args.file, "r") as f:
        json_str = f.read()

    flags = generate_flags(json_str, args.configuration)

    print(" ".join(flags))
