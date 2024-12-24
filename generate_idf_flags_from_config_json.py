"""Generate idf.py build flags from esp_idf_project_configuration.json.
This is so that you can manually build from CLI with the same options as your vscode.
"""

import argparse
import os
import json
from pathlib import Path


WORKSPACE_DIR = os.path.dirname(os.path.realpath(__file__))


def generate_flags(file_path: Path, configuration: str) -> List[str]:
    pass


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__
    )
    parser.add_argument("-f", "--file", default="esp_idf_project_configuration.json")
    parser.add_argument("configuration")

    args = parser.parse_args()
