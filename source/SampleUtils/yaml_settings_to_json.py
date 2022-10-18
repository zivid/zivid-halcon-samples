"""
Script to convert from YAML/YML to JSON

"""

import argparse
import json
from pathlib import Path

import yaml


def _args() -> argparse.Namespace:
    """Function for taking in arguments from user.

    Returns:
        Argument from user

    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", type=Path, required=True)
    return parser.parse_args()


def _main() -> None:
    input_directory = _args().directory
    if not input_directory.is_dir():
        raise Exception(f"{input_directory} is not a directory")
    output_directory = Path(f"{input_directory}_json")
    output_directory.mkdir(exist_ok=True)
    for x in list(_args().directory.glob("*.yml")):
        output_file = output_directory / f"{x.stem}.json"
        with open(x, "r", encoding="utf-8") as yaml_in, open(output_file, "w", encoding="utf-8") as json_out:
            yaml_object = yaml.safe_load(yaml_in)
            json.dump(yaml_object, json_out, indent=2)


if __name__ == "__main__":
    _main()
