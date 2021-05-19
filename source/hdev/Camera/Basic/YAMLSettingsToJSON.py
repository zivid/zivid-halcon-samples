""" Script to convert from .yaml/.yml to .json
"""

from pathlib import Path
import argparse
import os
import json
import yaml


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory")
    args = parser.parse_args()
    path = Path(args.directory)
    print(path)
    outdirpath = Path(f"{path}_json")
    if not outdirpath.exists():
        os.mkdir(outdirpath)

    if outdirpath.is_dir():
        for x in list(path.glob("*.yml")):
            out_name = f"{outdirpath}\\{x.stem}.json"
            with open(x, "r") as yaml_in, open(out_name, "w") as json_out:
                yaml_object = yaml.safe_load(yaml_in)
                json.dump(yaml_object, json_out, indent=2)

    else:
        print(f"{outdirpath} is not a directory")


if __name__ == "__main__":
    _main()
