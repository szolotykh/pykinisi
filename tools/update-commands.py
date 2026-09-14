"""Regenerate SDK payload codecs from an explicit local schema or firmware branch."""

import argparse
import json
from pathlib import Path
from urllib.parse import quote
from urllib.request import urlopen

from generator import generate, generate_python_code


OUTPUT_PATH = Path(__file__).resolve().parents[1] / "pykinisi" / "KinisiCommands.py"


def main(argv=None):
    """Resolve the source explicitly and write output relative to this script."""
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--schema", type=Path, help="Path to a local protocol-v2 commands.json.")
    source.add_argument("--branch", help="Firmware Git branch or tag to download explicitly.")
    args = parser.parse_args(argv)

    if args.schema is not None:
        generate(args.schema, OUTPUT_PATH)
    else:
        branch = quote(args.branch, safe="/")
        url = ("https://raw.githubusercontent.com/szolotykh/"
               f"kinisi-motor-controller-firmware/{branch}/commands.json")
        with urlopen(url, timeout=30) as response:
            schema = json.loads(response.read().decode("utf-8-sig"))
        # Validate before opening the output so HTTP/schema failures preserve it.
        generated_code = generate_python_code(schema)
        with OUTPUT_PATH.open("w", encoding="utf-8", newline="\n") as output_file:
            output_file.write(generated_code)
    print(f"Generated {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
