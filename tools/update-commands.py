"""Regenerate SDK payload codecs from an explicit local schema or firmware branch."""

import argparse
import json
from pathlib import Path
from urllib.parse import quote
from urllib.request import urlopen

from generator import generate_python_code


OUTPUT_PATH = Path(__file__).resolve().parents[1] / "pykinisi" / "KinisiCommands.py"
SCHEMA_PATH = Path(__file__).resolve().parent / "commands.json"


def main(argv=None):
    """Resolve the source explicitly and write output relative to this script."""
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--schema", type=Path, help="Path to a local protocol-v2 commands.json.")
    source.add_argument("--branch", help="Firmware Git branch or tag to download explicitly.")
    parser.add_argument("--check", action="store_true", help="Check bundled output without writing.")
    args = parser.parse_args(argv)

    if args.schema is not None:
        schema = json.loads(args.schema.read_text(encoding="utf-8-sig"))
    else:
        branch = quote(args.branch, safe="/")
        url = ("https://raw.githubusercontent.com/szolotykh/"
               f"kinisi-motor-controller-firmware/{branch}/commands.json")
        with urlopen(url, timeout=30) as response:
            schema = json.loads(response.read().decode("utf-8-sig"))
    # Validate before opening either output so HTTP/schema failures preserve both.
    generated_code = generate_python_code(schema)
    outputs = {OUTPUT_PATH: generated_code, SCHEMA_PATH: json.dumps(schema, indent=2) + "\n"}
    for path, content in outputs.items():
        if args.check:
            if not path.exists() or path.read_text(encoding="utf-8") != content:
                raise ValueError("Bundled commands are stale; run tools/update-commands.py")
        else:
            with path.open("w", encoding="utf-8", newline="\n") as output_file:
                output_file.write(content)
    print("Commands up to date" if args.check else f"Generated {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
