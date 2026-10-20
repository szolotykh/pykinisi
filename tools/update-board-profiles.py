"""Bundle public firmware board profiles for the Python SDK."""
import argparse
import json
from pathlib import Path
import re
from urllib.parse import quote
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def fields(value, names, at):
    require(isinstance(value, dict) and set(value) == set(names), at + ': invalid fields')


def integer(value, low, high):
    return type(value) is int and low <= value <= high


def validate(bundle):
    """Reject unsupported formats or private hardware fields before replacing output."""
    fields(bundle, ('schema_version', 'boards'), 'profiles')
    require(type(bundle['schema_version']) is int and bundle['schema_version'] == 1, 'Unsupported profile schema')
    require(isinstance(bundle['boards'], list) and bool(bundle['boards']), 'Expected board profiles')
    identities = set()
    for profile in bundle['boards']:
        fields(profile, ('board', 'resources', 'motor_features', 'sensors'), 'board profile')
        board = profile['board']
        fields(board, ('model', 'revision'), 'board identity')
        require(integer(board['model'], 1, 255), 'Invalid board model')
        revision = board['revision']
        require(isinstance(revision, str) and re.fullmatch(r'(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)', revision), 'Invalid board revision')
        require(all(int(v) <= 255 for v in revision.split('.')), 'Invalid board revision')
        identity = (board['model'], revision)
        require(identity not in identities, 'Duplicate board identity')
        identities.add(identity)
        resources = profile['resources']
        fields(resources, ('motors', 'encoders', 'gpio', 'status_leds'), 'resources')
        require(all(integer(v, 0, 255) for v in resources.values()), 'Invalid resource count')
        features = profile['motor_features']
        require(isinstance(features, list) and len(features) == resources['motors'], 'Motor feature count mismatch')
        for index, feature in enumerate(features):
            fields(feature, ('id', 'current_sense', 'current_limit'), 'motor features')
            require(type(feature['id']) is int and feature['id'] == index, 'Invalid motor ID')
            require(type(feature['current_sense']) is bool and type(feature['current_limit']) is bool, 'Invalid motor features')
        fields(profile['sensors'], ('temperature', 'imu'), 'sensors')
        for kind, sensors in profile['sensors'].items():
            require(isinstance(sensors, list) and len(sensors) <= 255, 'Invalid sensor list')
            for index, sensor in enumerate(sensors):
                fields(sensor, ('id',) if kind == 'temperature' else ('id', 'accelerometer', 'gyroscope'), 'sensor')
                require(type(sensor['id']) is int and sensor['id'] == index, 'Invalid sensor ID')
                if kind == 'imu':
                    require(type(sensor['accelerometer']) is bool and type(sensor['gyroscope']) is bool, 'Invalid IMU features')
    return bundle


def generate(bundle):
    """Serialize the public bundle deterministically after complete validation."""
    validate(bundle)
    return json.dumps(bundle, indent=2) + "\n"


OUTPUT_PATH = ROOT / 'pykinisi/board_profiles.json'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--profiles', type=Path, help='Generated firmware include/board_profiles.json')
    source.add_argument('--branch', help='Firmware repository branch or tag to import')
    parser.add_argument('--check', action='store_true', help='Check bundled output without writing')
    args = parser.parse_args(argv)
    if args.profiles is not None:
        raw = args.profiles.read_text(encoding='utf-8-sig')
    else:
        url = ('https://raw.githubusercontent.com/szolotykh/kinisi-motor-controller-firmware/' +
               quote(args.branch, safe='/') + '/include/board_profiles.json')
        with urlopen(url, timeout=30) as response:
            raw = response.read().decode('utf-8-sig')
    content = generate(json.loads(raw))
    if args.check:
        require(OUTPUT_PATH.exists() and OUTPUT_PATH.read_text(encoding='utf-8') == content,
                'Bundled board profiles are stale; run tools/update-board-profiles.py')
    else:
        with OUTPUT_PATH.open('w', encoding='utf-8', newline='\n') as output:
            output.write(content)
    print('Board profiles up to date' if args.check else f'Generated {OUTPUT_PATH}')


if __name__ == '__main__':
    main()
