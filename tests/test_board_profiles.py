"""Check bundled hardware metadata and explicit, validated profile imports."""

from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from pykinisi import get_board_profile, has_board_sensor, supports_board_sensor
from test_generator import load_module, ROOT


def identity(minor=4, protocol=5, **overrides):
    info = dict(board_model=1, board_major=0, board_minor=minor, board_patch=0,
                protocol_major=2, protocol_minor=protocol, protocol_patch=0)
    info.update(overrides)
    return SimpleNamespace(**info)


class BoardProfileTests(unittest.TestCase):
    def test_exact_lookup_and_resource_counts(self):
        for minor, encoders, gpio in ((1, 1, 4), (2, 4, 4), (3, 4, 10), (4, 4, 8)):
            profile = get_board_profile(identity(minor))
            self.assertEqual(profile['board'], {'model': 1, 'revision': f'0.{minor}.0'})
            self.assertEqual(profile['resources'], dict(motors=4, encoders=encoders, gpio=gpio, status_leds=1))
            self.assertIs(profile, get_board_profile(identity(minor, 1)))
            self.assertEqual([m['current_sense'] for m in profile['motor_features']], [minor == 4] * 4)
            self.assertEqual([m['current_limit'] for m in profile['motor_features']], [minor == 4] * 4)

    def test_sensor_presence_and_protocol_support_are_independent(self):
        self.assertTrue(has_board_sensor(identity(4, 1), 'temperature'))
        self.assertTrue(has_board_sensor(identity(4, 1), 'imu'))
        self.assertFalse(supports_board_sensor(identity(4, 3), 'temperature'))
        self.assertTrue(supports_board_sensor(identity(4, 4), 'temperature'))
        self.assertFalse(supports_board_sensor(identity(4, 4), 'imu'))
        self.assertTrue(supports_board_sensor(identity(4, 5), 'imu'))
        self.assertTrue(supports_board_sensor(identity(4, 6), 'imu'))
        self.assertFalse(supports_board_sensor(identity(protocol_major=3), 'imu'))
        for minor in (1, 2, 3):
            self.assertFalse(supports_board_sensor(identity(minor), 'temperature'))
            self.assertFalse(has_board_sensor(identity(minor), 'imu'))

    def test_unknown_or_malformed_identity_has_no_fallback(self):
        for info in (None, {}, identity(99), identity(board_patch=1), identity(board_model=2),
                     identity(board_minor='4'), identity(board_patch=True), identity(board_minor=-1)):
            self.assertIsNone(get_board_profile(info))
            self.assertFalse(supports_board_sensor(info, 'imu'))
        for index in (-1, 1, 256, 0.5, '0', True, None):
            self.assertFalse(has_board_sensor(identity(), 'imu', index))
        for kind in ('missing', '__proto__', None, []):
            self.assertFalse(has_board_sensor(identity(), kind))
            self.assertFalse(supports_board_sensor(identity(), kind))
        self.assertIsNotNone(get_board_profile(vars(identity())))

    def test_profiles_are_recursively_immutable(self):
        profile = get_board_profile(identity())
        with self.assertRaises(TypeError):
            profile['resources']['motors'] = 99
        with self.assertRaises(TypeError):
            profile['sensors']['imu'][0]['id'] = 99
        with self.assertRaises(AttributeError):
            profile['sensors']['imu'].append({'id': 1})


class ProfileUpdaterTests(unittest.TestCase):
    def setUp(self):
        self.updater = load_module('profile_updater', ROOT / 'tools/update-board-profiles.py')
        self.bundle = json.loads((ROOT / 'pykinisi/board_profiles.json').read_text(encoding='utf-8'))

    def test_bundled_output_is_deterministic_and_public(self):
        self.assertEqual(self.updater.generate(self.bundle),
                         (ROOT / 'pykinisi/board_profiles.json').read_text(encoding='utf-8'))
        self.assertEqual(len(self.bundle['boards']), 4)
        private_fields = {'pin', 'bus', 'timer', 'address', 'mcu', 'build_define', 'control_model'}

        def inspect(value):
            if isinstance(value, dict):
                self.assertFalse(private_fields.intersection(value))
                for child in value.values():
                    inspect(child)
            elif isinstance(value, list):
                for child in value:
                    inspect(child)
        inspect(self.bundle)
        firmware = ROOT.parent / 'kinisi-motor-controller-firmware/include/board_profiles.json'
        if firmware.exists():
            self.assertEqual(self.bundle, json.loads(firmware.read_text(encoding='utf-8')))

    def test_rejects_private_invalid_and_duplicate_data(self):
        for mutate in (
            lambda b: b.update(schema_version=2),
            lambda b: b['boards'].append(deepcopy(b['boards'][0])),
            lambda b: b['boards'][0]['board'].update(build_define='MP_V1'),
            lambda b: b['boards'][0]['resources'].update(motors=True),
            lambda b: b['boards'][3]['sensors']['imu'][0].update(id=1),
            lambda b: b['boards'][3]['motor_features'][0].update(current_limit='yes'),
        ):
            bundle = deepcopy(self.bundle)
            mutate(bundle)
            with self.assertRaises(ValueError):
                self.updater.generate(bundle)

    def test_source_is_explicit(self):
        with mock.patch('sys.stderr', new=io.StringIO()), self.assertRaises(SystemExit):
            self.updater.main([])

    def test_local_import_check_and_failed_import_preserve_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory) / 'source.json', Path(directory) / 'profiles.json'
            source.write_text(json.dumps(self.bundle), encoding='utf-8')
            with mock.patch.object(self.updater, 'OUTPUT_PATH', output), \
                    mock.patch.object(self.updater, 'urlopen') as fetch, \
                    mock.patch('sys.stdout', new=io.StringIO()):
                self.updater.main(['--profiles', str(source)])
                self.updater.main(['--profiles', str(source), '--check'])
                fetch.assert_not_called()
                output.write_bytes(b'preserved\n')
                with self.assertRaisesRegex(ValueError, 'stale'):
                    self.updater.main(['--profiles', str(source), '--check'])
                self.bundle['schema_version'] = 2
                source.write_text(json.dumps(self.bundle), encoding='utf-8')
                with self.assertRaisesRegex(ValueError, 'schema'):
                    self.updater.main(['--profiles', str(source)])
                self.assertEqual(output.read_bytes(), b'preserved\n')

    def test_explicit_branch_download_uses_the_requested_source(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'profiles.json'
            with mock.patch.object(self.updater, 'OUTPUT_PATH', output), \
                    mock.patch.object(self.updater, 'urlopen', return_value=io.BytesIO(json.dumps(self.bundle).encode())) as fetch, \
                    mock.patch('sys.stdout', new=io.StringIO()):
                self.updater.main(['--branch', 'feature/board-profiles'])
            fetch.assert_called_once_with(
                'https://raw.githubusercontent.com/szolotykh/kinisi-motor-controller-firmware/feature/board-profiles/include/board_profiles.json', timeout=30)
            self.assertEqual(json.loads(output.read_text(encoding='utf-8')), self.bundle)


if __name__ == '__main__':
    unittest.main()
