"""Exercise generated protocol-v2 payloads without serial hardware or networking."""

from copy import deepcopy
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    """Load a standalone source file without importing the SDK transport."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


generator = load_module("kinisi_generator", ROOT / "tools" / "generator.py")


def schema_fixture():
    """Create a small schema covering scalar, object, ACK, and internal messages."""
    sample_fields = [
        ("counter", "uint64_t"), ("signed_byte", "int8_t"),
        ("signed_word", "int16_t"), ("signed_long", "int32_t"),
        ("signed_wide", "int64_t"), ("enabled", "bool"), ("value", "double"),
    ]
    fields = [{"name": name, "type": kind, "description": name} for name, kind in sample_fields]
    return {
        "version": "2.1.0",
        "header": {"properties": [
            {"name": "length", "type": "uint8_t"},
            {"name": "command", "type": "uint8_t"},
            {"name": "message_id", "type": "uint16_t"},
        ]},
        "objects": [{"name": "sample_state", "description": "Sample state.", "properties": fields}],
        "error_codes": [{"name": "INVALID_ARGUMENT", "code": 2, "description": "Invalid field."}],
        "commands": [
            {"command": "SET_SAMPLE", "code": "0x01", "direction": "client_to_controller",
             "description": "Set a sample.", "errors": ["INVALID_ARGUMENT"],
             "properties": [fields[0], fields[5], fields[6]]},
            {"command": "GET_SAMPLE", "code": "0x02", "direction": "client_to_controller",
             "response": {"name": "sample_state", "type": "object", "direction": "controller_to_client"}},
            {"command": "GET_DIRECT", "code": "0x03", "direction": "client_to_controller",
             "response": {"name": "sample", "type": "sample_state", "direction": "controller_to_client"}},
            {"command": "GET_COUNT", "code": "0x04", "direction": "client_to_controller",
             "response": {"name": "count", "type": "int16_t", "direction": "controller_to_client"}},
            {"command": "GET_FLAG", "code": "0x05", "direction": "client_to_controller",
             "response": {"name": "enabled", "type": "bool", "direction": "controller_to_client"}},
            {"command": "ERROR", "code": "0x7F", "direction": "controller_to_client"},
            {"command": "TIME_SYNC_REQUEST", "code": "0x71", "direction": "controller_to_client"},
            {"command": "TIME_SYNC_RESPONSE", "code": "0x72", "direction": "client_to_controller",
             "properties": [{"name": "host_receive_us", "type": "uint64_t"},
                            {"name": "host_send_us", "type": "uint64_t"}]},
            {"command": "READY", "code": "0x73", "direction": "controller_to_client"},
            *[{"command": name, "code": code, "description": name,
               "direction": "controller_to_client" if name.endswith("_EVENT") else "client_to_controller"}
              for name, code in [("PING", "0x76"), ("SET_HEARTBEAT_CONFIG", "0x77"),
                                 ("SUBSCRIBE_ODOMETRY", "0x79"), ("UNSUBSCRIBE_ODOMETRY", "0x7A"),
                                 ("ENCODER_ODOMETRY_EVENT", "0x7B"), ("PLATFORM_ODOMETRY_EVENT", "0x7C")]],
        ],
    }


def generate_namespace(schema):
    """Compile generated source so tests exercise the emitted public methods."""
    namespace = {"__name__": "generated_fixture"}
    exec(compile(generator.generate_python_code(schema), "<generated_fixture>", "exec"), namespace)
    return namespace


def recording_client(namespace):
    """Return a minimal transport that records requests and supplies raw replies."""
    class Client(namespace["KinisiCommands"]):
        """Capture the contract between a generated command and its transport."""

        def __init__(self):
            """Start with no previous request and an empty acknowledgement."""
            self.calls = []
            self.reply = b""

        def _request(self, command, payload, expected_response_length):
            """Record only payload bytes, leaving framing outside the generator."""
            self.calls.append((command, payload, expected_response_length))
            return self.reply

    return Client()


class GeneratedCodecTests(unittest.TestCase):
    """Check byte ordering, signed values, complete samples, and method direction."""

    def setUp(self):
        """Compile the independent test schema for each behavior check."""
        self.namespace = generate_namespace(schema_fixture())
        self.client = recording_client(self.namespace)

    def test_request_is_payload_only_and_waits_for_ack(self):
        """An ordinary setter requests its ACK and does not construct a v1 frame."""
        self.assertIsNone(self.client.set_sample(0x0102030405060708, True, 1.5))
        expected = bytes.fromhex("08 07 06 05 04 03 02 01 01 00 00 00 00 00 00 f8 3f")
        self.assertEqual(self.client.calls, [(0x01, expected, 0)])

    def test_object_round_trip_signed_bool_and_uint64(self):
        """A golden payload verifies little-endian signed, boolean, and float fields."""
        golden = bytes.fromhex(
            "08 07 06 05 04 03 02 01 f9 2e fb c0 1d fe ff "
            "00 00 00 00 00 00 00 80 01 00 00 00 00 00 00 0a c0")
        sample = self.namespace["SampleState"].decode(golden)
        self.assertEqual(sample.counter, 0x0102030405060708)
        self.assertEqual(sample.signed_byte, -7)
        self.assertEqual(sample.signed_word, -1234)
        self.assertEqual(sample.signed_long, -123456)
        self.assertEqual(sample.signed_wide, -(1 << 63))
        self.assertIs(sample.enabled, True)
        self.assertEqual(sample.value, -3.25)
        self.assertEqual(sample.encode(), golden)
        self.assertEqual(sample.get_size(), 32)
        for method, command in [(self.client.get_sample, 2), (self.client.get_direct, 3)]:
            self.client.reply = golden
            self.assertEqual(method().encode(), golden)
            self.assertEqual(self.client.calls[-1], (command, b"", 32))

    def test_scalar_signed_and_boolean_responses(self):
        """Signed scalar decoding preserves negatives and booleans are actual bools."""
        self.client.reply = b"\xfe\xff"
        self.assertEqual(self.client.get_count(), -2)
        self.client.reply = b"\x00"
        self.assertIs(self.client.get_flag(), False)
        self.client.reply = b"\x01"
        self.assertIs(self.client.get_flag(), True)

    def test_truncated_and_oversized_responses_are_rejected(self):
        """Object and scalar codecs reject any reply with an unexpected byte count."""
        for length in (0, 31, 33):
            with self.subTest(object_length=length), self.assertRaises(ValueError):
                self.namespace["SampleState"].decode(bytes(length))
        for length in (0, 1, 3):
            self.client.reply = bytes(length)
            with self.subTest(scalar_length=length), self.assertRaises(ValueError):
                self.client.get_count()

    def test_internal_messages_are_constants_without_public_request_methods(self):
        """Controller messages and its sync reply cannot accidentally wait for ACKs."""
        for name in ("ERROR", "TIME_SYNC_REQUEST", "TIME_SYNC_RESPONSE", "READY"):
            self.assertIn(name, self.namespace)
            self.assertFalse(hasattr(self.client, name.lower()))
        self.assertEqual(self.namespace["PROTOCOL_VERSION"], (2, 1, 0))
        self.assertEqual(self.namespace["ErrorCode"].INVALID_ARGUMENT, 2)
        self.assertEqual(self.namespace["ERROR_DESCRIPTIONS"][2], "Invalid field.")

    def test_out_of_range_integer_cannot_be_encoded(self):
        """The uint64 payload must not silently truncate an oversized timestamp."""
        with self.assertRaises(struct.error):
            self.client.set_sample(1 << 64, True, 0.0)
        self.assertEqual(self.client.calls, [])


class SchemaValidationTests(unittest.TestCase):
    """Reject incompatible input before overwriting the generated SDK module."""

    def test_missing_and_invalid_message_directions(self):
        """Every top-level message and response must declare the correct direction."""
        for direction in (None, "request", "host_to_device"):
            data = schema_fixture()
            data["commands"][0]["direction"] = direction
            with self.subTest(direction=direction), self.assertRaisesRegex(ValueError, "direction"):
                generator.generate_python_code(data)
        data = schema_fixture()
        data["commands"][1]["response"]["direction"] = "client_to_controller"
        with self.assertRaisesRegex(ValueError, "opposite direction"):
            generator.generate_python_code(data)

    def test_protocol_v1_and_old_header_are_rejected(self):
        """An old firmware branch cannot generate legacy framing for this transport."""
        for version in ("1.4.0", "3.0.0", "2", None):
            data = schema_fixture()
            data["version"] = version
            with self.subTest(version=version), self.assertRaisesRegex(ValueError, "protocol 2"):
                generator.generate_python_code(data)
        data = schema_fixture()
        data["header"]["properties"].pop()
        with self.assertRaisesRegex(ValueError, "message_id"):
            generator.generate_python_code(data)

    def test_invalid_references_and_collisions(self):
        """Duplicate wire codes and undefined types/errors fail with useful errors."""
        cases = []
        data = schema_fixture()
        data["commands"][1]["code"] = "0x01"
        cases.append((data, "Duplicate"))
        data = schema_fixture()
        data["commands"][1]["response"]["name"] = "missing_object"
        cases.append((data, "undefined response"))
        data = schema_fixture()
        data["commands"][0]["errors"] = ["MISSING_ERROR"]
        cases.append((data, "undefined error"))
        data = schema_fixture()
        data["objects"][0]["properties"][0]["type"] = "unknown_t"
        cases.append((data, "Unsupported"))
        data = schema_fixture()
        data["error_codes"].append(deepcopy(data["error_codes"][0]))
        cases.append((data, "Duplicate"))
        for data, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                generator.generate_python_code(data)

    def test_invalid_schema_preserves_existing_output(self):
        """Validation happens before an existing generated file is opened for writing."""
        with tempfile.TemporaryDirectory() as directory:
            schema = Path(directory) / "schema.json"
            output = Path(directory) / "commands.py"
            output.write_text("preserved\n", encoding="utf-8")
            for version in ("1.4.0", "2.0.0"):
                data = schema_fixture()
                data["version"] = version
                if version == "2.0.0":
                    data["commands"] = [c for c in data["commands"] if c["command"] != "PING"]
                schema.write_text(json.dumps(data), encoding="utf-8")
                with self.subTest(version=version), self.assertRaises(ValueError):
                    generator.generate(schema, output)
                self.assertEqual(output.read_text(encoding="utf-8"), "preserved\n")

    def test_new_versions_and_commands_are_generated(self):
        """New v2 schemas add SDK methods without requiring a generator version edit."""
        for version in ("2.1.1", "2.2.0", "2.10.0"):
            data = schema_fixture()
            data["version"] = version
            data["commands"].append({"command": "GET_NEW_VALUE", "code": "0x60",
                "direction": "client_to_controller", "description": "A future command.",
                "response": {"name": "value", "type": "uint16_t", "direction": "controller_to_client"}})
            namespace = generate_namespace(data)
            client = recording_client(namespace)
            client.reply = struct.pack("<H", 42)
            self.assertEqual(client.get_new_value(), 42)
            self.assertEqual(namespace["PROTOCOL_VERSION"], tuple(map(int, version.split('.'))))

    def test_generation_is_deterministic(self):
        """Regeneration has no timestamp or other machine-dependent content."""
        first = generator.generate_python_code(schema_fixture())
        second = generator.generate_python_code(deepcopy(schema_fixture()))
        self.assertEqual(first, second)
        self.assertNotIn("Timestamp:", first)


class CheckedInSchemaTests(unittest.TestCase):
    """Check important firmware wire layouts in the shipped generated module."""

    def setUp(self):
        """Load the committed/generated module without requiring pyserial."""
        self.commands = load_module("checked_in_commands", ROOT / "pykinisi" / "KinisiCommands.py")

    def test_odometry_preserves_timestamp_and_clock_metadata(self):
        """Both odometry methods return measurement time and full numeric state."""
        client = recording_client(vars(self.commands))
        client.reply = bytes.fromhex("08 07 06 05 04 03 02 01 01 02 00 00 00 00 00 00 f8 3f")
        sample = client.get_encoder_odometry(2)
        self.assertEqual(client.calls[-1], (0x16, b"\x02", 18))
        self.assertEqual((sample.timestamp_us, sample.clock_mode, sample.clock_quality, sample.angle),
                         (0x0102030405060708, 1, 2, 1.5))
        client.reply = sample.encode() + bytes.fromhex("00 00 00 00 00 00 00 40 00 00 00 00 00 00 08 40")
        platform = client.get_platform_odometry()
        self.assertEqual(client.calls[-1], (0x48, b"", 34))
        self.assertEqual((platform.timestamp_us, platform.clock_mode, platform.clock_quality),
                         (sample.timestamp_us, 1, 2))
        self.assertEqual((platform.x, platform.y, platform.t), (1.5, 2.0, 3.0))

    def test_init_identity_uses_firmware_layout(self):
        """INIT encodes all eight fields and returns the complete board identity."""
        client = recording_client(vars(self.commands))
        client.reply = bytes.fromhex("01 03 01 00 02 00 00 78 56 34 12 f0 de bc 9a")
        identity = client.init(1, 2, 0, 0, 2, 0, 0, 1)
        self.assertEqual(client.calls[-1], (0x70, bytes.fromhex("01 02 00 00 02 00 00 01"), 15))
        self.assertEqual((identity.board_major, identity.board_minor, identity.board_patch), (3, 1, 0))
        self.assertEqual(identity.firmware_build_high, 0x12345678)
        self.assertEqual(identity.firmware_build_low, 0x9ABCDEF0)

    def test_local_firmware_regeneration_matches(self):
        """When the sibling checkout exists, detect stale generated command code."""
        schema = ROOT.parent / "kinisi-motor-controller-firmware" / "commands.json"
        if not schema.is_file():
            self.skipTest("Sibling firmware checkout is optional")
        data = json.loads(schema.read_text(encoding="utf-8-sig"))
        expected = generator.generate_python_code(data)
        actual = (ROOT / "pykinisi" / "KinisiCommands.py").read_text(encoding="utf-8")
        self.assertEqual(actual, expected)


class UpdaterTests(unittest.TestCase):
    """Check explicit source selection and local generation without any network."""

    def setUp(self):
        """Load the command-line updater while supplying its sibling generator."""
        with mock.patch.dict(sys.modules, {"generator": generator}):
            self.updater = load_module("command_updater", ROOT / "tools" / "update-commands.py")

    def test_source_is_required_and_output_is_script_relative(self):
        """Calling from another folder must never choose main or a relative output."""
        self.assertEqual(self.updater.OUTPUT_PATH, ROOT / "pykinisi" / "KinisiCommands.py")
        with mock.patch("sys.stderr", new=io.StringIO()), self.assertRaises(SystemExit):
            self.updater.main([])

    def test_local_schema_does_not_fetch_remote(self):
        """An explicit schema regenerates the same output without opening a URL."""
        with tempfile.TemporaryDirectory() as directory:
            schema = Path(directory) / "commands.json"
            output = Path(directory) / "generated.py"
            schema.write_text(json.dumps(schema_fixture()), encoding="utf-8")
            with mock.patch.object(self.updater, "OUTPUT_PATH", output), \
                    mock.patch.object(self.updater, "urlopen") as fetch, \
                    mock.patch("sys.stdout", new=io.StringIO()):
                self.updater.main(["--schema", str(schema)])
            fetch.assert_not_called()
            self.assertEqual(output.read_text(encoding="utf-8"), generator.generate_python_code(schema_fixture()))


if __name__ == "__main__":
    unittest.main()
