"""Generate deterministic Python payload codecs from the firmware protocol schema.

The transport owns protocol-v2 framing, message IDs, acknowledgements, and clock
synchronization. Generated methods only encode payloads and decode their replies.
"""

import json
import keyword
from pathlib import Path
import re
import struct
import textwrap
from protocol_schema import validate_metadata, retirement_notice


# Explicit little-endian struct formats keep payloads independent of the host ABI.
TYPE_FORMATS = {
    "bool": "?", "uint8_t": "B", "uint16_t": "H", "uint32_t": "I",
    "uint64_t": "Q", "int8_t": "b", "int16_t": "h", "int32_t": "i",
    "int64_t": "q", "double": "d", "integer": "i",
}
type_to_size_map = {name: struct.calcsize("<" + fmt) for name, fmt in TYPE_FORMATS.items()}
type_mapping = {name: "bool" if name == "bool" else "float" if name == "double" else "int"
                for name in TYPE_FORMATS}
DIRECTIONS = {"client_to_controller", "controller_to_client"}
# These messages support handwritten session behavior, independently of schema version.
SESSION_COMMANDS = {
    "PING", "SET_HEARTBEAT_CONFIG", "SUBSCRIBE_ODOMETRY", "UNSUBSCRIBE_ODOMETRY",
    "ENCODER_ODOMETRY_EVENT", "PLATFORM_ODOMETRY_EVENT",
    "GET_TEMPERATURE", "GET_IMU", "GET_MOTOR_CURRENT", "GET_MOTOR_FAULTS", "CLEAR_MOTOR_FAULTS",
    "SUBSCRIBE", "SUBSCRIBE_WITH_MODE", "UNSUBSCRIBE", "TELEMETRY_EVENT",
    "GET_PWM_FREQUENCY", "SET_PWM_FREQUENCY", "SUBSCRIBE_CONFIGURED", "GET_MOTOR_CURRENT_LIMIT", "SET_MOTOR_CURRENT_LIMIT",
}



def validate_pwm_frequency(schema):
    values = schema.get('pwm_frequencies', [])
    expected = [(v, 'KHZ_' + str(v // 1000)) for v in (10000, 20000, 25000, 50000, 100000)]
    if [(v.get('id'), v.get('name')) for v in values] != expected or any(type(v.get('id')) is not int for v in values):
        raise ValueError('Invalid PWM frequency catalog')
    commands = {c['command']: c for c in schema.get('commands', [])}
    for name in ('GET_PWM_FREQUENCY', 'SET_PWM_FREQUENCY'):
        command = commands.get(name, {})
        properties = [(p['name'], p['type'], p.get('enum')) for p in command.get('properties', [])]
        expected_props = [] if name.startswith('GET') else [('frequency_hz', 'uint32_t', 'pwm_frequency')]
        if (not command or command.get('direction') != 'client_to_controller' or properties != expected_props or
                command.get('response', {}).get('type') != 'uint32_t'):
            raise ValueError('Invalid ' + name + ' envelope')


def get_object_name(object_data):
    """Translate a schema object name into the public PascalCase class name."""
    return "".join(word.capitalize() for word in object_data["name"].split("_"))


def _identifier(value, context):
    """Reject schema names that cannot safely become Python identifiers."""
    if not isinstance(value, str) or not value.isidentifier() or keyword.iskeyword(value):
        raise ValueError(f"Invalid {context}: {value!r}")


def _code(value, context):
    """Parse an integer or hexadecimal message/error code and require one byte."""
    try:
        result = int(value, 0) if isinstance(value, str) else value
    except ValueError as exc:
        raise ValueError(f"Invalid {context}: {value!r}") from exc
    if isinstance(result, bool) or not isinstance(result, int) or not 0 <= result <= 255:
        raise ValueError(f"Invalid {context}: {value!r}")
    return result


def _properties(properties, context):
    """Validate the ordered scalar fields that make up one payload."""
    if not isinstance(properties, list):
        raise ValueError(f"{context} properties must be an array")
    names = set()
    for prop in properties:
        if not isinstance(prop, dict):
            raise ValueError(f"{context} property must be an object")
        name = prop.get("name")
        _identifier(name, f"{context} property name")
        if name in names or name in {"self", "cls", "struct", "encode", "decode", "get_size"}:
            raise ValueError(f"Duplicate or reserved {context} property: {name}")
        names.add(name)
        if prop.get("type") not in TYPE_FORMATS:
            raise ValueError(f"Unsupported {context} property type: {prop.get('type')!r}")
    if sum(type_to_size_map[prop["type"]] for prop in properties) > 252:
        raise ValueError(f"{context} payload exceeds the protocol-v2 frame capacity")


def _response_object(response, objects):
    """Resolve both legacy object references and direct named response types."""
    name = response.get("name") if response.get("type") == "object" else response.get("type")
    return objects.get(name)


def validate_schema(data):
    """Check protocol version, framing, message directions, and codec references.

    Accept evolving v2 schemas while preserving the heartbeat and telemetry
    definitions used by the handwritten transport.
    """
    if not isinstance(data, dict):
        raise ValueError("Command schema must be an object")
    version = data.get("version")
    if not isinstance(version, str) or not re.fullmatch(r"2\.\d+\.\d+", version):
        raise ValueError("The Python transport requires a protocol 2.x.x schema")
    validate_metadata(data, type_to_size_map)
    expected_header = [("length", "uint8_t"), ("command", "uint8_t"), ("message_id", "uint16_t")]
    header = data.get("header", {}).get("properties", [])
    if [(prop.get("name"), prop.get("type")) for prop in header] != expected_header:
        raise ValueError("Protocol-v2 header must contain length, command, and message_id")

    objects = {}
    class_names = set()
    for obj in data.get("objects", []):
        _identifier(obj.get("name"), "object name")
        class_name = get_object_name(obj)
        _identifier(class_name, "object class name")
        if (obj["name"] in objects or class_name in class_names or
                class_name in {"KinisiCommands", "ErrorCode"} or obj["name"] in TYPE_FORMATS):
            raise ValueError(f"Duplicate or reserved object name: {obj['name']}")
        _properties(obj.get("properties", []), obj["name"])
        objects[obj["name"]] = obj
        class_names.add(class_name)

    error_names = set()
    error_values = set()
    for error in data.get("error_codes", []):
        name = error.get("name")
        _identifier(name, "error name")
        if name != name.upper() or name.startswith("_"):
            raise ValueError(f"Error name must be an uppercase public identifier: {name}")
        value = _code(error.get("code"), "error code")
        if not value or name in error_names or value in error_values:
            raise ValueError(f"Duplicate or reserved error code: {name}")
        error_names.add(name)
        error_values.add(value)

    commands = data.get("commands")
    if not isinstance(commands, list) or not commands:
        raise ValueError("Command schema must contain a nonempty commands array")
    missing = SESSION_COMMANDS - {cmd.get("command") for cmd in commands}
    if missing:
        raise ValueError("Schema is missing runtime commands: " + ", ".join(sorted(missing)))
    names, codes = set(), set()
    for cmd in commands:
        name = cmd.get("command")
        _identifier(name, "command name")
        value = _code(cmd.get("code"), "command code")
        if (name in names or value in codes or name != name.upper() or name.startswith("_")
                or name in {"PROTOCOL_VERSION", "ERROR_DESCRIPTIONS"} or keyword.iskeyword(name.lower())):
            raise ValueError(f"Duplicate or non-uppercase command: {name}")
        names.add(name)
        codes.add(value)
        if cmd.get("direction") not in DIRECTIONS:
            raise ValueError(f"{name} must specify a valid direction")
        _properties(cmd.get("properties", []), name)
        errors = cmd.get("errors", [])
        if not isinstance(errors, list) or any(error not in error_names for error in errors):
            raise ValueError(f"{name} references an undefined error")
        response = cmd.get("response")
        if response is None:
            continue
        expected_direction = ("controller_to_client" if cmd["direction"] == "client_to_controller"
                              else "client_to_controller")
        if response.get("direction") != expected_direction:
            raise ValueError(f"{name} response must have the opposite direction")
        if response.get("type") not in TYPE_FORMATS and _response_object(response, objects) is None:
            raise ValueError(f"{name} references an undefined response type: {response.get('type')!r}")
    return objects


def _format(properties):
    """Build the packed little-endian format for a payload's ordered fields."""
    return "<" + "".join(TYPE_FORMATS[prop["type"]] for prop in properties)


def _docstring(text, indent):
    """Quote schema prose as a safe Python docstring, preserving arbitrary text."""
    prose = (text or "Protocol payload.").replace("\\", "\\\\").replace('"', '\\"')
    lines = []
    for line in prose.splitlines():
        lines.extend(textwrap.wrap(line, width=92 - len(indent), break_long_words=False,
                                   break_on_hyphens=False) if line else [""])
    return (indent + '"""' + lines[0] + "\n" +
            "".join(indent + line + "\n" if line else "\n" for line in lines[1:]) +
            indent + '"""\n')


def generate_object(object_data):
    """Emit one public value object with strict, symmetric payload codecs."""
    name = get_object_name(object_data)
    properties = object_data.get("properties", [])
    fmt = _format(properties)
    args = ", ".join(f"{prop['name']}: {type_mapping[prop['type']]}" for prop in properties)
    values = ", ".join(f"self.{prop['name']}" for prop in properties)
    result = f"class {name}:\n"
    description = object_data.get("description", "")
    if properties:
        description += "\n\n" + "\n".join(f"{prop['name']}: {prop.get('description', '')}" for prop in properties)
    result += _docstring(description, "    ")
    result += f"\n    def __init__(self{', ' if args else ''}{args}):\n"
    result += '        """Store the fields of one controller payload."""\n'
    for prop in properties:
        result += f"        self.{prop['name']} = {prop['name']}\n"
    result += "\n    @staticmethod\n    def get_size() -> int:\n"
    result += '        """Return the encoded payload size, excluding the message header."""\n'
    result += f"        return {struct.calcsize(fmt)}\n"
    result += "\n    def encode(self) -> bytearray:\n"
    result += '        """Encode every field using the packed little-endian wire layout."""\n'
    result += f"        return bytearray(struct.pack({fmt!r}{', ' if values else ''}{values}))\n"
    result += f"\n    @classmethod\n    def decode(cls, payload: bytes) -> '{name}':\n"
    result += '        """Decode exactly one payload; reject truncation and trailing bytes."""\n'
    if object_data['name'] == 'motor_current_limit':
        result += "        if len(payload) != 3 or payload[0] > 1 or int.from_bytes(payload[1:], 'little') == 0:\n"
        result += "            raise ValueError('Invalid current-limit setting')\n"
    if object_data['name'] in ('motor_fault_status', 'motor_fault_sample'):
        offset = 10 if object_data['name'] == 'motor_fault_sample' else 0
        result += f"        if any(value > 1 for value in payload[{offset}:]):\n"
        result += "            raise ValueError('Invalid motor fault flags')\n"
    result += f"        return cls(*_unpack_payload({fmt!r}, payload))\n\n\n"
    return result


def _generate_method(cmd, objects):
    """Emit a synchronous request method; framing and locking stay in transport."""
    props = cmd.get("properties", [])
    args = ", ".join(f"{prop['name']}: {type_mapping[prop['type']]}" for prop in props)
    velocity = cmd['command'] in ('INITIALIZE_MOTOR_CONTROLLER', 'START_PLATFORM_CONTROLLER')
    if velocity:
        args += ' = 100.0'
    response = cmd.get("response")
    obj = _response_object(response, objects) if response else None
    return_type = get_object_name(obj) if obj else type_mapping[response["type"]] if response else "None"
    result = f"    def {cmd['command'].lower()}(self{', ' if args else ''}{args}) -> {return_type}:\n"
    description = cmd.get("description", "")
    if 'retired' in cmd:
        description += '\n\n' + retirement_notice(cmd)
    if props:
        description += "\n\n" + "\n".join(f"{prop['name']}: {prop.get('description', '')}" for prop in props)
    if cmd.get("errors"):
        description += "\n\nController errors: " + ", ".join(cmd["errors"]) + "."
    result += _docstring(description, "        ")
    if cmd['command'] in ('SUBSCRIBE', 'SUBSCRIBE_WITH_MODE', 'SUBSCRIBE_CONFIGURED', 'UNSUBSCRIBE'):
        result += "        validate_topic(topic_id, index)\n"
    if cmd['command'] in ('SUBSCRIBE', 'SUBSCRIBE_WITH_MODE'):
        result += "        if type(interval_ms) is not int or not TOPIC_DEFINITIONS[topic_id][2] <= interval_ms <= 3600000:\n"
        result += "            raise ValueError('Invalid subscription interval')\n"
    if cmd['command'] in ('GET_TEMPERATURE', 'GET_IMU'):
        result += "        if type(sensor_index) is not int or not 0 <= sensor_index <= 255:\n"
        result += "            raise ValueError('sensor_index must be an integer from 0 to 255')\n"
    if cmd['command'] in ('GET_MOTOR_CURRENT', 'GET_MOTOR_FAULTS', 'CLEAR_MOTOR_FAULTS', 'GET_MOTOR_CURRENT_LIMIT', 'SET_MOTOR_CURRENT_LIMIT'):
        result += "        if type(motor_index) is not int or not 0 <= motor_index <= 3:\n"
        result += "            raise ValueError('motor_index must be an integer from 0 to 3')\n"
    if cmd['command'] == 'SET_PWM_FREQUENCY':
        result += "        if isinstance(frequency_hz, bool) or not isinstance(frequency_hz, int) or frequency_hz not in tuple(PwmFrequency):\n            raise ValueError('Invalid PWM frequency')\n"
    if cmd['command'] == 'SET_MOTOR_CURRENT_LIMIT':
        result += "        if type(enabled) is not bool or type(threshold_ma) is not int or not 1 <= threshold_ma <= 65535:\n"
        result += "            raise ValueError('Expected enabled boolean and threshold_ma integer from 1 to 65535')\n"
    if velocity:
        result += "        if any(not math.isfinite(value) or value < 0 for value in (kp, ki, kd, integral_limit)) or integral_limit > 100:\n"
        result += "            raise ValueError('Velocity gains must be finite and nonnegative; integral_limit must be 0..100 PWM percentage points')\n"
    if cmd['command'] == 'SUBSCRIBE_CONFIGURED':
        result += "        if isinstance(trigger, bool) or not isinstance(trigger, int) or trigger not in tuple(SubscriptionTrigger) or isinstance(delivery, bool) or not isinstance(delivery, int) or delivery not in tuple(SubscriptionDelivery):\n            raise ValueError('Invalid subscription settings')\n"
        result += "        minimum = TOPIC_DEFINITIONS[topic_id][2]\n"
        result += "        if type(poll_interval_ms) is not int or poll_interval_ms > 3600000 or (trigger == SubscriptionTrigger.ON_EVENT and (poll_interval_ms != 0 or topic_id not in EVENT_TOPICS)) or (trigger == SubscriptionTrigger.PERIODIC and poll_interval_ms < minimum):\n            raise ValueError('Invalid polling interval or trigger')\n"
        result += "        if type(max_silence_ms) is not int or not 0 <= max_silence_ms <= 3600000 or (max_silence_ms != 0 and (delivery != SubscriptionDelivery.ON_CHANGE or max_silence_ms < minimum)):\n            raise ValueError('Invalid maximum silence')\n"
    if cmd['command'] == 'SUBSCRIBE_WITH_MODE':
        result += "        if isinstance(mode, bool) or not isinstance(mode, int) or mode not in tuple(SubscriptionMode):\n"
        result += "            raise ValueError('Invalid subscription mode')\n"
    values = ", ".join(prop["name"] for prop in props)
    result += f"        payload = struct.pack({_format(props)!r}{', ' if values else ''}{values})\n"
    size = f"{get_object_name(obj)}.get_size()" if obj else str(type_to_size_map[response["type"]]) if response else "0"
    result += f"        result = self._request({cmd['command']}, payload, {size})\n"
    if cmd['command'] == 'GET_IMU':
        result += f"        sample = {get_object_name(obj)}.decode(result)\n"
        result += "        if not all(math.isfinite(value) for value in vars(sample).values()):\n"
        result += "            raise ValueError('Invalid IMU payload')\n"
        result += "        return sample\n"
    elif cmd['command'] == 'GET_TEMPERATURE':
        result += "        temperature = _unpack_payload('<d', result)[0]\n"
        result += "        if not math.isfinite(temperature):\n"
        result += "            raise ValueError('Invalid temperature payload')\n"
        result += "        return temperature\n"
    elif cmd['command'] == 'GET_MOTOR_CURRENT':
        result += "        amps = _unpack_payload('<d', result)[0]\n"
        result += "        if not math.isfinite(amps) or amps < 0:\n"
        result += "            raise ValueError('Invalid motor current payload')\n"
        result += "        return amps\n"
    elif obj:
        result += f"        return {get_object_name(obj)}.decode(result)\n"
    elif response:
        result += f"        return _unpack_payload({'<' + TYPE_FORMATS[response['type']]!r}, result)[0]\n"
    return result + "\n"


def generate_python_code(commands_data):
    """Validate a firmware schema and return deterministic Python module text."""
    objects = validate_schema(commands_data)
    validate_pwm_frequency(commands_data)
    version = tuple(int(part) for part in commands_data["version"].split("."))
    result = ('"""Kinisi protocol-v2 payloads, generated from firmware commands.json.\n\n'
              'Do not edit this file manually; run tools/update-commands.py instead.\n'
              'Transport framing, message IDs, and synchronization live in KinisiController.\n"""\n\n'
              'import math\nimport struct\nfrom enum import IntEnum\n\n\n')
    result += f"PROTOCOL_VERSION = {version!r}\n\n"
    for cmd in commands_data["commands"]:
        if 'retired' in cmd:
            result += '# ' + retirement_notice(cmd) + '\n'
        result += f"{cmd['command']} = 0x{_code(cmd['code'], 'command code'):02X}\n"
    result += '\n\nclass ErrorCode(IntEnum):\n    """Error codes returned by the shared controller ERROR message."""\n\n'
    for error in commands_data.get("error_codes", []):
        result += f"    {error['name']} = {_code(error['code'], 'error code')}\n"
    result += "\n\nERROR_DESCRIPTIONS = {\n"
    for error in commands_data.get("error_codes", []):
        result += f"    {_code(error['code'], 'error code')}: {error.get('description', '')!r},\n"
    result += "}\n\n\n"
    result += '''def _unpack_payload(fmt: str, payload: bytes) -> tuple:
    """Decode one exact payload rather than accepting a partial or extra frame."""
    expected = struct.calcsize(fmt)
    if len(payload) != expected:
        raise ValueError(f"Expected {expected} payload bytes, received {len(payload)}")
    return struct.unpack(fmt, payload)


'''
    for obj in commands_data.get("objects", []):
        result += generate_object(obj)
    if commands_data.get('subscription_modes'):
        result += 'class SubscriptionMode(IntEnum):\n    """Delivery behavior for a generic telemetry subscription."""\n'
        for mode in commands_data['subscription_modes']:
            result += f"    {mode['name']} = {mode['id']}\n"
        result += '\n\n'
    for key, name in [('subscription_triggers', 'SubscriptionTrigger'), ('subscription_deliveries', 'SubscriptionDelivery'), ('pwm_frequencies', 'PwmFrequency')]:
        if commands_data.get(key):
            result += f'class {name}(IntEnum):\n'
            for item in commands_data[key]:
                result += f"    {item['name']} = {item['id']}\n"
            result += '\n\n'
    topics = commands_data.get('topics', [])
    if topics:
        result += 'EVENT_TOPICS = ' + repr(tuple(t['id'] for t in topics if t.get('event_supported'))) + '\n\n'
        result += 'class Topic(IntEnum):\n    """Stable wire identifiers for telemetry sources."""\n'
        for topic in topics:
            result += f"    {topic['name']} = {topic['id']}\n"
        result += '\n\nTOPIC_DEFINITIONS = {\n'
        for topic in topics:
            result += f"    Topic.{topic['name']}: ({get_object_name(objects[topic['payload_type']])}, {topic['index_max']}, {topic['min_interval_ms']}),\n"
        result += '''}


def validate_topic(topic, index):
    """Validate enum and resource index before encoding a command."""
    if isinstance(topic, bool) or not isinstance(topic, int) or topic not in TOPIC_DEFINITIONS:
        raise ValueError('Invalid telemetry topic')
    if type(index) is not int or not 0 <= index <= TOPIC_DEFINITIONS[topic][1]:
        raise ValueError('Invalid telemetry index')


def decode_telemetry_event(payload):
    """Return (topic, index, sample), or None for a future unknown topic."""
    if len(payload) < 2:
        raise ValueError('Truncated telemetry envelope')
    topic, index = payload[:2]
    if topic not in TOPIC_DEFINITIONS:
        return None
    validate_topic(topic, index)
    sample = TOPIC_DEFINITIONS[topic][0].decode(payload[2:])
    if (sample.clock_mode not in (0, 1) or sample.clock_quality not in (1, 2) or
            any(isinstance(value, float) and not math.isfinite(value) for value in vars(sample).values())):
        raise ValueError('Invalid telemetry sample')
    if topic == Topic.MOTOR_CURRENT and sample.amps < 0:
        raise ValueError('Invalid motor current payload')
    return Topic(topic), index, sample


'''
    result += '''class KinisiCommands:
    """Public command methods shared by transports implementing protocol v2."""

    def __init__(self):
        """Leave connection state and synchronization to the transport."""

    def _request(self, command: int, payload: bytes, expected_response_length: int) -> bytes:
        """Exchange a command payload and return its matching response payload."""
        raise NotImplementedError("A protocol-v2 transport must implement _request")

'''
    for cmd in commands_data["commands"]:
        # TIME_SYNC_RESPONSE replies to a controller request without receiving an ACK.
        if cmd["direction"] == "client_to_controller" and cmd["command"] != "TIME_SYNC_RESPONSE":
            result += _generate_method(cmd, objects)
    return result


def generate(path, output_path):
    """Read and validate a local JSON schema, then write the generated module."""
    with Path(path).open("r", encoding="utf-8-sig") as schema_file:
        commands_data = json.load(schema_file)
    generated_code = generate_python_code(commands_data)
    with Path(output_path).open("w", encoding="utf-8", newline="\n") as output_file:
        output_file.write(generated_code)
    return Path(output_path)
