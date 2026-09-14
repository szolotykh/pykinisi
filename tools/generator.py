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

    Fail before writing output so malformed or protocol-v1 schemas cannot replace
    a working protocol-v2 module with incompatible generated code.
    """
    if not isinstance(data, dict):
        raise ValueError("Command schema must be an object")
    version = data.get("version")
    if not isinstance(version, str) or not re.fullmatch(r"2\.\d+\.\d+", version):
        raise ValueError("The Python transport requires a protocol 2.x.x schema")
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
    result += f"        return cls(*_unpack_payload({fmt!r}, payload))\n\n\n"
    return result


def _generate_method(cmd, objects):
    """Emit a synchronous request method; framing and locking stay in transport."""
    props = cmd.get("properties", [])
    args = ", ".join(f"{prop['name']}: {type_mapping[prop['type']]}" for prop in props)
    response = cmd.get("response")
    obj = _response_object(response, objects) if response else None
    return_type = get_object_name(obj) if obj else type_mapping[response["type"]] if response else "None"
    result = f"    def {cmd['command'].lower()}(self{', ' if args else ''}{args}) -> {return_type}:\n"
    description = cmd.get("description", "")
    if props:
        description += "\n\n" + "\n".join(f"{prop['name']}: {prop.get('description', '')}" for prop in props)
    if cmd.get("errors"):
        description += "\n\nController errors: " + ", ".join(cmd["errors"]) + "."
    result += _docstring(description, "        ")
    values = ", ".join(prop["name"] for prop in props)
    result += f"        payload = struct.pack({_format(props)!r}{', ' if values else ''}{values})\n"
    size = f"{get_object_name(obj)}.get_size()" if obj else str(type_to_size_map[response["type"]]) if response else "0"
    result += f"        result = self._request({cmd['command']}, payload, {size})\n"
    if obj:
        result += f"        return {get_object_name(obj)}.decode(result)\n"
    elif response:
        result += f"        return _unpack_payload({'<' + TYPE_FORMATS[response['type']]!r}, result)[0]\n"
    return result + "\n"


def generate_python_code(commands_data):
    """Validate a firmware schema and return deterministic Python module text."""
    objects = validate_schema(commands_data)
    version = tuple(int(part) for part in commands_data["version"].split("."))
    result = ('"""Kinisi protocol-v2 payloads, generated from firmware commands.json.\n\n'
              'Do not edit this file manually; run tools/update-commands.py instead.\n'
              'Transport framing, message IDs, and synchronization live in KinisiController.\n"""\n\n'
              'import struct\nfrom enum import IntEnum\n\n\n')
    result += f"PROTOCOL_VERSION = {version!r}\n\n"
    for cmd in commands_data["commands"]:
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
