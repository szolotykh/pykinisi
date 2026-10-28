"""Read-only hardware profiles bundled with the SDK; no runtime downloads."""

from collections.abc import Mapping
from importlib import resources
import json
from types import MappingProxyType


def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


with resources.open_text(__package__, "board_profiles.json", encoding="utf-8") as _file:
    _bundle = json.load(_file)
if _bundle["schema_version"] != 1:
    raise ValueError("Unsupported bundled board profile schema")
_profiles = {(profile["board"]["model"], profile["board"]["revision"]): _freeze(profile)
             for profile in _bundle["boards"]}
del _bundle


def _field(info, name):
    return info.get(name) if isinstance(info, Mapping) else getattr(info, name, None)


def get_board_profile(info):
    """Select an immutable profile by the exact INIT hardware identity, or None."""
    values = [_field(info, name) for name in
              ("board_model", "board_major", "board_minor", "board_patch")]
    if not all(type(value) is int and 0 <= value <= 255 for value in values):
        return None
    model, major, minor, patch = values
    return _profiles.get((model, f"{major}.{minor}.{patch}"))


def has_board_sensor(info, kind, index=0):
    """Check fitted hardware only, independently of firmware command support."""
    if not isinstance(kind, str) or type(index) is not int or not 0 <= index <= 255:
        return False
    profile = get_board_profile(info)
    return profile is not None and any(sensor["id"] == index
                                      for sensor in profile["sensors"].get(kind, ()))


def supports_board_sensor(info, kind, index=0):
    """Require both a fitted sensor and compatible firmware protocol commands."""
    required_minor = 4 if kind == "temperature" else 5 if kind == "imu" else None
    major, minor = _field(info, "protocol_major"), _field(info, "protocol_minor")
    return (required_minor is not None and type(major) is int and major == 2 and
            type(minor) is int and required_minor <= minor <= 255 and
            has_board_sensor(info, kind, index))


def supports_motor_current(info, index=0):
    """Require both a motor current sensor and protocol support."""
    profile = get_board_profile(info)
    major, minor = _field(info, 'protocol_major'), _field(info, 'protocol_minor')
    return (type(index) is int and type(major) is int and major == 2 and
            type(minor) is int and 7 <= minor <= 255 and profile is not None and
            any(motor['id'] == index and motor['current_sense'] for motor in profile['motor_features']))


def supports_motor_faults(info, index=0):
    """Require both motor-driver fault reporting and protocol support."""
    profile = get_board_profile(info)
    major, minor = _field(info, 'protocol_major'), _field(info, 'protocol_minor')
    return (type(index) is int and type(major) is int and major == 2 and
            type(minor) is int and 8 <= minor <= 255 and profile is not None and
            any(motor['id'] == index and motor['fault_reporting'] for motor in profile['motor_features']))


def get_motor_current_limit_options(info, index=0):
    """Immutable hardware options, independent of firmware support; None if unknown/absent."""
    profile = get_board_profile(info)
    if type(index) is not int or index < 0 or profile is None:
        return None
    return next((m.get('current_limit_options') for m in profile['motor_features'] if m['id'] == index), None)


def supports_motor_current_limit(info, index=0):
    """Require fitted current-limit hardware and firmware configuration commands."""
    major, minor = _field(info, 'protocol_major'), _field(info, 'protocol_minor')
    return (type(major) is int and major == 2 and type(minor) is int and 11 <= minor <= 255 and
            get_motor_current_limit_options(info, index) is not None)
