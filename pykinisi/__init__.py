# Filename: __init__.py
# Description: Public SDK commands, sample types, controller, and structured errors.
"""Python SDK for Kinisi protocol-v2 firmware."""
from ._version import __version__
from .KinisiCommands import *
from .KinisiController import (
    KinisiController, MotorIndex, EncoderIndex, GPIOIndex, GPIOMode, State,
    ClockMode, ClockQuality,
)
from .errors import KinisiError, ConnectionClosedError, ProtocolError, RequestTimeoutError, ControllerError
from .board_profile import get_board_profile, has_board_sensor, supports_board_sensor

__author__ = 'Sergey Zolotykh'

__all__ = [
    'KinisiController', 'MotorIndex', 'EncoderIndex', 'GPIOIndex', 'GPIOMode', 'State',
    'ClockMode', 'ClockQuality', 'ErrorCode', 'InitResponse', 'TimeStatus', 'HeartbeatConfig',
    'EncoderOdometrySample', 'PlatformOdometrySample', 'ImuSample', 'KinisiError',
    'get_board_profile', 'has_board_sensor', 'supports_board_sensor',
    'ConnectionClosedError', 'ProtocolError', 'RequestTimeoutError', 'ControllerError',
    '__version__',
]
