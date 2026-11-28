"""Omni layouts through the public, version-negotiated serial interface."""
import struct
import unittest

import test_sensors
from test_controller import frame
from pykinisi import (ControllerError, ERROR, ErrorCode, GET_CONFIGURATION_FIELD,
                      OmniWheelOrientation, PROTOCOL_VERSION, ProtocolError)


class OmniOrientationTests(unittest.TestCase):
    connect = test_sensors.SensorSessionTests.connect
    geometry = (True, False, True, False, True, False, .10, .17, 4096.)

    def test_legacy_firmware_and_original_initializer(self):
        for version in (1, 25):
            client, transport, board = self.connect(minor=3, protocol=version)
            before = len(transport.frames)
            self.assertIs(client.get_omni_wheel_orientation(), OmniWheelOrientation.REAR)
            with self.assertRaises(ProtocolError):
                client.initialize_omni_platform_oriented(*self.geometry, OmniWheelOrientation.FRONT)
            self.assertEqual(len(transport.frames), before)
            board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid))
            client.initialize_omni_platform(*self.geometry)
            self.assertEqual(transport.frames[-1][0:2], bytes((33, 0x31)))
            self.assertEqual(transport.frames[-1][4:], struct.pack('<6?3d', *self.geometry))

    def test_both_applied_layouts_and_wire_bytes(self):
        self.assertGreaterEqual(PROTOCOL_VERSION, (2, 26, 0))
        for minor in (3, 4):
            client, transport, board = self.connect(minor=minor, protocol=26)
            for layout in OmniWheelOrientation:
                board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid))
                client.initialize_omni_platform_oriented(*self.geometry, layout)
                request = transport.frames[-1]
                self.assertEqual(request[:2], bytes((34, 0x88)))
                self.assertEqual(request[4:], struct.pack('<6?3dB', *self.geometry, layout))
                board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid, struct.pack('<d', layout)))
                self.assertIs(client.get_omni_wheel_orientation(), layout)
                self.assertEqual(transport.frames[-1][1], GET_CONFIGURATION_FIELD)
                self.assertEqual(transport.frames[-1][4:], bytes((8, 0)))
            if minor == 3:
                with self.assertRaises(ProtocolError):
                    client.get_configuration_field(8, 1)

    def test_rejections(self):
        client, transport, board = self.connect(protocol=26)
        before = len(transport.frames)
        for invalid in (True, 1.0, 2, -1):
            with self.assertRaises(ValueError):
                client.initialize_omni_platform_oriented(*self.geometry, invalid)
        self.assertEqual(len(transport.frames), before)
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid, struct.pack('<d', 0.5)))
        with self.assertRaises(ProtocolError):
            client.get_omni_wheel_orientation()
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(ERROR, mid, bytes((cmd, ErrorCode.INVALID_ARGUMENT))))
        with self.assertRaises(ControllerError):
            client.get_omni_wheel_orientation()
