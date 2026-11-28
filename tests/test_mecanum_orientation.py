"""Mecanum initialization and readback through the public serial session."""
import struct
import unittest

from pykinisi import (ERROR, GET_CONFIGURATION_FIELD, ControllerError, ErrorCode,
                      MecanumRollerOrientation, PROTOCOL_VERSION, ProtocolError)
from test_controller import frame
import test_sensors


class MecanumOrientationTests(unittest.TestCase):
    connect = test_sensors.SensorSessionTests.connect
    geometry = (True, False, True, False, False, True, False, True, .34, .26, .1, 4096.)

    def test_legacy_firmware_defaults_without_probe(self):
        for version in (1, 14, 24):
            client, transport, board = self.connect(minor=3, protocol=version)
            before = len(transport.frames)
            self.assertEqual(client.get_mecanum_roller_orientation(), MecanumRollerOrientation.STANDARD)
            with self.assertRaises(ProtocolError):
                client.initialize_mecanum_platform_oriented(*self.geometry, MecanumRollerOrientation.MIRRORED)
            self.assertEqual(len(transport.frames), before)
            board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid))
            client.initialize_mecanum_platform(*self.geometry)
            self.assertEqual(transport.frames[-1][1], 0x30)
            self.assertEqual(transport.frames[-1][4:], struct.pack('<8?4d', *self.geometry))

    def test_oriented_frames_and_applied_readback(self):
        self.assertGreaterEqual(PROTOCOL_VERSION, (2, 25, 0))
        for minor in (3, 4):
            client, transport, board = self.connect(minor=minor, protocol=25)
            for layout in MecanumRollerOrientation:
                board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid))
                client.initialize_mecanum_platform_oriented(*self.geometry, layout)
                request = transport.frames[-1]
                self.assertEqual((request[0], request[1]), (44, 0x87))
                self.assertEqual(request[4:], struct.pack('<8?4dB', *self.geometry, layout))
                board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid, struct.pack('<d', layout)))
                self.assertIs(client.get_mecanum_roller_orientation(), layout)
                self.assertEqual(transport.frames[-1][1], GET_CONFIGURATION_FIELD)
                self.assertEqual(transport.frames[-1][4:], bytes([7, 0]))
            if minor == 3:
                with self.assertRaises(ProtocolError):
                    client.get_configuration_field(0, 0)

    def test_bad_inputs_readbacks_and_firmware_errors(self):
        client, transport, board = self.connect(protocol=25)
        before = len(transport.frames)
        for bad in (True, 2, -1, 1.0):
            with self.assertRaises(ValueError):
                client.initialize_mecanum_platform_oriented(*self.geometry, bad)
        self.assertEqual(len(transport.frames), before)
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid, struct.pack('<d', 0.5)))
        with self.assertRaises(ProtocolError):
            client.get_mecanum_roller_orientation()
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(ERROR, mid, bytes([cmd, ErrorCode.INVALID_ARGUMENT])))
        with self.assertRaises(ControllerError) as error:
            client.initialize_mecanum_platform_oriented(*self.geometry, MecanumRollerOrientation.MIRRORED)
        self.assertEqual(error.exception.error_code, ErrorCode.INVALID_ARGUMENT)
