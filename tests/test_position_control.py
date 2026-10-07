"""Position payload order, version gates and velocity defaults without hardware."""
import math
import struct
import unittest
from unittest.mock import patch

from pykinisi import KinisiController, KinisiCommands, ProtocolError
from test_controller import FakeSerial, Board, controller_module


class MotionCommandsTests(unittest.TestCase):
    def test_payloads_defaults_and_velocity_validation(self):
        client = KinisiCommands()
        calls = []
        def request(command, payload, size):
            calls.append((command, payload, size))
            return struct.pack('<d', 12.5) if size == 8 else b''
        client._request = request
        client.initialize_motor_controller(0, False, 1, False, 1425.1, 1, 1, 0)
        self.assertEqual(calls[-1][1], struct.pack('<B?B?ddddd', 0, False, 1, False, 1425.1, 1, 1, 0, 100))
        client.start_platform_controller(1, 1, 0)
        self.assertEqual(calls[-1][1], struct.pack('<dddd', 1, 1, 0, 100))
        client.initialize_motor_position_pid_controller(2, 2, 3, .02, .1, .2, 3)
        self.assertEqual(calls[-1], (0x10, struct.pack('<B6d', 2, 2, 3, .02, .1, .2, 3), 0))
        values = (1, 2, .2, .5, .01, .03, .1, .2, .3, .4, .5, .6)
        client.initialize_platform_position_pid_controller(*values)
        self.assertEqual(calls[-1], (0x4e, struct.pack('<12d', *values), 0))
        client.set_motor_position(2, -4 * math.pi)
        self.assertEqual(calls[-1], (0x0e, struct.pack('<Bd', 2, -4 * math.pi), 0))
        self.assertEqual(client.get_motor_position(2), 12.5)
        client.reset_motor_position(2)
        self.assertEqual(calls[-1], (0x0d, b'\x02', 0))
        client.set_platform_position(1, -2, .5)
        self.assertEqual(calls[-1], (0x4d, struct.pack('<3d', 1, -2, .5), 0))
        client.reset_platform_position()
        self.assertEqual(calls[-1], (0x4c, b'', 0))
        before = len(calls)
        for gains in [(1, 1, 0, 101), (1, 1, 0, -1), (1, math.nan, 0, 100), (math.inf, 1, 0, 100)]:
            with self.assertRaises(ValueError): client.start_platform_controller(*gains)
            with self.assertRaises(ValueError): client.initialize_motor_controller(0, False, 0, False, 1000, *gains)
        self.assertEqual(len(calls), before)

    def test_version_gates_preserve_legacy_connections(self):
        for minor in (1, 2, 3):
            with self.subTest(minor=minor):
                transport = FakeSerial()
                board = Board(transport)
                board.protocol = (2, minor, 1)
                with patch.object(controller_module.serial, 'Serial', return_value=transport):
                    client = KinisiController(wall_clock=False, heartbeat_timeout_ms=None)
                    try:
                        self.assertTrue(client.connect('fake'))
                        before = len(board.commands)
                        if minor < 2:
                            with self.assertRaises(ProtocolError): client.set_motor_position(0, 1)
                            with self.assertRaises(ProtocolError): client.reset_platform_position()
                            self.assertEqual(len(board.commands), before)
                        else:
                            client.set_motor_position(0, 1)
                            client.reset_platform_position()
                        before = len(board.commands)
                        if minor < 3:
                            with self.assertRaises(ProtocolError): client.initialize_motor_position_pid_controller(0, 2, 1, .02, 0, 0, 1)
                            with self.assertRaises(ProtocolError): client.initialize_platform_position_pid_controller(1, 2, .2, .5, .01, .03, 0, 0, .2, 0, 0, .5)
                            self.assertEqual(len(board.commands), before)
                        else:
                            client.initialize_motor_position_pid_controller(0, 2, 1, .02, 0, 0, 1)
                            client.initialize_platform_position_pid_controller(1, 2, .2, .5, .01, .03, 0, 0, .2, 0, 0, .5)
                        self.assertTrue(client.ready)
                    finally:
                        client.disconnect()
