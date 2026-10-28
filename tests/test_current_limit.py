"""Current-limit wire values, profile lookup and pre-I/O validation."""
import unittest
from pykinisi import (GET_MOTOR_CURRENT_LIMIT, SET_MOTOR_CURRENT_LIMIT, MotorCurrentLimit,
                     ProtocolError, ControllerError, ErrorCode, ERROR)
from pykinisi.KinisiCommands import KinisiCommands
import test_sensors
from test_sensors import frame


class CurrentLimitTests(unittest.TestCase):
    def test_wire_encoding_and_invalid_values(self):
        class Client(KinisiCommands):
            def _request(self, command, payload, size):
                self.call = (command, payload, size)
                return bytes([1, 0x18, 0x15])
        c = Client()
        for index in range(4):
            self.assertEqual(vars(c.set_motor_current_limit(index, True, 5400)), dict(enabled=True, threshold_ma=5400))
            self.assertEqual(c.call, (SET_MOTOR_CURRENT_LIMIT, bytes([index, 1, 0x18, 0x15]), 3))
            c.get_motor_current_limit(index)
            self.assertEqual(c.call, (GET_MOTOR_CURRENT_LIMIT, bytes([index]), 3))
        for index in (-1, 4, True, .5):
            with self.assertRaises(ValueError): c.get_motor_current_limit(index)
        for value in (0, 65536, True, 5400.5, '5400'):
            with self.assertRaises(ValueError): c.set_motor_current_limit(0, True, value)
        for value in (0, 1, 'True'):
            with self.assertRaises(ValueError): c.set_motor_current_limit(0, value, 5400)
        for raw in (b'', b'\1\0', b'\1\0\0', bytes([2, 0x18, 0x15]), bytes([1, 0x18, 0x15, 0])):
            with self.assertRaises(ValueError): MotorCurrentLimit.decode(raw)

    def test_profile_options_and_runtime_validation(self):
        c, transport, board = test_sensors.SensorSessionTests.connect(self, protocol=11)
        for index in range(4):
            self.assertTrue(c.supports_motor_current_limit(index))
            options = c.get_motor_current_limit_options(index)
            self.assertEqual(options['supported_thresholds_ma'], (4000, 5400, 6500, 7000))
            with self.assertRaises(TypeError): options['can_disable'] = False
        board.command_hook = lambda cmd, mid, data: transport.inject(frame(cmd, mid, bytes([1, 0x18, 0x15])))
        before = len(transport.frames)
        with self.assertRaises(ValueError): c.set_motor_current_limit(0, True, 5000)
        self.assertEqual(len(transport.frames), before)
        c.set_motor_current_limit(3, False, 5400)
        self.assertEqual(transport.frames[-1][4:], bytes([3, 0, 0x18, 0x15]))
        for code in (ErrorCode.INTERNAL_ERROR, ErrorCode.MOTOR_OWNED, ErrorCode.MOTOR_NOT_INITIALIZED):
            board.command_hook = lambda cmd, mid, data: transport.inject(frame(ERROR, mid, bytes([cmd, code])))
            with self.assertRaises(ControllerError): c.set_motor_current_limit(0, True, 5400)
        c.disconnect()
        self.assertIsNone(c.get_motor_current_limit_options())

    def test_old_firmware_absent_and_unknown_boards(self):
        for revision, protocol in ((4, 10), (1, 11), (2, 11), (3, 11), (99, 10)):
            c, transport, _ = test_sensors.SensorSessionTests.connect(self, minor=revision, protocol=protocol)
            before = len(transport.frames)
            self.assertFalse(c.supports_motor_current_limit())
            with self.assertRaises(ProtocolError): c.get_motor_current_limit(0)
            with self.assertRaises(ProtocolError): c.set_motor_current_limit(0, True, 5400)
            self.assertEqual(len(transport.frames), before)
            c.disconnect()
        c, transport, board = test_sensors.SensorSessionTests.connect(self, minor=99, protocol=11)
        self.assertIsNone(c.get_motor_current_limit_options())
        board.command_hook = lambda cmd, mid, data: transport.inject(frame(cmd, mid, bytes([1, 0x88, 0x13])))
        # An unknown profile cannot advertise capabilities; explicit requests still reach firmware.
        self.assertEqual(c.set_motor_current_limit(0, True, 5000).threshold_ma, 5000)
        c.disconnect()
