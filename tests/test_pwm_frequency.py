"""Board-wide PWM enum, wire encoding and feature gating."""
import struct
import unittest
from pykinisi import (PwmFrequency, GET_PWM_FREQUENCY, SET_PWM_FREQUENCY, ProtocolError,
                     supports_pwm_frequency, ControllerError, ErrorCode, ERROR)
from types import SimpleNamespace
from pykinisi.KinisiCommands import KinisiCommands
import test_sensors
from test_sensors import frame


class PwmFrequencyTests(unittest.TestCase):
    def test_wire_and_enum_validation(self):
        class Client(KinisiCommands):
            def _request(self, command, payload, size):
                self.call = (command, payload, size)
                return bytes([0xa8, 0x61, 0, 0])
        c = Client()
        self.assertEqual(c.set_pwm_frequency(PwmFrequency.KHZ_25), 25000)
        self.assertEqual(c.call, (SET_PWM_FREQUENCY, bytes([0xa8, 0x61, 0, 0]), 4))
        self.assertEqual(c.get_pwm_frequency(), 25000)
        self.assertEqual(c.call, (GET_PWM_FREQUENCY, b'', 4))
        for value in (0, 12345, 25000.5, True, '25000', 2**32 + 25000):
            with self.assertRaises(ValueError): c.set_pwm_frequency(value)

    def test_capability_accepts_mappings_and_rejects_malformed_identity(self):
        info = dict(board_model=1, board_major=0, board_minor=4, board_patch=0, protocol_major=2, protocol_minor=13)
        self.assertTrue(supports_pwm_frequency(info))
        self.assertTrue(supports_pwm_frequency(SimpleNamespace(**info)))
        for identity in (None, {}, SimpleNamespace(), {**info, 'protocol_minor': '13'},
                         {**info, 'protocol_minor': True}, {**info, 'protocol_minor': 256},
                         {**info, 'protocol_major': '2'}, {**info, 'board_minor': 99}):
            self.assertFalse(supports_pwm_frequency(identity))

    def test_pwm_commands_reject_lost_readiness_before_io(self):
        c, transport, board = test_sensors.SensorSessionTests.connect(self, protocol=13)
        board.command_hook = lambda cmd, mid, data: transport.inject(frame(ERROR, mid, bytes([cmd, ErrorCode.CLOCK_NOT_READY])))
        with self.assertRaises(ControllerError): c.get_pwm_frequency()
        self.assertFalse(c.ready)
        self.assertIsNotNone(c.board_info)
        before = len(transport.frames)
        with self.assertRaises(ProtocolError): c.get_pwm_frequency()
        with self.assertRaises(ProtocolError): c.set_pwm_frequency(PwmFrequency.KHZ_25)
        self.assertEqual(len(transport.frames), before)
        c.disconnect()

    def test_profiles_protocol_gate_and_reconnect(self):
        for revision in (1, 2, 3, 4):
            c, transport, board = test_sensors.SensorSessionTests.connect(self, minor=revision, protocol=13)
            self.assertTrue(c.supports_pwm_frequency())
            options = c.get_pwm_frequency_options()
            self.assertEqual(options['supported_hz'], tuple(PwmFrequency))
            with self.assertRaises(TypeError): options['default_hz'] = 25000
            board.command_hook = lambda cmd, mid, data: transport.inject(frame(cmd, mid, struct.pack('<I', 25000)))
            self.assertEqual(c.set_pwm_frequency(PwmFrequency.KHZ_25), 25000)
            self.assertEqual(c.get_pwm_frequency(), 25000)
            c.disconnect()
            self.assertIsNone(c.get_pwm_frequency_options())
        c, transport, _ = test_sensors.SensorSessionTests.connect(self, protocol=12)
        before = len(transport.frames)
        self.assertFalse(c.supports_pwm_frequency())
        with self.assertRaises(ProtocolError): c.get_pwm_frequency()
        with self.assertRaises(ProtocolError): c.set_pwm_frequency(PwmFrequency.KHZ_25)
        self.assertEqual(len(transport.frames), before)
        c.disconnect()
        c, transport, board = test_sensors.SensorSessionTests.connect(self, minor=99, protocol=13)
        self.assertFalse(c.supports_pwm_frequency())
        self.assertIsNone(c.get_pwm_frequency_options())
        board.command_hook = lambda cmd, mid, data: transport.inject(frame(cmd, mid, struct.pack('<I', 25000)))
        self.assertEqual(c.get_pwm_frequency(), 25000)
        c.disconnect()
