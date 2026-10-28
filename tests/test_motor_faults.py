"""Fault status remains distinct from unavailable driver diagnostics."""
import unittest
from pykinisi import (MotorFaultStatus, MotorFaultSample, Topic, GET_MOTOR_FAULTS,
                     ProtocolError, supports_motor_faults, CLEAR_MOTOR_FAULTS, ControllerError, ErrorCode, ERROR)
from pykinisi.KinisiCommands import KinisiCommands, decode_telemetry_event
import test_sensors
from test_sensors import frame


class FaultTests(unittest.TestCase):
    def test_codecs_and_index_validation(self):
        class Client(KinisiCommands):
            def _request(self, command, payload, size):
                self.call = (command, payload, size)
                return bytes([0, 1, 1, 0, 0, 0, 1, 0, 0, 0])
        client = Client()
        for index in range(4):
            status = client.get_motor_faults(index)
            self.assertTrue(status.status_valid and status.overcurrent)
            self.assertEqual(client.call, (GET_MOTOR_FAULTS, bytes([index]), 10))
        for index in (-1, 4, 255, .5, '0', True, None):
            with self.assertRaises(ValueError): client.get_motor_faults(index)
        for length in (0, 9, 11):
            with self.assertRaises(ValueError): MotorFaultStatus.decode(bytes(length))
        for offset in range(10):
            raw = bytearray(10); raw[offset] = 2
            with self.assertRaises(ValueError): MotorFaultStatus.decode(raw)

    def test_typed_event_and_unknown_diagnostics(self):
        sample = MotorFaultSample(2**55 + 7, 0, 1, False, True, True, False, False, False, True, False, False, False)
        event = bytes([Topic.MOTOR_FAULTS, 3]) + sample.encode()
        topic, index, decoded = decode_telemetry_event(event)
        self.assertEqual((topic, index, decoded.timestamp_us), (Topic.MOTOR_FAULTS, 3, sample.timestamp_us))
        self.assertTrue(decoded.overcurrent)
        self.assertFalse(MotorFaultStatus.decode(bytes(10)).status_valid)

    def test_session_gates_and_reads(self):
        for revision, protocol in ((4, 7), (1, 8), (2, 8), (3, 8), (99, 7)):
            client, transport, _ = test_sensors.SensorSessionTests.connect(self, minor=revision, protocol=protocol)
            count = len(transport.frames)
            self.assertFalse(client.supports_motor_faults())
            with self.assertRaises(ProtocolError): client.get_motor_faults(0)
            with self.assertRaises(ProtocolError): client.subscribe(Topic.MOTOR_FAULTS, lambda sample: None, interval_ms=100)
            self.assertEqual(len(transport.frames), count)
            client.disconnect()
        client, transport, board = test_sensors.SensorSessionTests.connect(self, protocol=8)
        self.assertTrue(supports_motor_faults(client.board_info, 3))
        board.command_hook = lambda cmd, mid, data: transport.inject(frame(cmd, mid, bytes([0, 1, 0, 0, 0, 0, int(data[0] == 2), 0, 0, 0])))
        for index in range(4):
            self.assertEqual(client.get_motor_faults(index).overcurrent, index == 2)
        client.disconnect()
        self.assertFalse(client.supports_motor_faults())


    def test_clear_fault_command_and_errors(self):
        client, transport, board = test_sensors.SensorSessionTests.connect(self, protocol=10)
        board.command_hook = lambda cmd, mid, data: transport.inject(frame(cmd, mid, bytes([0, 1, 1, 0, 0, 0, 1, 0, 0, 0])))
        for index in range(4):
            status = client.clear_motor_faults(index)
            self.assertTrue(status.status_valid and status.overcurrent)
            self.assertEqual(transport.frames[-1][1], CLEAR_MOTOR_FAULTS)
            self.assertEqual(transport.frames[-1][4:], bytes([index]))
        before = len(transport.frames)
        for index in (-1, 4, .5, True, '0'):
            with self.assertRaises(ValueError): client.clear_motor_faults(index)
        self.assertEqual(len(transport.frames), before)
        for code in (ErrorCode.MOTOR_OWNED, ErrorCode.MOTOR_NOT_INITIALIZED, ErrorCode.INTERNAL_ERROR):
            board.command_hook = lambda cmd, mid, data: transport.inject(frame(ERROR, mid, bytes([cmd, code])))
            with self.assertRaises(ControllerError) as error: client.clear_motor_faults(0)
            self.assertEqual(error.exception.error_code, code)
        client.disconnect()
        for revision, protocol in ((4, 9), (1, 10), (2, 10), (3, 10), (99, 9)):
            client, transport, _ = test_sensors.SensorSessionTests.connect(self, minor=revision, protocol=protocol)
            before = len(transport.frames)
            with self.assertRaises(ProtocolError): client.clear_motor_faults(0)
            self.assertEqual(len(transport.frames), before)
            client.disconnect()
