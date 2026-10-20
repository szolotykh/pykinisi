"""Sensor payloads and profile/protocol gates through the production serial session."""

from concurrent.futures import ThreadPoolExecutor
import struct
import unittest
from unittest.mock import patch

from pykinisi import (KinisiController, KinisiCommands, ImuSample, ErrorCode,
                     ControllerError, ProtocolError, GET_TEMPERATURE, GET_IMU, ERROR)
from test_controller import FakeSerial, Board, frame, wait_until, controller_module
from test_generator import recording_client


VALUES = (-1.25, 0.0, 9.80665, 0.125, -0.25, 0.5)
FIELDS = tuple(field + '_' + axis for field in ('acceleration', 'angular_velocity') for axis in 'xyz')


class SensorCodecTests(unittest.TestCase):
    def setUp(self):
        self.client = recording_client({'KinisiCommands': KinisiCommands})

    def test_signed_measurements_and_request_bytes(self):
        for value in (-40.0, -0.0625, 0.0, 25.0625, 125.0):
            self.client.reply = struct.pack('<d', value)
            self.assertEqual(self.client.get_temperature(0), value)
            self.assertEqual(self.client.calls[-1], (0x50, b'\x00', 8))
        self.client.reply = struct.pack('<6d', *VALUES)
        sample = self.client.get_imu(255)
        self.assertIsInstance(sample, ImuSample)
        self.assertEqual(tuple(getattr(sample, field) for field in FIELDS), VALUES)
        self.assertEqual(self.client.calls[-1], (0x51, b'\xff', 48))

    def test_invalid_indices_fail_before_io(self):
        for method in (self.client.get_temperature, self.client.get_imu):
            for index in (-1, 256, 0.5, '0', None, True, float('nan'), float('inf')):
                with self.subTest(method=method.__name__, index=index), self.assertRaises(ValueError):
                    method(index)
        self.assertFalse(self.client.calls)

    def test_wrong_lengths_and_nonfinite_values_are_rejected(self):
        for method, size in ((self.client.get_temperature, 8), (self.client.get_imu, 48)):
            for length in (0, size - 1, size + 1):
                self.client.reply = bytes(length)
                with self.assertRaisesRegex(ValueError, 'payload bytes'):
                    method(0)
            for field in range(size // 8):
                for invalid in (float('nan'), float('inf'), -float('inf')):
                    values = [0.0] * (size // 8)
                    values[field] = invalid
                    self.client.reply = struct.pack('<' + 'd' * len(values), *values)
                    with self.assertRaisesRegex(ValueError, 'Invalid .* payload'):
                        method(0)


class SensorSessionTests(unittest.TestCase):
    def connect(self, minor=4, protocol=5, client=None):
        transport = FakeSerial()
        board = Board(transport)
        board.identity = (1, 0, minor, 0)
        board.protocol = (2, protocol, 0)
        client = client or KinisiController(request_timeout=0.25, init_timeout=0.5, heartbeat_timeout_ms=None)
        self.addCleanup(client.disconnect)
        with patch.object(controller_module.serial, 'Serial', return_value=transport):
            self.assertTrue(client.connect('TEST'), str(client.last_error))
        return client, transport, board

    def test_automatic_profile_readings_and_disconnect(self):
        client, transport, board = self.connect()
        self.assertEqual(client.board_profile['board']['revision'], '0.4.0')
        self.assertTrue(client.supports_sensor('temperature'))
        self.assertTrue(client.supports_sensor('imu'))
        board.command_hook = lambda command, mid, payload: transport.inject(frame(command, mid,
            struct.pack('<d', -0.0625) if command == GET_TEMPERATURE else struct.pack('<6d', *VALUES)))
        self.assertEqual(client.get_temperature(0), -0.0625)
        self.assertEqual(client.get_imu(0).acceleration_z, VALUES[2])
        self.assertEqual(board.commands[-1][2], b'\x00')
        client.disconnect()
        self.assertIsNone(client.board_profile)
        self.assertIsNone(client.board_info)
        self.assertIsNone(client.clock_mode)
        self.assertFalse(client.supports_sensor('imu'))
        client, _, _ = self.connect(minor=3, client=client)
        self.assertEqual(client.board_profile['board']['revision'], '0.3.0')
        self.assertFalse(client.supports_sensor('imu'))

    def test_old_protocol_blocks_only_unsupported_sensors_before_io(self):
        for protocol in (1, 2, 3, 4):
            client, transport, board = self.connect(protocol=protocol)
            count = len(transport.frames)
            with self.assertRaisesRegex(ProtocolError, '2.5'):
                client.get_imu(0)
            if protocol < 4:
                with self.assertRaisesRegex(ProtocolError, '2.4'):
                    client.get_temperature(0)
            self.assertEqual(len(transport.frames), count)
            self.assertEqual(client.get_encoder_value(0), 65530)
            client.disconnect()

    def test_known_absent_sensor_or_index_is_rejected_without_io(self):
        for minor in (1, 2, 3, 4):
            client, transport, _ = self.connect(minor=minor)
            count = len(transport.frames)
            index = 1 if minor == 4 else 0
            for method in (client.get_temperature, client.get_imu):
                with self.assertRaisesRegex(ProtocolError, 'no .* sensor'):
                    method(index)
            self.assertEqual(len(transport.frames), count)
            self.assertEqual(client.get_encoder_value(0), 65530)
            client.disconnect()

    def test_unknown_board_keeps_connection_and_low_level_reads(self):
        client, transport, board = self.connect(minor=99)
        self.assertIsNone(client.board_profile)
        self.assertFalse(client.supports_sensor('imu'))
        self.assertEqual(client.get_encoder_value(0), 65530)
        board.command_hook = lambda command, mid, payload: transport.inject(frame(command, mid, struct.pack('<6d', *VALUES)))
        self.assertEqual(client.get_imu(0).angular_velocity_z, VALUES[-1])

    def test_sensor_errors_preserve_correlation_and_session(self):
        client, transport, board = self.connect()
        for method, command in ((client.get_temperature, GET_TEMPERATURE), (client.get_imu, GET_IMU)):
            for code in (ErrorCode.SENSOR_NOT_AVAILABLE, ErrorCode.SAMPLE_NOT_AVAILABLE, ErrorCode.INTERNAL_ERROR):
                board.command_hook = lambda cmd, mid, payload: transport.inject(frame(ERROR, mid, bytes((cmd, code))))
                with self.assertRaises(ControllerError) as caught:
                    method(0)
                self.assertEqual(caught.exception.command, command)
                self.assertEqual(caught.exception.error_code, code)
                self.assertEqual(caught.exception.message_id, board.commands[-1][1])
                board.command_hook = None
                self.assertTrue(client.ready)
                self.assertIsNone(client.ping())

    def test_identity_does_not_allow_read_before_ready(self):
        transport = FakeSerial()
        board = Board(transport)
        board.identity, board.protocol = (1, 0, 4, 0), (2, 5, 0)
        board.hold_ready = True
        client = KinisiController(init_timeout=1.0, heartbeat_timeout_ms=None)
        self.addCleanup(client.disconnect)
        with patch.object(controller_module.serial, 'Serial', return_value=transport), ThreadPoolExecutor() as pool:
            connection = pool.submit(client.connect, 'TEST')
            wait_until(lambda: client.board_info is not None)
            count = len([c for c in board.commands if c[0] in (GET_TEMPERATURE, GET_IMU)])
            with self.assertRaisesRegex(ProtocolError, 'ready'):
                client.get_imu(0)
            self.assertEqual(len([c for c in board.commands if c[0] in (GET_TEMPERATURE, GET_IMU)]), count)
            wait_until(lambda: board.initial_samples == 3)
            board.ready()
            self.assertTrue(connection.result())


if __name__ == '__main__':
    unittest.main()
