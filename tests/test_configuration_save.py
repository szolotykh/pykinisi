import struct
import unittest

from pykinisi import (ControllerError, ERROR, ErrorCode, GET_CONFIGURATION_FIELD,
                      ProtocolError, SAVE_CONFIGURATION)
import test_sensors
from test_controller import frame


class ConfigurationSaveTests(unittest.TestCase):
    def test_session_gate_preserves_unsupported_board(self):
        for revision, protocol in ((1, 14), (2, 14), (3, 14), (4, 13), (99, 14)):
            with self.subTest(revision=revision, protocol=protocol):
                client, transport, _ = test_sensors.SensorSessionTests.connect(self, minor=revision, protocol=protocol)
                self.assertFalse(client.supports_configuration_save())
                before = len(transport.frames)
                with self.assertRaises(ProtocolError):
                    client.save_configuration()
                with self.assertRaises(ProtocolError):
                    client.get_configuration_field(0, 0)
                self.assertEqual(len(transport.frames), before)
                client.disconnect()

    def test_ack_field_and_write_error(self):
        client, transport, board = test_sensors.SensorSessionTests.connect(self, protocol=14)
        self.assertTrue(client.supports_configuration_save())
        requests = []

        def respond(command, message_id, payload):
            requests.append((command, payload))
            value = struct.pack('<d', 1024) if command == GET_CONFIGURATION_FIELD else b''
            transport.inject(frame(command, message_id, value))

        board.command_hook = respond
        self.assertIsNone(client.save_configuration())
        self.assertEqual(client.get_configuration_field(4, 1), 1024)
        self.assertEqual(requests, [(SAVE_CONFIGURATION, b''), (GET_CONFIGURATION_FIELD, bytes([4, 1]))])
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(ERROR, mid, bytes([cmd, ErrorCode.INTERNAL_ERROR])))
        with self.assertRaises(ControllerError) as error:
            client.save_configuration()
        self.assertEqual(error.exception.error_code, ErrorCode.INTERNAL_ERROR)
