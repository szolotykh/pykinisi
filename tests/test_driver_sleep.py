"""Driver-sleep wire values and capability gates through the real serial session."""
import unittest
from unittest.mock import patch

from pykinisi import (KinisiController, KinisiCommands, SET_DRIVER_SLEEP, GET_DRIVER_SLEEP,
                     SET_MOTOR_SPEED, ERROR, ErrorCode, ControllerError, ProtocolError,
                     ConnectionClosedError)
import test_sensors
from test_controller import frame, controller_module
from test_generator import recording_client


class DriverSleepTests(unittest.TestCase):
    def connect(self, **kwargs):
        return test_sensors.SensorSessionTests.connect(self, **kwargs)

    def test_revision_specific_protocol_gate_and_nano_roundtrip(self):
        cases = ((3, 0, 16, False), (3, 1, 15, False), (3, 1, 16, False),
                 (3, 0, 17, True), (3, 1, 17, True), (3, 1, 18, True),
                 (3, 2, 17, False), (4, 1, 17, False), (99, 0, 17, False),
                 (1, 0, 17, False), (2, 0, 17, False), (4, 0, 15, True))
        for minor, patch, protocol, supported in cases:
            with self.subTest(minor=minor, patch=patch, protocol=protocol):
                client, transport, board = self.connect(minor=minor, board_patch=patch, protocol=protocol)
                self.assertEqual(client.supports_driver_sleep(), supported)
                before = len(transport.frames)
                if not supported:
                    with self.assertRaises(ProtocolError): client.set_driver_sleep(1)
                    with self.assertRaises(ProtocolError): client.get_driver_sleep()
                    self.assertEqual(len(transport.frames), before)
                    client.disconnect()
                    continue
                state = {'sleeping': 0}
                def respond(command, mid, payload):
                    if command == SET_DRIVER_SLEEP:
                        state['sleeping'] = payload[0]
                    transport.inject(frame(command, mid, bytes([state['sleeping']]) if command == GET_DRIVER_SLEEP else b''))
                board.command_hook = respond
                for sleeping in (1, 1, 0, 0):
                    self.assertIsNone(client.set_driver_sleep(sleeping))
                    self.assertEqual(client.get_driver_sleep(), sleeping)
                self.assertEqual([raw[1] for raw in transport.frames[before:]], [SET_DRIVER_SLEEP, GET_DRIVER_SLEEP] * 4)
                if minor == 3:
                    self.assertFalse(client.supports_board_idle())
                client.board_info.protocol_major = 3
                before = len(transport.frames)
                self.assertFalse(client.supports_driver_sleep())
                with self.assertRaises(ProtocolError): client.set_driver_sleep(0)
                self.assertEqual(len(transport.frames), before)
                client.disconnect()

    def test_superseded_nano_wake_preserves_device_error(self):
        client, transport, board = self.connect(minor=3, board_patch=1, protocol=17)
        board.command_hook = lambda command, mid, payload: transport.inject(
            frame(ERROR, mid, bytes([command, ErrorCode.DRIVERS_ASLEEP])))
        with self.assertRaises(ControllerError) as error:
            client.set_driver_sleep(0)
        self.assertEqual(error.exception.error_code, ErrorCode.DRIVERS_ASLEEP)
        self.assertTrue(client.ready)

    def test_codec_rejects_invalid_input_and_reply(self):
        client = recording_client({'KinisiCommands': KinisiCommands})
        client.reply = b''
        for sleeping in (0, 1):
            self.assertIsNone(client.set_driver_sleep(sleeping))
            self.assertEqual(client.calls[-1], (0x5b, bytes([sleeping]), 0))
        before = len(client.calls)
        for sleeping in (-1, 2, 255, 256, True, False, .5, '1', None):
            with self.subTest(sleeping=sleeping), self.assertRaises(ValueError):
                client.set_driver_sleep(sleeping)
        self.assertEqual(len(client.calls), before)
        for sleeping in (0, 1):
            client.reply = bytes([sleeping])
            self.assertEqual(client.get_driver_sleep(), sleeping)
            self.assertEqual(client.calls[-1], (0x5c, b'', 1))
        for raw in (b'', b'\x00\x01', b'\x02', b'\xff'):
            client.reply = raw
            with self.subTest(reply=raw), self.assertRaises(ValueError):
                client.get_driver_sleep()

    def test_unsupported_profiles_and_old_protocol_never_send(self):
        for revision, protocol in ((1, 15), (2, 15), (3, 15), (4, 14), (99, 15)):
            with self.subTest(revision=revision, protocol=protocol):
                client, transport, _ = self.connect(minor=revision, protocol=protocol)
                self.assertFalse(client.supports_driver_sleep())
                before = len(transport.frames)
                with self.assertRaises(ProtocolError): client.set_driver_sleep(1)
                with self.assertRaises(ProtocolError): client.get_driver_sleep()
                self.assertEqual(len(transport.frames), before)
                client.disconnect()

    def test_ready_and_explicit_capability_required_before_io(self):
        client = KinisiController()
        self.assertFalse(client.supports_driver_sleep())
        with self.assertRaises(ConnectionClosedError): client.get_driver_sleep()
        client, transport, _ = self.connect(protocol=15, client=client)
        self.assertTrue(client.supports_driver_sleep())
        self.assertIs(client.board_profile['power']['driver_sleep'], True)
        before = len(transport.frames)
        client._ready = False
        self.assertFalse(client.supports_driver_sleep())
        with self.assertRaises(ProtocolError): client.set_driver_sleep(1)
        with self.assertRaises(ProtocolError): client.get_driver_sleep()
        client._ready = True
        with patch.object(controller_module, 'get_board_profile', return_value={'power': {'driver_sleep': False}}):
            self.assertFalse(client.supports_driver_sleep())
            with self.assertRaises(ProtocolError): client.get_driver_sleep()
        self.assertEqual(len(transport.frames), before)

    def test_ack_readback_repeated_requests_errors_and_reconnect(self):
        client, transport, board = self.connect(protocol=15)
        state = {'sleeping': 0, 'wake_failure': False}
        requests = []

        def respond(command, mid, payload):
            requests.append((command, payload))
            if command == SET_DRIVER_SLEEP:
                if payload == b'\x00' and state['wake_failure']:
                    transport.inject(frame(ERROR, mid, bytes([command, ErrorCode.INTERNAL_ERROR])))
                    return
                state['sleeping'] = payload[0]
            elif command == SET_MOTOR_SPEED:
                transport.inject(frame(ERROR, mid, bytes([command, ErrorCode.DRIVERS_ASLEEP])))
                return
            transport.inject(frame(command, mid,
                bytes([state['sleeping']]) if command == GET_DRIVER_SLEEP else b''))

        board.command_hook = respond
        self.assertIsNone(client.set_driver_sleep(1))
        self.assertIsNone(client.set_driver_sleep(1))
        self.assertEqual(client.get_driver_sleep(), 1)
        self.assertEqual(requests[:3], [(SET_DRIVER_SLEEP, b'\x01')] * 2 + [(GET_DRIVER_SLEEP, b'')])
        with self.assertRaises(ControllerError) as error:
            client.set_motor_speed(0, 10)
        self.assertEqual(error.exception.error_code, ErrorCode.DRIVERS_ASLEEP)
        self.assertEqual(ErrorCode.DRIVERS_ASLEEP, 19)
        self.assertTrue(client.ready)
        state['wake_failure'] = True
        with self.assertRaises(ControllerError) as error:
            client.set_driver_sleep(0)
        self.assertEqual(error.exception.error_code, ErrorCode.INTERNAL_ERROR)
        self.assertEqual(client.get_driver_sleep(), 1)
        client.disconnect()
        self.assertFalse(client.supports_driver_sleep())
        client, transport, board = self.connect(protocol=15, client=client)
        self.assertFalse(any(raw[1] == SET_DRIVER_SLEEP for raw in transport.frames))
        board.command_hook = respond
        self.assertEqual(client.get_driver_sleep(), 1)
        state['wake_failure'] = False
        self.assertIsNone(client.set_driver_sleep(0))
        self.assertEqual(client.get_driver_sleep(), 0)
