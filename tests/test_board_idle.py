"""Board-idle wire values and capability gates through the real serial session."""
import unittest
import struct
from unittest.mock import patch

from pykinisi import (KinisiController, KinisiCommands, SET_BOARD_IDLE, GET_BOARD_IDLE,
                     ERROR, ErrorCode, ControllerError, ProtocolError,
                     ConnectionClosedError)
import test_sensors
from test_controller import frame, controller_module
from test_controller import wait_until
from pykinisi import (Topic, SUBSCRIBE, SUBSCRIBE_WITH_MODE, SUBSCRIBE_CONFIGURED,
                     TELEMETRY_EVENT, ENCODER_ODOMETRY_EVENT, GET_TIME_STATUS,
                     TIME_SYNC_REQUEST, GET_DRIVER_SLEEP, SET_DRIVER_SLEEP, PING)
from test_generator import recording_client


class BoardIdleTests(unittest.TestCase):
    def connect(self, **kwargs):
        return test_sensors.SensorSessionTests.connect(self, **kwargs)

    def test_codec_rejects_invalid_input_and_reply(self):
        client = recording_client({'KinisiCommands': KinisiCommands})
        client.reply = b''
        for idle in (0, 1):
            self.assertIsNone(client.set_board_idle(idle))
            self.assertEqual(client.calls[-1], (0x5d, bytes([idle]), 0))
        before = len(client.calls)
        for idle in (-1, 2, 255, 256, True, False, .5, '1', None):
            with self.subTest(idle=idle), self.assertRaises(ValueError):
                client.set_board_idle(idle)
        self.assertEqual(len(client.calls), before)
        for idle in (0, 1):
            client.reply = bytes([idle])
            self.assertEqual(client.get_board_idle(), idle)
            self.assertEqual(client.calls[-1], (0x5e, b'', 1))
        for raw in (b'', b'\x00\x01', b'\x02', b'\xff'):
            client.reply = raw
            with self.subTest(reply=raw), self.assertRaises(ValueError):
                client.get_board_idle()

    def test_unsupported_profiles_and_old_protocol_never_send(self):
        for revision, protocol in ((1, 16), (2, 16), (3, 16), (4, 15), (99, 16)):
            with self.subTest(revision=revision, protocol=protocol):
                client, transport, _ = self.connect(minor=revision, protocol=protocol)
                self.assertFalse(client.supports_board_idle())
                before = len(transport.frames)
                with self.assertRaises(ProtocolError): client.set_board_idle(1)
                with self.assertRaises(ProtocolError): client.get_board_idle()
                self.assertEqual(len(transport.frames), before)
                client.disconnect()

    def test_ready_and_explicit_capability_required_before_io(self):
        client = KinisiController()
        self.assertFalse(client.supports_board_idle())
        with self.assertRaises(ConnectionClosedError): client.get_board_idle()
        client, transport, _ = self.connect(protocol=16, client=client)
        self.assertTrue(client.supports_board_idle())
        self.assertIs(client.board_profile['power']['board_idle'], True)
        before = len(transport.frames)
        client._ready = False
        self.assertFalse(client.supports_board_idle())
        with self.assertRaises(ProtocolError): client.set_board_idle(1)
        with self.assertRaises(ProtocolError): client.get_board_idle()
        client._ready = True
        with patch.object(controller_module, 'get_board_profile', return_value={'power': {'board_idle': False}}):
            self.assertFalse(client.supports_board_idle())
            with self.assertRaises(ProtocolError): client.get_board_idle()
        self.assertEqual(len(transport.frames), before)

    def test_ack_readback_repeated_requests_errors_and_reconnect(self):
        client, transport, board = self.connect(protocol=16)
        state = {'idle': 0, 'wake_failure': False}
        requests = []

        def respond(command, mid, payload):
            requests.append((command, payload))
            if command == SET_BOARD_IDLE:
                if payload == b'\x00' and state['wake_failure']:
                    transport.inject(frame(ERROR, mid, bytes([command, ErrorCode.INTERNAL_ERROR])))
                    return
                state['idle'] = payload[0]
            elif command == SET_DRIVER_SLEEP:
                transport.inject(frame(ERROR, mid, bytes([command, ErrorCode.BOARD_IDLE])))
                return
            transport.inject(frame(command, mid,
                bytes([state['idle']]) if command == GET_BOARD_IDLE else b''))

        board.command_hook = respond
        self.assertIsNone(client.set_board_idle(1))
        self.assertIsNone(client.set_board_idle(1))
        self.assertEqual(client.get_board_idle(), 1)
        self.assertEqual(requests[:3], [(SET_BOARD_IDLE, b'\x01')] * 2 + [(GET_BOARD_IDLE, b'')])
        with self.assertRaises(ControllerError) as error:
            client.set_driver_sleep(0)
        self.assertEqual(error.exception.error_code, ErrorCode.BOARD_IDLE)
        self.assertEqual(ErrorCode.BOARD_IDLE, 20)
        self.assertTrue(client.ready)
        state['wake_failure'] = True
        with self.assertRaises(ControllerError) as error:
            client.set_board_idle(0)
        self.assertEqual(error.exception.error_code, ErrorCode.INTERNAL_ERROR)
        self.assertEqual(client.get_board_idle(), 1)
        client.disconnect()
        self.assertFalse(client.supports_board_idle())
        client, transport, board = self.connect(protocol=16, client=client)
        self.assertFalse(any(raw[1] == SET_BOARD_IDLE for raw in transport.frames))
        board.command_hook = respond
        self.assertEqual(client.get_board_idle(), 1)
        state['wake_failure'] = False
        self.assertIsNone(client.set_board_idle(0))
        self.assertEqual(client.get_board_idle(), 0)

    def test_idle_preserves_cached_timestamps_and_subscriptions_until_fresh_events(self):
        client, transport, board = self.connect(protocol=16)
        received = []
        state = {'idle': 0}

        def respond(command, mid, payload):
            if command in (SUBSCRIBE, SUBSCRIBE_WITH_MODE, SUBSCRIBE_CONFIGURED) and state['idle']:
                transport.inject(frame(ERROR, mid, bytes([command, ErrorCode.BOARD_IDLE])))
                return
            if command == SET_BOARD_IDLE:
                state['idle'] = payload[0]
            reply = (bytes([state['idle']]) if command == GET_BOARD_IDLE else
                     b'\x01' if command == GET_DRIVER_SLEEP else
                     struct.pack('<BBIQ', 1, 1, 30000, 7) if command == GET_TIME_STATUS else b'')
            transport.inject(frame(command, mid, reply))

        def deliver(timestamp):
            sample = struct.pack('<QBBd', timestamp, 1, 1, 25.0)
            transport.inject(frame(TELEMETRY_EVENT, 0, bytes([Topic.TEMPERATURE, 0]) + sample))
            transport.inject(frame(ENCODER_ODOMETRY_EVENT, 0, b'\x00' + sample))

        board.command_hook = respond
        handle = client.subscribe(Topic.TEMPERATURE, received.append, interval_ms=250)
        client.subscribe_odometry(0, 100)
        deliver(100)
        wait_until(lambda: len(received) == 1 and client.get_subscription_sample(0) is not None)
        cached = client.get_subscription_sample(0)
        client.set_board_idle(1)
        self.assertTrue(handle.active)
        with self.assertRaises(ControllerError) as error:
            client.subscribe(Topic.TEMPERATURE, received.append, interval_ms=250)
        self.assertEqual(error.exception.error_code, ErrorCode.BOARD_IDLE)
        self.assertTrue(handle.active)  # Failed replacement retains the original stream.
        self.assertTrue(client.ready)
        self.assertEqual(client.get_time_status().clock_quality, 1)
        client.ping()
        before_sync = len(board.sync_replies)
        transport.inject(frame(TIME_SYNC_REQUEST, 90))
        wait_until(lambda: len(board.sync_replies) == before_sync + 1)
        self.assertIs(client.get_subscription_sample(0), cached)
        self.assertEqual(cached.timestamp_us, 100)
        self.assertEqual(len(received), 1)
        client.set_board_idle(0)
        self.assertIs(client.get_subscription_sample(0), cached)
        self.assertEqual(client.get_driver_sleep(), 1)  # Independent inhibit is untouched.
        self.assertFalse(any(raw[1] == SET_DRIVER_SLEEP for raw in transport.frames))
        deliver(200)
        wait_until(lambda: len(received) == 2 and client.get_subscription_sample(0).timestamp_us == 200)
        self.assertTrue(handle.active)
        handle.unsubscribe()

    def test_protocol_215_keeps_driver_sleep_available(self):
        client, _, _ = self.connect(protocol=15)
        self.assertTrue(client.supports_driver_sleep())
        self.assertFalse(client.supports_board_idle())
        client.set_driver_sleep(1)
