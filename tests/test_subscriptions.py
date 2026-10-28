from pykinisi import SubscriptionTrigger, SubscriptionDelivery
from pykinisi.KinisiCommands import SUBSCRIBE_CONFIGURED
"""Generic subscriptions through the real reader and callback worker."""
import struct
import threading
import unittest
from unittest.mock import patch
from pykinisi import KinisiController, Topic, SubscriptionMode, ProtocolError, RequestTimeoutError, ControllerError
from pykinisi.KinisiCommands import (SUBSCRIBE, SUBSCRIBE_WITH_MODE, UNSUBSCRIBE, TELEMETRY_EVENT, GET_ENCODER_VALUE,
                                    ERROR, SET_HEARTBEAT_CONFIG, decode_telemetry_event)
from test_controller import FakeSerial, Board, frame, wait_until, controller_module


class SubscriptionTests(unittest.TestCase):
    def setUp(self):
        self.transport = FakeSerial()
        self.board = Board(self.transport)
        self.board.protocol = (2, 9, 0)
        self.board.identity = (1, 0, 4, 0)
        self.client = KinisiController(request_timeout=0.12, init_timeout=0.5)
        patched = patch.object(controller_module.serial, 'Serial', return_value=self.transport)
        patched.start()
        self.addCleanup(patched.stop)
        self.addCleanup(self.client.disconnect)
        self.assertTrue(self.client.connect('TEST'), self.client.last_error)

    def event(self, topic=Topic.TEMPERATURE, index=0, value=25.0):
        if topic == Topic.MOTOR_FAULTS:
            payload = bytes((topic, index)) + struct.pack('<QBB10B', 2**55 + 7, 1, 1, 0, 1, 0, 0, 0, 0, int(value != 0), 0, 0, 0)
            return frame(TELEMETRY_EVENT, 0, payload)
        count = {Topic.ENCODER_ODOMETRY: 1, Topic.PLATFORM_ODOMETRY: 3, Topic.TEMPERATURE: 1, Topic.IMU: 6, Topic.MOTOR_CURRENT: 1}[topic]
        payload = bytes((topic, index)) + struct.pack('<QBB' + 'd' * count, 2**55 + 7, 1, 1, *([value] * count))
        return frame(TELEMETRY_EVENT, 0, payload)

    def test_configured_wire_snapshot_validation_and_gate(self):
        self.client.board_info.protocol_minor = 12
        def respond(command, mid, payload):
            response = frame(command, mid)
            if command == SUBSCRIBE_CONFIGURED:
                response += self.event(Topic(payload[0]), payload[1])
            self.transport.inject(response)
        self.board.command_hook = respond
        received = []
        options = dict(trigger=SubscriptionTrigger.ON_EVENT, delivery=SubscriptionDelivery.ON_CHANGE, max_silence_ms=5000)
        handle = self.client.subscribe(Topic.MOTOR_FAULTS, received.append, **options)
        wait_until(lambda: len(received) == 1)
        self.assertTrue(handle.active)
        self.assertIn((SUBSCRIBE_CONFIGURED, struct.pack('<BBIBBI', 6, 0, 0, 1, 1, 5000)), [(c, p) for c, _, p in self.board.commands])
        count = len(self.board.commands)
        for invalid in (dict(poll_interval_ms=100), dict(delivery=SubscriptionDelivery.ALL), dict(max_silence_ms=99), dict(interval_ms=100)):
            with self.assertRaises(ValueError):
                self.client.subscribe(Topic.MOTOR_FAULTS, received.append, **(options | invalid))
        self.assertEqual(len(self.board.commands), count)
        self.client.board_info.protocol_minor = 11
        with self.assertRaisesRegex(ProtocolError, '2.12'):
            self.client.subscribe(Topic.MOTOR_FAULTS, received.append, **options)
        self.assertEqual(len(self.board.commands), count)
        handle.unsubscribe()
        self.assertFalse(handle.active)

    def test_on_change_wire_immediate_snapshot_and_replacement(self):
        def respond(command, mid, payload):
            response = frame(command, mid)
            if command == SUBSCRIBE_WITH_MODE:
                response += self.event(Topic(payload[0]), payload[1])
            self.transport.inject(response)
        self.board.command_hook = respond
        received = []
        old = self.client.subscribe(Topic.MOTOR_FAULTS, received.append, interval_ms=100, mode=SubscriptionMode.ON_CHANGE)
        wait_until(lambda: len(received) == 1)
        self.assertTrue(old.active)
        self.assertIn((SUBSCRIBE_WITH_MODE, struct.pack('<BBIB', Topic.MOTOR_FAULTS, 0, 100, 1)),
                      [(c, p) for c, _, p in self.board.commands])
        current = self.client.subscribe(Topic.MOTOR_FAULTS, received.append, interval_ms=100)
        self.assertFalse(old.active)
        count = len(self.board.commands)
        old.unsubscribe()
        self.assertEqual(len(self.board.commands), count)
        current.unsubscribe()
        self.assertFalse(current.active)

    def test_on_change_validation_and_older_firmware(self):
        count = len(self.board.commands)
        for mode in (True, False, 2, -1, '1', 1.0, None):
            with self.assertRaises(ValueError):
                self.client.subscribe(Topic.MOTOR_FAULTS, lambda _: None, interval_ms=100, mode=mode)
        self.assertEqual(len(self.board.commands), count)
        self.client.board_info.protocol_minor = 8
        count = len(self.board.commands)
        with self.assertRaisesRegex(ProtocolError, '2.9'):
            self.client.subscribe(Topic.MOTOR_FAULTS, lambda _: None, interval_ms=100, mode=SubscriptionMode.ON_CHANGE)
        self.assertEqual(len(self.board.commands), count)
        handle = self.client.subscribe(Topic.MOTOR_FAULTS, lambda _: None, interval_ms=100)
        self.assertTrue(handle.active)
        handle.unsubscribe()

    def test_four_fault_streams_and_unsubscribe(self):
        self.board.command_hook = lambda cmd, mid, payload: self.transport.inject(frame(cmd, mid))
        received = [[], [], [], []]
        handles = [self.client.subscribe(Topic.MOTOR_FAULTS, received[i].append, index=i, interval_ms=100) for i in range(4)]
        for i in range(4): self.transport.inject(self.event(Topic.MOTOR_FAULTS, i, i))
        wait_until(lambda: all(received))
        for i in range(4):
            self.assertTrue(received[i][0].status_valid)
            self.assertEqual(received[i][0].overcurrent, i != 0)
            self.assertEqual(received[i][0].timestamp_us, 2**55 + 7)
        for handle in handles: handle.unsubscribe()
        self.assertTrue(all(not handle.active for handle in handles))
        self.client.disconnect()
        self.assertFalse(self.client.supports_topic(Topic.MOTOR_FAULTS))

    def test_all_topics_golden_wire_and_immediate_samples(self):
        def respond(command, mid, payload):
            self.transport.inject(frame(command, mid))
            if command == SUBSCRIBE:
                self.transport.inject(self.event(Topic(payload[0]), payload[1]))
        self.board.command_hook = respond
        for topic, interval in ((Topic.ENCODER_ODOMETRY, 100), (Topic.PLATFORM_ODOMETRY, 100),
                                (Topic.TEMPERATURE, 250), (Topic.IMU, 10), (Topic.MOTOR_CURRENT, 20)):
            received = []
            self.assertTrue(self.client.supports_topic(topic))
            handle = self.client.subscribe(topic, received.append, interval_ms=interval)
            self.assertTrue(handle.active)
            wait_until(lambda: len(received) == 1)
            self.assertEqual(received[0].timestamp_us, 2**55 + 7)
            self.assertIn((SUBSCRIBE, struct.pack('<BBI', topic, 0, interval)),
                          [(c, p) for c, _, p in self.board.commands])
            handle.unsubscribe()
            self.assertFalse(handle.active)

    def test_replacement_and_old_handle(self):
        first, second = [], []
        old = self.client.subscribe(Topic.TEMPERATURE, first.append, interval_ms=250)
        new = self.client.subscribe(Topic.TEMPERATURE, second.append, interval_ms=500)
        before = len(self.board.commands)
        old.unsubscribe()
        self.assertEqual(len(self.board.commands), before)
        self.assertFalse(old.active)
        self.transport.inject(self.event())
        wait_until(lambda: len(second) == 1)
        self.assertEqual(first, [])
        self.client.set_heartbeat_config(False, 500)
        self.assertFalse(new.active)

    def test_current_stream_rejects_negative_or_nonfinite_samples(self):
        for value in (-1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                decode_telemetry_event(self.event(Topic.MOTOR_CURRENT, 3, value)[4:])
        topic, index, sample = decode_telemetry_event(self.event(Topic.MOTOR_CURRENT, 3, 0)[4:])
        self.assertEqual((topic, index, sample.amps, sample.timestamp_us), (Topic.MOTOR_CURRENT, 3, 0, 2**55 + 7))

    def test_slow_callback_does_not_block_requests_and_coalesces(self):
        started, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        received = []
        def callback(sample):
            received.append(sample.celsius)
            started.set()
            release.wait(2)
        self.client.subscribe(Topic.TEMPERATURE, callback, interval_ms=250)
        self.transport.inject(self.event(value=1))
        self.assertTrue(started.wait(1))
        for value in range(2, 21):
            self.transport.inject(self.event(value=value))
        self.assertEqual(self.client.get_encoder_value(0), 65530)
        release.set()
        wait_until(lambda: len(received) == 2)
        self.assertEqual(received, [1, 20])

    def test_gating_and_validation_before_write(self):
        for topic, index, interval in [(False, 0, 250), (99, 0, 250), (Topic.PLATFORM_ODOMETRY, 1, 100),
                                       (Topic.IMU, 0, 9), (Topic.TEMPERATURE, 0, 249)]:
            before = len(self.board.commands)
            with self.assertRaises(ValueError):
                self.client.subscribe(topic, lambda s: None, index=index, interval_ms=interval)
            self.assertEqual(len(self.board.commands), before)
        self.client.board_info.protocol_minor = 5
        self.assertFalse(self.client.supports_topic(Topic.IMU))
        with self.assertRaises(ProtocolError):
            self.client.subscribe(Topic.IMU, lambda s: None, interval_ms=10)
        self.client.board_info.protocol_minor = 6
        self.client.board_info.board_minor = 3
        with self.assertRaises(ProtocolError):
            self.client.subscribe(Topic.TEMPERATURE, lambda s: None, interval_ms=250)

    def test_timeout_cancels_unknown_subscription(self):
        def respond(command, mid, payload):
            if command != SUBSCRIBE:
                self.transport.inject(frame(command, mid))
        self.board.command_hook = respond
        with self.assertRaises(RequestTimeoutError):
            self.client.subscribe(Topic.TEMPERATURE, lambda s: None, interval_ms=250)
        self.assertTrue(any(c == UNSUBSCRIBE for c, _, _ in self.board.commands))
        self.assertTrue(self.client.ready)
        self.assertEqual(self.client._telemetry, {})

    def test_disconnect_invalidates_handle_and_callbacks(self):
        handle = self.client.subscribe(Topic.TEMPERATURE, lambda s: None, interval_ms=250)
        self.client.disconnect()
        self.assertFalse(handle.active)
        handle.unsubscribe()

    def test_rejected_replacement_preserves_owner_and_callback_failures_are_isolated(self):
        seen = []
        def callback(sample):
            seen.append(sample.celsius)
            raise RuntimeError('application callback failed')
        handle = self.client.subscribe(Topic.TEMPERATURE, callback, interval_ms=250)
        self.board.command_hook = lambda command, mid, payload: self.transport.inject(
            frame(ERROR, mid, bytes((command, 17))) if command == SUBSCRIBE else frame(command, mid))
        with self.assertRaises(ControllerError):
            self.client.subscribe(Topic.TEMPERATURE, lambda s: None, interval_ms=500)
        self.assertTrue(handle.active)
        self.transport.inject(self.event())
        wait_until(lambda: self.client.last_telemetry_error is not None)
        self.assertEqual(seen, [25.0])
        self.assertTrue(self.client.ready)
        self.board.command_hook = None
        self.assertEqual(self.client.get_encoder_value(0), 65530)

    def test_old_handle_cannot_cancel_new_connection(self):
        old = self.client.subscribe(Topic.TEMPERATURE, lambda s: None, interval_ms=250)
        transport = FakeSerial()
        board = Board(transport)
        board.protocol, board.identity = (2, 6, 0), (1, 0, 4, 0)
        with patch.object(controller_module.serial, 'Serial', return_value=transport):
            self.assertTrue(self.client.connect('NEW'), self.client.last_error)
        current = self.client.subscribe(Topic.TEMPERATURE, lambda s: None, interval_ms=250)
        before = len(board.commands)
        old.unsubscribe()
        self.assertFalse(old.active)
        self.assertTrue(current.active)
        self.assertEqual(len(board.commands), before)

    def test_reconnect_between_subscription_creation_and_send(self):
        transport = FakeSerial()
        board = Board(transport)
        board.protocol, board.identity = (2, 6, 0), (1, 0, 4, 0)
        original = controller_module.KinisiCommands.subscribe
        def interrupted(client, topic, index, interval):
            with patch.object(controller_module.serial, 'Serial', return_value=transport):
                self.assertTrue(client.connect('NEW'), client.last_error)
            return original(client, topic, index, interval)
        with patch.object(controller_module.KinisiCommands, 'subscribe', new=interrupted):
            with self.assertRaises(controller_module.ConnectionClosedError):
                self.client.subscribe(Topic.TEMPERATURE, lambda s: None, interval_ms=250)
        self.assertFalse(any(c == SUBSCRIBE for c, _, _ in board.commands))
        self.assertTrue(self.client.ready)

    def test_decoder_rejects_bad_samples_and_ignores_future_topics(self):
        self.assertIsNone(decode_telemetry_event(b'\xff\x00some future sample'))
        for payload in (b'', b'\x03', self.event()[4:-1], self.event(value=float('nan'))[4:]):
            with self.assertRaises(ValueError):
                decode_telemetry_event(payload)
        self.transport.inject(frame(TELEMETRY_EVENT, 1, self.event()[4:]))
        wait_until(lambda: not self.client.ready)
