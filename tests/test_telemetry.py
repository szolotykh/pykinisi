# Filename: test_telemetry.py
# Description: Verify idle heartbeat scheduling and unsolicited sample delivery.
"""Exercise connection features through the serial transport and public SDK API."""
import struct
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import patch

from pykinisi import KinisiController
from pykinisi.KinisiCommands import PING, SET_HEARTBEAT_CONFIG, ENCODER_ODOMETRY_EVENT, PLATFORM_ODOMETRY_EVENT, GET_ENCODER_VALUE
from test_controller import FakeSerial, Board, frame, wait_until, controller_module


class TelemetryTests(unittest.TestCase):
    """Keep real reader/heartbeat threads while replacing only the serial device."""

    def setUp(self):
        """Connect with the new default watchdog configuration."""
        self.transport = FakeSerial()
        self.board = Board(self.transport)
        self.client = KinisiController(request_timeout=0.2, init_timeout=0.5)
        mock_serial = patch.object(controller_module.serial, 'Serial', return_value=self.transport)
        mock_serial.start()
        self.addCleanup(mock_serial.stop)
        self.addCleanup(self.client.disconnect)
        self.assertTrue(self.client.connect('TEST'), self.client.last_error)

    def test_idle_ping_and_regular_traffic(self):
        """Configure 500 ms, send after idle, and suppress pings during normal traffic."""
        config = next(p for command, _, p in self.board.commands if command == SET_HEARTBEAT_CONFIG)
        self.assertEqual(config, struct.pack('<BI', 1, 500))
        wait_until(lambda: any(c == PING for c, _, _ in self.board.commands))
        before = sum(c == PING for c, _, _ in self.board.commands)
        for _ in range(8):
            self.client.get_encoder_value(0)
            time.sleep(0.025)
        self.assertEqual(sum(c == PING for c, _, _ in self.board.commands), before)
        wait_until(lambda: sum(c == PING for c, _, _ in self.board.commands) > before)
        self.client.set_heartbeat_config(False, 500)
        before = len(self.board.commands)
        time.sleep(0.15)
        self.assertEqual(len(self.board.commands), before)

    def _check_delayed_config_caller(self, reconnect=False):
        """Hold an acknowledged caller while a newer configuration takes effect."""
        acknowledged = threading.Event()
        release = threading.Event()
        original = controller_module._PendingRequest

        class DelayedEvent(threading.Event):
            """Delay the waiting caller without blocking the serial receive thread."""

            def wait(self, timeout=None):
                """Expose receipt of the ACK before allowing the caller to return."""
                result = super().wait(timeout)
                acknowledged.set()
                if not release.wait(5):
                    raise AssertionError('Delayed caller was not released')
                return result

        def pending_request(*args, **kwargs):
            """Delay only the worker's configuration request, leaving other I/O live."""
            pending = original(*args, **kwargs)
            if args[0] == SET_HEARTBEAT_CONFIG and threading.current_thread().name.startswith('older-config'):
                pending.event = DelayedEvent()
            return pending

        with patch.object(controller_module, '_PendingRequest', side_effect=pending_request), \
                ThreadPoolExecutor(max_workers=1, thread_name_prefix='older-config') as executor:
            older = executor.submit(self.client.set_heartbeat_config, True, 10000)
            try:
                self.assertTrue(acknowledged.wait(1), 'Configuration ACK was not received')
                if reconnect:
                    new_transport = FakeSerial()
                    self.board = Board(new_transport)
                    with patch.object(controller_module.serial, 'Serial', return_value=new_transport):
                        self.assertTrue(self.client.connect('NEW'), self.client.last_error)
                else:
                    self.client.set_heartbeat_config(True, 500)
            finally:
                release.set()
            older.result(timeout=1)
        configs = [p for c, _, p in self.board.commands if c == SET_HEARTBEAT_CONFIG]
        self.assertEqual(struct.unpack('<BI', configs[-1]), (1, 500))
        self.assertEqual(self.client._heartbeat_idle_s, 0.1)

    def test_config_ack_order_survives_delayed_caller(self):
        """An older caller cannot restore a two-second cadence over a 500 ms watchdog."""
        self._check_delayed_config_caller()

    def test_old_config_caller_cannot_change_reconnected_session(self):
        """An ACK from the old connection cannot change the new session's cadence."""
        self._check_delayed_config_caller(reconnect=True)

    def test_events_interleave_with_replies_and_replace_cache(self):
        """Unsolicited ID zero never satisfies a normal pending request."""
        self.client.subscribe_odometry(0, 100)
        def respond(command, message_id, payload):
            """Send samples before the command ACK to stress demultiplexing."""
            for timestamp in (123, 456):
                self.transport.inject(frame(ENCODER_ODOMETRY_EVENT, 0, struct.pack('<BQBBd', 0, timestamp, 1, 1, 1.25)))
            self.transport.inject(frame(PLATFORM_ODOMETRY_EVENT, 0, struct.pack('<QBBddd', 789, 1, 1, 1, 2, 3)))
            self.transport.inject(frame(command, message_id, struct.pack('<H', 77) if command == GET_ENCODER_VALUE else b''))
        self.board.command_hook = respond
        self.assertEqual(self.client.get_encoder_value(0), 77)
        self.assertEqual(self.client.get_subscription_sample(0).timestamp_us, 456)
        self.assertEqual(self.client.get_subscription_sample(4).timestamp_us, 789)
        self.assertEqual(len(self.client._subscription_samples), 2)
        self.client.unsubscribe_odometry(0)
        self.assertIsNone(self.client.get_subscription_sample(0))
        self.client.disconnect()
        self.assertIsNone(self.client.get_subscription_sample(4))

    def test_missing_ping_ack_closes_session(self):
        """A background worker must not quietly keep a dead connection ready."""
        self.board.command_hook = lambda *args: None
        wait_until(lambda: not self.client.ready)
        self.assertIsNotNone(self.client.last_error)

    def test_invalid_event_closes_session(self):
        """Reject truncated unsolicited telemetry before storing it."""
        self.transport.inject(frame(ENCODER_ODOMETRY_EVENT, 0, b'bad'))
        wait_until(lambda: not self.client.ready)
        self.assertIsNone(self.client.get_subscription_sample(0))
