# Filename: test_controller.py
# Description: Exercise the real serial reader, caller concurrency, framing, and clock handshake.
"""Protocol-v2 integration tests with a fragmented in-memory serial peer."""

from concurrent.futures import ThreadPoolExecutor
import importlib
import struct
import threading
import time
import unittest
from unittest.mock import patch

from pykinisi import (
    KinisiController, ClockMode, ControllerError, ErrorCode, ProtocolError,
    RequestTimeoutError, ConnectionClosedError, INIT, READY, ERROR,
    TIME_SYNC_REQUEST, TIME_SYNC_RESPONSE, GET_ENCODER_VALUE,
    GET_ENCODER_ODOMETRY, GET_PLATFORM_ODOMETRY, GET_TIME_STATUS,
    GET_GPIO_PIN_STATE, SET_GPIO_PIN_STATE, STOP_MOTOR,
)

controller_module = importlib.import_module("pykinisi.KinisiController")


def frame(command, message_id, payload=b""):
    """Build golden protocol frames without using the SDK's encoder."""
    return struct.pack("<BBH", 3 + len(payload), command, message_id) + payload


def wait_until(predicate, timeout=1.0):
    """Wait for an asynchronous fixture event with a finite test deadline."""
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("Asynchronous fixture did not reach the expected state")
        time.sleep(0.002)


class FakeSerial:
    """Return partial reads/writes while preserving real blocking-reader behavior."""

    def __init__(self):
        """Create a byte stream and independently configurable transport failures."""
        self.condition = threading.Condition()
        self.input = bytearray()
        self.output = bytearray()
        self.frames = []
        self.on_frame = None
        self.read_chunk = 3
        self.write_chunk = 255
        self.write_timeout = 0.25
        self.closed = False
        self.read_error = None
        self.write_error = None

    def inject(self, data):
        """Make board bytes available to the SDK's background reader."""
        with self.condition:
            self.input.extend(data)
            self.condition.notify_all()

    def reset_input_buffer(self):
        """Discard traffic from a previous connection before starting INIT."""
        with self.condition:
            self.input.clear()

    def read(self, size):
        """Block briefly like pyserial, then return at most the requested fragment."""
        with self.condition:
            if not self.input and not self.closed and self.read_error is None:
                self.condition.wait(0.01)
            if self.read_error is not None:
                raise self.read_error
            if self.closed:
                return b""
            count = min(size, len(self.input), self.read_chunk)
            result = bytes(self.input[:count])
            del self.input[:count]
            return result

    def write(self, data):
        """Accumulate host fragments and dispatch only fully written request frames."""
        if self.closed:
            raise OSError("Port closed")
        if self.write_error is not None:
            raise self.write_error
        count = min(len(data), self.write_chunk)
        self.output.extend(data[:count])
        while self.output and len(self.output) >= self.output[0] + 1:
            size = self.output[0] + 1
            request = bytes(self.output[:size])
            del self.output[:size]
            self.frames.append(request)
            self.on_frame(request)
        return count

    def cancel_read(self):
        """Wake the reader so disconnect can join it promptly."""
        with self.condition:
            self.condition.notify_all()

    def cancel_write(self):
        """Writes are immediate in this fixture."""

    def close(self):
        """Close the fake descriptor and wake blocked reads."""
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class Board:
    """Speak the wire protocol independently, with hooks for reordered/missing replies."""

    def __init__(self, transport):
        """Default to identity, three timing samples, then READY for wall-clock clients."""
        self.transport = transport
        transport.on_frame = self.receive
        self.commands = []
        self.sync_replies = []
        self.init_payloads = []
        self.init_id = None
        self.mode = 1
        self.initial_samples = 0
        self.hold_ready = False
        self.identity_only = False
        self.init_error = None
        self.command_hook = None
        self.sample_time = (1 << 55) + 123

    def ready(self):
        """Deliver the final response to the latest INIT."""
        self.transport.inject(frame(READY, self.init_id, bytes([self.mode])))

    def receive(self, request):
        """Interpret INIT/sync and generate golden normal replies or a test override."""
        command, message_id = struct.unpack("<BH", request[1:4])
        payload = request[4:]
        self.commands.append((command, message_id, payload))
        if command == INIT:
            self.init_id = message_id
            self.init_payloads.append(payload)
            if self.init_error is not None:
                self.transport.inject(frame(ERROR, message_id, bytes([INIT, self.init_error])))
                return
            self.mode = payload[7] & 1
            self.initial_samples = 0
            identity = struct.pack("<7BII", 1, 0, 3, 0, 2, 0, 0, 0x12345678, 0x90ABCDEF)
            self.transport.inject(frame(INIT, message_id, identity))
            if self.identity_only:
                return
            if self.mode:
                self.transport.inject(frame(TIME_SYNC_REQUEST, 1))
            elif not self.hold_ready:
                self.ready()
        elif command == TIME_SYNC_RESPONSE:
            h2, h3 = struct.unpack("<QQ", payload)
            self.sync_replies.append((message_id, h2, h3))
            if self.initial_samples < 3:
                self.initial_samples += 1
                if self.initial_samples < 3:
                    self.transport.inject(frame(TIME_SYNC_REQUEST, self.initial_samples + 1))
                elif not self.hold_ready:
                    self.ready()
            # Deliberately send no ACK to a TIME_SYNC_RESPONSE.
        elif self.command_hook is not None:
            self.command_hook(command, message_id, payload)
        else:
            responses = {
                GET_ENCODER_VALUE: struct.pack("<H", 65530),
                GET_GPIO_PIN_STATE: b"\x01",
                GET_ENCODER_ODOMETRY: struct.pack("<QBBd", self.sample_time, self.mode, 1, 1.25),
                GET_PLATFORM_ODOMETRY: struct.pack("<QBBddd", self.sample_time, self.mode, 2, 1, 2, 3),
                GET_TIME_STATUS: struct.pack("<BBIQ", self.mode, 1, 30000, 7),
            }
            self.transport.inject(frame(command, message_id, responses.get(command, b"")))


class ControllerTests(unittest.TestCase):
    """Run public SDK calls through actual framing, reader, and pending-request code."""

    def setUp(self):
        """Install a fresh in-memory serial port without opening hardware."""
        self.transport = FakeSerial()
        self.board = Board(self.transport)
        self.controller = KinisiController(request_timeout=0.25, init_timeout=0.5)
        self.factory = patch.object(controller_module.serial, "Serial", return_value=self.transport)
        self.factory.start()
        self.addCleanup(self.factory.stop)
        self.addCleanup(self.controller.disconnect)

    def connect(self):
        """Fail a test with the SDK's real handshake error if setup cannot complete."""
        self.assertTrue(self.controller.connect("TEST"), str(self.controller.last_error))

    def test_wall_clock_init_fragmentation_and_idle_periodic_sync(self):
        """Connect waits for three samples; idle sync is answered without a public GET."""
        self.transport.read_chunk = 1
        self.transport.write_chunk = 2
        self.connect()
        self.assertEqual(self.board.init_payloads, [bytes([1, 2, 0, 0, 2, 0, 0, 1])])
        self.assertTrue(self.controller.ready)
        self.assertEqual(self.controller.clock_mode, ClockMode.WALL)
        self.assertEqual(self.controller.board_info.board_minor, 3)
        self.assertEqual(self.controller.board_info.firmware_build_high, 0x12345678)
        self.assertEqual([reply[0] for reply in self.board.sync_replies], [1, 2, 3])
        self.transport.inject(frame(TIME_SYNC_REQUEST, 500))
        wait_until(lambda: len(self.board.sync_replies) == 4)
        message_id, h2, h3 = self.board.sync_replies[-1]
        self.assertEqual(message_id, 500)
        self.assertGreater(h2, 1000000000000000)
        self.assertGreaterEqual(h3, h2)
        self.assertIsNone(self.controller.stop_motor(0))

    def test_uptime_and_timestamped_samples(self):
        """Uptime clients skip sync and both odometry results preserve full uint64 precision."""
        self.controller.wall_clock = False
        self.connect()
        self.assertFalse(self.board.sync_replies)
        self.assertEqual(self.controller.clock_mode, ClockMode.UPTIME)
        encoder = self.controller.get_encoder_odometry(0)
        pose = self.controller.get_platform_odometry()
        self.assertEqual(encoder.timestamp_us, self.board.sample_time)
        self.assertEqual((encoder.clock_mode, encoder.clock_quality, encoder.angle), (0, 1, 1.25))
        self.assertEqual(pose.timestamp_us, self.board.sample_time)
        self.assertEqual((pose.x, pose.y, pose.t, pose.clock_quality), (1, 2, 3, 2))

    def test_connect_waits_for_ready_and_gates_odometry(self):
        """Identity alone does not release connect or permit odometry requests."""
        self.board.hold_ready = True
        with ThreadPoolExecutor(1) as pool:
            connected = pool.submit(self.controller.connect, "TEST")
            wait_until(lambda: len(self.board.sync_replies) == 3)
            self.assertFalse(connected.done())
            with self.assertRaises(ProtocolError):
                self.controller.start_platform_odometry()
            self.assertFalse(any(command == GET_PLATFORM_ODOMETRY for command, _, _ in self.board.commands))
            self.board.ready()
            self.assertTrue(connected.result(timeout=1))

    def test_init_errors_and_missing_ready_close_connection(self):
        """Controller setup failures and missing READY cannot masquerade as a successful connect."""
        self.board.init_error = ErrorCode.INCOMPATIBLE_PROTOCOL
        self.assertFalse(self.controller.connect("TEST"))
        self.assertIsInstance(self.controller.last_error, ControllerError)
        self.assertTrue(self.transport.closed)
        self.assertFalse(self.controller.ready)
        replacement = FakeSerial()
        board = Board(replacement)
        board.identity_only = True
        with patch.object(controller_module.serial, "Serial", return_value=replacement):
            self.assertFalse(self.controller.connect("TEST"))
        self.assertIsInstance(self.controller.last_error, RequestTimeoutError)
        self.assertTrue(replacement.closed)

    def test_setter_ack_shared_error_and_sync_id_collision(self):
        """Consume setter ACKs and keep controller-owned sync IDs separate from application IDs."""
        self.connect()
        self.assertIsNone(self.controller.set_gpio_pin_state(0, 1))
        self.assertEqual(self.controller.get_encoder_value(0), 65530)
        def replies(command, message_id, payload):
            """Interleave a sync request/error sharing the normal request's numeric ID."""
            self.transport.inject(frame(TIME_SYNC_REQUEST, message_id))
            self.transport.inject(frame(ERROR, message_id, bytes([TIME_SYNC_RESPONSE, ErrorCode.INVALID_ARGUMENT])))
            self.transport.inject(frame(ERROR, message_id, bytes([command, ErrorCode.MOTOR_OWNED])))
        self.board.command_hook = replies
        with self.assertRaises(ControllerError) as caught:
            self.controller.stop_motor(0)
        self.assertEqual(caught.exception.command, STOP_MOTOR)
        self.assertEqual(caught.exception.error_code, ErrorCode.MOTOR_OWNED)
        self.assertEqual(self.controller.last_sync_error.command, TIME_SYNC_RESPONSE)
        self.assertTrue(self.controller.ready)

    def test_readiness_error_requires_a_new_handshake(self):
        """Lost board readiness blocks further odometry until INIT/READY succeeds again."""
        self.connect()
        for code in (ErrorCode.INIT_REQUIRED, ErrorCode.CLOCK_NOT_READY):
            with self.subTest(code=code):
                self.board.command_hook = lambda command, message_id, payload: self.transport.inject(
                    frame(ERROR, message_id, bytes([command, code])))
                with self.assertRaises(ControllerError) as caught:
                    self.controller.get_encoder_odometry(0)
                self.assertEqual(caught.exception.error_code, code)
                self.assertFalse(self.controller.ready)
                before = len(self.board.commands)
                with self.assertRaises(ProtocolError):
                    self.controller.get_encoder_odometry(0)
                self.assertEqual(len(self.board.commands), before)
                self.board.command_hook = None
                self.controller.init(1, 2, 0, 0, 2, 0, 0, 1)
                self.assertTrue(self.controller.ready)
                self.assertEqual(self.controller.get_encoder_odometry(0).angle, 1.25)

    def test_concurrent_replies_out_of_order(self):
        """Two callers receive their own payload even when the board reverses reply order."""
        self.connect()
        requests = []
        def reverse(command, message_id, payload):
            """Wait for both writes before replying in reverse order."""
            requests.append((command, message_id, payload))
            if len(requests) == 2:
                for code, identifier, arguments in reversed(requests):
                    self.transport.inject(frame(code, identifier, struct.pack("<H", 100 + arguments[0])))
        self.board.command_hook = reverse
        with ThreadPoolExecutor(2) as pool:
            first = pool.submit(self.controller.get_encoder_value, 0)
            second = pool.submit(self.controller.get_encoder_value, 1)
            self.assertEqual(first.result(timeout=1), 100)
            self.assertEqual(second.result(timeout=1), 101)

    def test_timeout_late_reply_id_wrap_and_no_automatic_retry(self):
        """Retire timed-out IDs and discard their late responses instead of matching another call."""
        self.connect()
        self.board.command_hook = lambda *args: None
        before = len(self.board.commands)
        with self.assertRaises(RequestTimeoutError):
            self.controller.stop_motor(0)
        self.assertEqual(len(self.board.commands), before + 1)
        timed_out = self.board.commands[-1][1]
        self.transport.inject(frame(STOP_MOTOR, timed_out))
        self.controller._message_id = timed_out - 1
        self.board.command_hook = None
        self.assertEqual(self.controller.get_encoder_value(0), 65530)
        self.assertNotEqual(self.board.commands[-1][1], timed_out)
        self.controller._message_id = 65535
        self.assertEqual(self.controller.get_encoder_value(0), 65530)
        self.assertEqual(self.board.commands[-1][1], 1)

    def test_disconnect_unblocks_waiters_and_reconnects(self):
        """Closing a session wakes a blocked caller and a fresh session starts IDs at one."""
        self.connect()
        self.board.command_hook = lambda *args: None
        with ThreadPoolExecutor(1) as pool:
            pending = pool.submit(self.controller.get_encoder_value, 0)
            wait_until(lambda: self.board.commands[-1][0] == GET_ENCODER_VALUE)
            old_reader = self.controller._reader
            self.controller.disconnect()
            with self.assertRaises(ConnectionClosedError):
                pending.result(timeout=1)
            self.assertFalse(old_reader.is_alive())
        replacement = FakeSerial()
        board = Board(replacement)
        with patch.object(controller_module.serial, "Serial", return_value=replacement):
            self.assertTrue(self.controller.connect("NEW"))
        self.assertEqual(board.init_id, 1)
        self.assertTrue(self.controller.ready)

    def test_malformed_reply_fails_session(self):
        """Reject a framed but incorrectly sized payload rather than returning corrupted values."""
        self.connect()
        self.board.command_hook = lambda command, message_id, payload: self.transport.inject(
            frame(command, message_id, b"\x01"))
        with self.assertRaises(ProtocolError):
            self.controller.get_encoder_value(0)
        self.assertFalse(self.controller.ready)
        wait_until(lambda: self.transport.closed)

    def test_truncated_frame_is_not_spliced_into_later_replies(self):
        """An incomplete frame has a finite parsing deadline even while the application is idle."""
        self.connect()
        self.transport.inject(b"\x10\x75")
        wait_until(lambda: self.controller.serial is None)
        self.assertIsInstance(self.controller.last_error, ProtocolError)

    def test_serial_failure_fails_callers(self):
        """A write failure closes the stream instead of reusing potentially partial framing."""
        self.connect()
        self.transport.write_error = OSError("cable removed")
        with self.assertRaises(ConnectionClosedError):
            self.controller.stop_motor(0)
        self.assertFalse(self.controller.ready)
        self.assertTrue(self.transport.closed)

    def test_cancelled_partial_write_is_not_resumed(self):
        """Windows cancel_write may return a positive byte count; never send its remainder."""
        self.connect()
        entered, cancelled = threading.Event(), threading.Event()
        writes = []
        def partial_write(data):
            """Model a Windows serial write interrupted after its first two bytes."""
            writes.append(bytes(data))
            entered.set()
            if not cancelled.wait(1):
                raise AssertionError("The pending write was not cancelled")
            return min(2, len(data))
        self.transport.write = partial_write
        self.transport.cancel_write = cancelled.set
        with ThreadPoolExecutor(1) as pool:
            pending = pool.submit(self.controller.stop_motor, 0)
            self.assertTrue(entered.wait(1))
            self.controller.disconnect()
            with self.assertRaises(ConnectionClosedError):
                pending.result(timeout=1)
        self.assertEqual(len(writes), 1)

    def test_wrong_ready_order_and_mode_are_rejected(self):
        """READY cannot bypass identity validation or silently change the negotiated clock domain."""
        for mode in (0, 1):
            transport = FakeSerial()
            def malformed(request):
                """Send READY before identity, or after identity with the wrong mode."""
                command, message_id = struct.unpack("<BH", request[1:4])
                if mode == 0:
                    identity = struct.pack("<7BII", 1, 0, 3, 0, 2, 0, 0, 0, 0)
                    transport.inject(frame(command, message_id, identity))
                transport.inject(frame(READY, message_id, bytes([mode])))
            transport.on_frame = malformed
            with patch.object(controller_module.serial, "Serial", return_value=transport):
                self.assertFalse(self.controller.connect("BAD"))
            self.assertIsInstance(self.controller.last_error, ProtocolError)
            self.assertTrue(transport.closed)


if __name__ == "__main__":
    unittest.main()
