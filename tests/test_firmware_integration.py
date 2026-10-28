"""Run the Python SDK against the actual C firmware session when locally available.

The fixture stubs hardware operations only. INIT validation, identity, framed ACKs
and errors, READY gating, and periodic clock synchronization use production C.
An isolated pykinisi checkout or a machine without a host C compiler skips these
checks; the ordinary Python unit tests do not require a firmware checkout.
"""

import ctypes
import importlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
import unittest
from unittest import mock

from pykinisi import KinisiController, Topic, SubscriptionMode, SubscriptionTrigger, SubscriptionDelivery, TELEMETRY_EVENT
from test_controller import wait_until
from pykinisi.KinisiCommands import (
    ErrorCode, GET_PLATFORM_ODOMETRY, INIT, READY, TIME_SYNC_REQUEST, TIME_SYNC_RESPONSE, PROTOCOL_VERSION,
)
from pykinisi.errors import ControllerError


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT.parent / "kinisi-motor-controller-firmware"
TRANSPORT_MODULE = importlib.import_module("pykinisi.KinisiController")


class FirmwareSerial:
    """A serialized byte stream between pyserial's API and the C session fixture."""

    def __init__(self, library, timeout=0.05, write_timeout=1.0):
        """Start one simulated board, with real clock deltas and adjustable uptime."""
        self.library = library
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.is_open = True
        self._lock = threading.RLock()
        self._base_ns = time.monotonic_ns()
        self._offset_us = 0
        self._cancelled = threading.Event()
        self.writes = []
        library.fixture_reset(0)

    def _now_us(self):
        """Keep elapsed host processing inside the controller's measured RTT."""
        return self._offset_us + (time.monotonic_ns() - self._base_ns) // 1000

    def _poll(self):
        """Give queued replies and synchronization several serialized task turns."""
        for _ in range(8):
            self.library.fixture_poll(self._now_us())

    def write(self, data):
        """Feed complete or partial client frames into the production C parser."""
        with self._lock:
            if not self.is_open:
                raise OSError("Fixture port is closed")
            payload = bytes(data)
            buffer = (ctypes.c_uint8 * len(payload)).from_buffer_copy(payload)
            if not self.library.fixture_feed(buffer, len(payload)):
                raise OSError("Fixture input buffer is full")
            self.writes.append(payload)
            self._poll()
            return len(payload)

    def read(self, size=1):
        """Poll periodic firmware work while exercising fragmented host reads."""
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline and not self._cancelled.is_set():
            with self._lock:
                if not self.is_open:
                    return b""
                self._poll()
                # Fragment larger replies to exercise the SDK's length-framed reader.
                capacity = min(size, 3)
                buffer = (ctypes.c_uint8 * capacity)()
                length = self.library.fixture_read(buffer, capacity)
                if length:
                    return bytes(buffer[:length])
            self._cancelled.wait(0.0005)
        return b""

    def reset_input_buffer(self):
        """Discard any controller bytes before opening a new Python session."""
        with self._lock:
            buffer = (ctypes.c_uint8 * 8192)()
            self.library.fixture_read(buffer, len(buffer))

    def cancel_read(self):
        """Wake a pending read so disconnect can join the reader thread promptly."""
        self._cancelled.set()

    def close(self):
        """Close the simulated serial port and release any waiting reader."""
        with self._lock:
            self.is_open = False
        self._cancelled.set()

    def advance(self, microseconds):
        """Trigger periodic firmware work without a thirty-second wall-clock wait."""
        with self._lock:
            self._offset_us += microseconds
            self._poll()

    def sent_count(self, command):
        """Read a controller message counter under the firmware serialization lock."""
        with self._lock:
            return self.library.fixture_sent_count(command)

    def set_faults(self, overcurrent, valid=True):
        with self._lock:
            self.library.fixture_set_faults(overcurrent, valid)

    def written_count(self, command):
        """Count host frames after their writes and C command-task turns complete."""
        with self._lock:
            return sum(frame[1] == command for frame in self.writes)


class FirmwareIntegrationTests(unittest.TestCase):
    """Verify the Python/C protocol contract rather than duplicating a fake board."""

    @classmethod
    def setUpClass(cls):
        """Build a temporary shared library from the sibling firmware's session code."""
        sources = [FIRMWARE / name for name in (
            "lib/connection/connection.c", "lib/protocol/protocol.c",
            "lib/time_sync/time_sync.c", "lib/initialization/initialization.c",
        )]
        if not all(path.is_file() for path in sources):
            raise unittest.SkipTest("Optional sibling firmware protocol-v2 sources are unavailable")
        compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
        if not compiler or not (shutil.which(compiler) or Path(compiler).is_file()):
            raise unittest.SkipTest("Set CC to a host gcc/clang to enable firmware interoperability tests")
        directory = tempfile.TemporaryDirectory(prefix="pykinisi-firmware-")
        cls.addClassCleanup(directory.cleanup)
        binary = Path(directory.name) / ("session.dll" if os.name == "nt" else "session.so")
        command = [compiler, "-std=c11", "-shared", "-Wall", "-Wextra", "-Werror", "-DMP_V4"]
        if os.name != "nt":
            command.append("-fPIC")
        for include in ("include", "lib/connection", "lib/protocol", "lib/time_sync", "lib/initialization"):
            command.extend(["-I", str(FIRMWARE / include)])
        command.extend([str(ROOT / "tests" / "fixtures" / "firmware_session.c"),
                        *(str(path) for path in sources), "-o", str(binary)])
        build = subprocess.run(command, capture_output=True, text=True, timeout=60)
        if build.returncode:
            raise AssertionError(f"Firmware fixture build failed:\n{build.stdout}\n{build.stderr}")
        cls.library = ctypes.CDLL(str(binary))
        signatures = {
            "fixture_set_faults": ([ctypes.c_uint8, ctypes.c_uint8], None),
            "fixture_reset": ([ctypes.c_uint64], None),
            "fixture_feed": ([ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t], ctypes.c_int),
            "fixture_poll": ([ctypes.c_uint64], None),
            "fixture_read": ([ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t], ctypes.c_size_t),
            "fixture_sent_count": ([ctypes.c_uint], ctypes.c_uint),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(cls.library, name)
            function.argtypes = arguments
            function.restype = result
        if os.name == "nt":
            # Windows cannot remove a loaded DLL from the temporary directory.
            cls.addClassCleanup(cls._unload_library)

    @classmethod
    def _unload_library(cls):
        """Release the Windows DLL after all reader threads have disconnected."""
        from _ctypes import FreeLibrary
        FreeLibrary(cls.library._handle)
        cls.library = None

    def connect_client(self, wall_clock=True):
        """Connect the real Python transport to a fresh instance of the C fixture."""
        serial = FirmwareSerial(self.library)
        patch = mock.patch.object(TRANSPORT_MODULE.serial, "Serial", return_value=serial)
        patch.start()
        self.addCleanup(patch.stop)
        client = KinisiController(request_timeout=1.0, init_timeout=3.0, wall_clock=wall_clock, heartbeat_timeout_ms=None)
        self.addCleanup(client.disconnect)
        self.assertTrue(client.connect("firmware-fixture"), repr(client.last_error))
        return client, serial

    def test_current_limit_commands_with_production_session(self):
        client, _ = self.connect_client(wall_clock=False)
        for index in range(4):
            self.assertEqual(client.get_motor_current_limit(index).threshold_ma, 6500)
            for threshold in (4000, 5400, 6500, 7000):
                for enabled in (False, True):
                    setting = client.set_motor_current_limit(index, enabled, threshold)
                    self.assertEqual(vars(setting), dict(enabled=enabled, threshold_ma=threshold))
                    self.assertEqual(vars(client.get_motor_current_limit(index)), vars(setting))

    def test_initial_sync_ack_error_and_periodic_refresh(self):
        """INIT/READY and periodic synchronization interoperate with production C."""
        client, serial = self.connect_client()
        self.assertTrue(client.ready)
        self.assertEqual((client.board_info.protocol_major, client.board_info.protocol_minor,
                          client.board_info.protocol_patch), PROTOCOL_VERSION)
        self.assertEqual(serial.sent_count(INIT), 1)
        self.assertEqual(serial.sent_count(TIME_SYNC_REQUEST), 3)
        self.assertEqual(serial.sent_count(READY), 1)
        sync_replies = [frame for frame in serial.writes if frame[1] == TIME_SYNC_RESPONSE]
        self.assertEqual(len(sync_replies), 3)
        self.assertTrue(all(len(frame) == 20 for frame in sync_replies))
        self.assertIsNone(client.stop_motor(0))
        self.assertEqual(client.get_encoder_value(0), 0xBEEF)
        with self.assertRaises(ControllerError) as failure:
            client.get_platform_odometry()
        self.assertEqual(failure.exception.error_code, ErrorCode.ODOMETRY_NOT_INITIALIZED)
        self.assertEqual(failure.exception.command, GET_PLATFORM_ODOMETRY)
        status = client.get_time_status()
        self.assertEqual((status.clock_mode, status.clock_quality, status.interval_ms), (1, 1, 30000))

        # No application request is needed to answer the next controller-initiated burst.
        serial.advance(30_100_000)
        deadline = time.monotonic() + 2.0
        while serial.written_count(TIME_SYNC_RESPONSE) < 6 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(serial.sent_count(TIME_SYNC_REQUEST), 6)
        self.assertEqual(serial.written_count(TIME_SYNC_RESPONSE), 6)
        status = client.get_time_status()
        self.assertEqual((status.clock_mode, status.clock_quality), (1, 1))
        self.assertLess(status.last_sync_age_us, 1_000_000)
        self.assertEqual(serial.sent_count(READY), 1)
        self.assertIsNone(client.last_sync_error)

    def test_uptime_init_skips_wall_clock_exchange(self):
        """Clients without wall time still get READY and ACKs from the real session."""
        client, serial = self.connect_client(wall_clock=False)
        self.assertTrue(client.ready)
        self.assertEqual(serial.sent_count(TIME_SYNC_REQUEST), 0)
        self.assertEqual(serial.sent_count(READY), 1)
        status = client.get_time_status()
        self.assertEqual((status.clock_mode, status.clock_quality, status.last_sync_age_us), (0, 1, 0))
        self.assertIsNone(client.stop_motor(0))

    def test_generic_subscription_with_production_firmware(self):
        client, serial = self.connect_client(wall_clock=False)
        client.set_heartbeat_config(True, 1000)
        samples = []
        handle = client.subscribe(Topic.ENCODER_ODOMETRY, samples.append, interval_ms=100)
        serial.advance(100000)
        deadline = time.monotonic() + 1
        while not samples and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(samples, client.last_error)
        self.assertEqual(samples[-1].angle, 1.25)
        self.assertEqual(samples[-1].clock_quality, 1)
        handle.unsubscribe()
        self.assertFalse(handle.active)

    def test_event_trigger_and_silence_refresh_with_production_firmware(self):
        client, serial = self.connect_client(wall_clock=False)
        client.set_heartbeat_config(True, 1000)
        samples = []
        handle = client.subscribe(Topic.MOTOR_FAULTS, samples.append,
            trigger=SubscriptionTrigger.ON_EVENT, delivery=SubscriptionDelivery.ON_CHANGE, max_silence_ms=500)
        wait_until(lambda: len(samples) == 1)
        serial.set_faults(True)
        wait_until(lambda: len(samples) == 2)
        self.assertTrue(samples[-1].overcurrent)
        self.assertLess(samples[-1].timestamp_us - samples[0].timestamp_us, 100000)
        serial.advance(510000)
        wait_until(lambda: len(samples) >= 3)
        self.assertTrue(samples[-1].overcurrent)
        self.assertGreater(samples[-1].timestamp_us, samples[-2].timestamp_us)
        handle.unsubscribe()

    def test_on_change_faults_with_production_firmware(self):
        client, serial = self.connect_client(wall_clock=False)
        client.set_heartbeat_config(True, 1000)
        samples = []
        handle = client.subscribe(Topic.MOTOR_FAULTS, samples.append, interval_ms=100, mode=SubscriptionMode.ON_CHANGE)
        wait_until(lambda: len(samples) == 1)
        self.assertTrue(samples[0].status_valid)
        self.assertFalse(samples[0].overcurrent)
        baseline = serial.sent_count(TELEMETRY_EVENT)
        for _ in range(3): serial.advance(100000)
        self.assertEqual(serial.sent_count(TELEMETRY_EVENT), baseline)
        serial.set_faults(True)
        serial.advance(100000)
        wait_until(lambda: len(samples) == 2)
        self.assertTrue(samples[-1].overcurrent)
        serial.set_faults(False, False)
        serial.advance(100000)
        wait_until(lambda: len(samples) == 3)
        self.assertFalse(samples[-1].status_valid)
        serial.set_faults(False, True)
        serial.advance(100000)
        wait_until(lambda: len(samples) == 4)
        self.assertTrue(samples[-1].status_valid)
        self.assertFalse(samples[-1].overcurrent)
        handle.unsubscribe()
        baseline = serial.sent_count(TELEMETRY_EVENT)
        serial.set_faults(True)
        serial.advance(100000)
        self.assertEqual(serial.sent_count(TELEMETRY_EVENT), baseline)

    def test_fault_clear_response_and_change_event_with_production_session(self):
        client, serial = self.connect_client(wall_clock=False)
        client.set_heartbeat_config(True, 1000)
        serial.set_faults(True)
        samples = []
        handle = client.subscribe(Topic.MOTOR_FAULTS, samples.append, interval_ms=100, mode=SubscriptionMode.ON_CHANGE)
        wait_until(lambda: len(samples) == 1)
        self.assertTrue(samples[-1].overcurrent)
        response = client.clear_motor_faults(0)
        self.assertTrue(response.status_valid)
        self.assertFalse(response.overcurrent)
        serial.advance(100000)
        wait_until(lambda: len(samples) == 2)
        self.assertTrue(samples[-1].status_valid)
        self.assertFalse(samples[-1].overcurrent)
        handle.unsubscribe()

    def test_current_pull_and_four_streams_with_production_firmware(self):
        client, serial = self.connect_client(wall_clock=False)
        client.set_heartbeat_config(True, 1000)
        readings = [[] for _ in range(4)]
        handles = []
        for index in range(4):
            self.assertTrue(client.supports_motor_current(index))
            self.assertEqual(client.get_motor_current(index), index + .125)
            handles.append(client.subscribe(Topic.MOTOR_CURRENT, readings[index].append, index=index, interval_ms=20))
        serial.advance(20000)
        deadline = time.monotonic() + 1
        while not all(readings) and time.monotonic() < deadline:
            time.sleep(.005)
        for index, samples in enumerate(readings):
            self.assertTrue(samples, client.last_error)
            self.assertEqual(samples[-1].amps, index + .125)
            self.assertEqual(samples[-1].clock_quality, 1)
        for handle in handles:
            handle.unsubscribe()
            self.assertFalse(handle.active)


if __name__ == "__main__":
    unittest.main()
