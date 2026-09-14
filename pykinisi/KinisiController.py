# Filename: KinisiController.py
# Description: Serial protocol-v2 sessions, correlated replies, and background time sync.
"""Keep a synchronous command API while servicing controller messages continuously."""

import math
import struct
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum

import serial

from .KinisiCommands import (
    KinisiCommands, InitResponse, ErrorCode, PROTOCOL_VERSION, INIT, READY, ERROR,
    TIME_SYNC_REQUEST, TIME_SYNC_RESPONSE, START_ENCODER_ODOMETRY,
    GET_ENCODER_ODOMETRY, START_PLATFORM_ODOMETRY, GET_PLATFORM_ODOMETRY,
)
from ._version import VERSION
from .errors import ConnectionClosedError, ControllerError, KinisiError, ProtocolError, RequestTimeoutError


class MotorIndex:
    """Motor connector indices."""
    Motor0, Motor1, Motor2, Motor3 = range(4)


class EncoderIndex:
    """Encoder connector indices."""
    Encoder0, Encoder1, Encoder2, Encoder3 = range(4)


class GPIOIndex:
    """GPIO connector indices."""
    GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7, GPIO8, GPIO9 = range(10)


class GPIOMode:
    """GPIO modes accepted by the command schema."""
    INPUT, INPUT_PULLUP, INPUT_NOPULL, OUTPUT = range(4)


class State:
    """Digital GPIO levels."""
    LOW, HIGH = range(2)


class ClockMode(IntEnum):
    """Timestamp domain returned by READY and odometry."""
    UPTIME = 0
    WALL = 1


class ClockQuality(IntEnum):
    """Quality of the controller's current timestamp mapping."""
    UNREADY = 0
    VALID = 1
    STALE = 2


@dataclass
class _PendingRequest:
    """One waiting caller; INIT waits for identity and READY on the same ID."""
    command: int
    response_length: int
    event: threading.Event = field(default_factory=threading.Event)
    payload: bytes = None
    error: Exception = None
    clock_mode: int = None


class KinisiController(KinisiCommands):
    """A protocol-v2 serial client; disconnect or use a context manager when finished."""

    def __init__(self, *, request_timeout=1.0, init_timeout=5.0, wall_clock=True):
        """Configure finite command/setup deadlines and whether Unix time is available."""
        super().__init__()
        for name, value in (("request_timeout", request_timeout), ("init_timeout", init_timeout)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.request_timeout = float(request_timeout)
        self.init_timeout = float(init_timeout)
        self.wall_clock = bool(wall_clock)
        self.serial = None
        self.last_error = None
        self.last_sync_error = None
        self.board_info = None
        self.clock_mode = None
        self._ready = False
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._pending = {}
        self._retired_ids = set()
        self._message_id = 0
        self._reader = None
        self._stop = threading.Event()
        self._wall_clock_active = self.wall_clock

    @property
    def ready(self):
        """True only after a compatible identity and matching READY have arrived."""
        with self._lock:
            return self.serial is not None and self._ready

    def connect(self, port):
        """Open and finish INIT/READY; return False with last_error on failure."""
        with self._lifecycle_lock:
            self.disconnect()
            transport = None
            try:
                transport = serial.Serial(port, 115200, timeout=0.05,
                                          write_timeout=self.request_timeout)
                transport.reset_input_buffer()
                with self._lock:
                    self.serial = transport
                    self.last_error = self.last_sync_error = None
                    self.board_info = self.clock_mode = None
                    self._ready = False
                    self._pending.clear()
                    self._retired_ids.clear()
                    self._message_id = 0
                    self._wall_clock_active = self.wall_clock
                    self._stop = threading.Event()
                    self._reader = threading.Thread(target=self._reader_loop,
                        args=(transport, self._stop), name="pykinisi-reader", daemon=True)
                    self._reader.start()
                # Python SDK, package version, required protocol, wall-clock capability only.
                self.init(1, *VERSION, *PROTOCOL_VERSION, int(self.wall_clock))
                return True
            except (KinisiError, serial.SerialException, ValueError, OSError) as error:
                self.last_error = error
                if transport is not None:
                    self._fail_session(error, transport)
                    try:
                        transport.close()
                    except (serial.SerialException, OSError):
                        pass
                self._join_reader()
                return False

    def _next_id(self):
        """Allocate a nonzero ID, skipping active and timed-out requests until reconnect."""
        for _ in range(65535):
            self._message_id = self._message_id % 65535 + 1
            if self._message_id not in self._pending and self._message_id not in self._retired_ids:
                return self._message_id
        raise ProtocolError("No safe message IDs remain; reconnect before sending more commands")

    def _request(self, command, payload, response_length):
        """Send a generated command and await its correlated ACK/data/error with a deadline."""
        timeout = self.init_timeout if command == INIT else self.request_timeout
        deadline = time.monotonic() + timeout
        with self._lock:
            transport = self.serial
            if transport is None:
                raise ConnectionClosedError("Connect before sending commands")
            if command in (START_ENCODER_ODOMETRY, GET_ENCODER_ODOMETRY,
                           START_PLATFORM_ODOMETRY, GET_PLATFORM_ODOMETRY) and not self._ready:
                raise ProtocolError("Odometry requires a completed INIT/READY exchange")
            if command == INIT and any(p.command == INIT for p in self._pending.values()):
                raise ProtocolError("INIT is already in progress")
            message_id = self._next_id()
            pending = _PendingRequest(command, response_length)
            if command == INIT:
                if len(payload) != 8:
                    raise ProtocolError("INIT requires an eight-byte payload")
                pending.clock_mode = int(bool(payload[7] & 1))
                self._wall_clock_active = bool(payload[7] & 1)
                self._ready = False
                self.board_info = self.clock_mode = None
            self._pending[message_id] = pending
        try:
            self._send_frame(transport, command, message_id, payload, deadline)
            pending.event.wait(max(0, deadline - time.monotonic()))
            with self._lock:
                if not pending.event.is_set():
                    if self.serial is transport:
                        self._retired_ids.add(message_id)
                    raise RequestTimeoutError(f"Timed out waiting for command 0x{command:02X}, "
                                              f"message {message_id}; it may have executed")
                if pending.error is not None:
                    raise pending.error
                return pending.payload
        finally:
            with self._lock:
                if self._pending.get(message_id) is pending:
                    del self._pending[message_id]

    def _send_frame(self, transport, command, message_id, payload, deadline):
        """Serialize frames; callable payloads capture H3 after taking the write lock."""
        if not self._write_lock.acquire(timeout=max(0, deadline - time.monotonic())):
            raise RequestTimeoutError("Timed out waiting to write a command")
        try:
            with self._lock:
                if self.serial is not transport or self._stop.is_set():
                    raise ConnectionClosedError("Connection closed before transmission")
            data = payload() if callable(payload) else payload
            if len(data) > 251:
                raise ProtocolError("Message payload exceeds the protocol frame limit")
            frame = struct.pack("<BBH", len(data) + 3, command, message_id) + data
            offset = 0
            while offset < len(frame):
                with self._lock:
                    if self.serial is not transport or self._stop.is_set():
                        raise ConnectionClosedError("Connection closed during transmission")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RequestTimeoutError("Serial write exceeded its deadline")
                transport.write_timeout = min(self.request_timeout, remaining)
                written = transport.write(frame[offset:])
                # Windows cancel_write can return a partial count instead of raising.
                with self._lock:
                    if self.serial is not transport or self._stop.is_set():
                        raise ConnectionClosedError("Connection closed during transmission")
                if not isinstance(written, int) or written <= 0 or written > len(frame) - offset:
                    raise ConnectionClosedError("Serial write did not make valid progress")
                offset += written
        except Exception as error:
            # A partial outgoing frame cannot be safely retried on the same byte stream.
            failure = error if isinstance(error, KinisiError) else ConnectionClosedError(str(error))
        else:
            failure = None
        finally:
            self._write_lock.release()
        if failure is not None:
            self._fail_session(failure, transport)
            raise failure

    def _reader_loop(self, transport, stop):
        """Read frames across partial reads and service sync even while callers are idle."""
        buffer = bytearray()
        started = None
        try:
            while not stop.is_set():
                if started is not None and time.monotonic() - started >= self.request_timeout:
                    raise ProtocolError("Timed out in an incomplete response frame; reconnect")
                wanted = 1 if not buffer else buffer[0] + 1 - len(buffer)
                chunk = transport.read(wanted)
                received_us = time.time_ns() // 1000
                if stop.is_set():
                    return
                if not chunk:
                    continue
                if not buffer:
                    started = time.monotonic()
                buffer.extend(chunk)
                if not 3 <= buffer[0] <= 254:
                    raise ProtocolError("Invalid response length; protocol v2 firmware is required")
                if len(buffer) > buffer[0] + 1:
                    raise ProtocolError("Serial transport returned more bytes than requested")
                if len(buffer) == buffer[0] + 1:
                    self._dispatch_frame(transport, bytes(buffer[1:]), received_us)
                    buffer.clear()
                    started = None
        except Exception as error:
            if not stop.is_set():
                failure = error if isinstance(error, KinisiError) else ConnectionClosedError(str(error))
                self._fail_session(failure, transport)

    def _dispatch_frame(self, transport, frame, received_us):
        """Route controller requests separately from replies sharing the same numeric ID."""
        command, message_id = struct.unpack("<BH", frame[:3])
        payload = frame[3:]
        with self._lock:
            if self.serial is not transport:
                return
        if command == TIME_SYNC_REQUEST:
            if payload or message_id == 0:
                raise ProtocolError("Malformed TIME_SYNC_REQUEST")
            if self._wall_clock_active:
                self._send_frame(transport, TIME_SYNC_RESPONSE, message_id,
                    lambda: struct.pack("<QQ", received_us, time.time_ns() // 1000),
                    time.monotonic() + self.request_timeout)
            return
        with self._lock:
            pending = self._pending.get(message_id)
            if command == ERROR:
                if len(payload) != 2:
                    raise ProtocolError("Malformed ERROR response")
                failed_command, code = payload
                error = ControllerError(failed_command, message_id, code)
                if failed_command == TIME_SYNC_RESPONSE:
                    self.last_sync_error = error
                    return
                if pending is None or pending.event.is_set():
                    return # A late error must not affect another request.
                if pending.command != failed_command:
                    raise ProtocolError("ERROR refers to the wrong command for its message ID")
                # A board reset or lost clock setup invalidates the old READY state.
                if code in (ErrorCode.INIT_REQUIRED, ErrorCode.CLOCK_NOT_READY):
                    self._ready = False
                pending.error = error
                pending.event.set()
                return
            if pending is None or pending.event.is_set():
                return # Discard late/duplicate replies and READY for retired INITs.
            if command == READY:
                if pending.command != INIT:
                    return # Controller and client ID sequences are independent.
                if len(payload) != 1 or payload[0] != pending.clock_mode or pending.payload is None:
                    raise ProtocolError("READY must follow INIT identity with the requested clock mode")
                self.clock_mode = ClockMode(payload[0])
                self._ready = True
                pending.event.set()
                return
            if command != pending.command or len(payload) != pending.response_length:
                raise ProtocolError("Response command or payload length does not match its request")
            if command == INIT:
                identity = InitResponse.decode(payload)
                if (identity.protocol_major != PROTOCOL_VERSION[0] or
                        identity.protocol_minor < PROTOCOL_VERSION[1]):
                    raise ProtocolError("The controller reported an incompatible protocol version")
                self.board_info = identity
                pending.payload = payload
                return # READY is the final response to INIT.
            pending.payload = payload
            pending.event.set()

    def _fail_session(self, error, transport):
        """Fail waiting callers and close only the session that experienced the failure."""
        with self._lock:
            if self.serial is not transport:
                return
            self.serial = None
            self._ready = False
            self._stop.set()
            self.last_error = error
            for pending in self._pending.values():
                if not pending.event.is_set():
                    pending.error = error
                    pending.event.set()
        # Cancellation releases concurrent I/O before close takes the write lock.
        for method in ("cancel_read", "cancel_write"):
            try:
                cancel = getattr(transport, method, None)
                if cancel is not None:
                    cancel()
            except (serial.SerialException, OSError):
                pass
        with self._write_lock:
            try:
                transport.close()
            except (serial.SerialException, OSError):
                pass

    def _join_reader(self):
        """Wait for the bounded serial reader without attempting to join its own thread."""
        reader = self._reader
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=self.request_timeout + 0.2)

    def disconnect(self):
        """Cancel pending requests and stop the reader; this does not send motor commands."""
        with self._lifecycle_lock:
            with self._lock:
                transport = self.serial
            if transport is not None:
                self._fail_session(ConnectionClosedError("Connection was disconnected"), transport)
            self._join_reader()

    def __enter__(self):
        """Return this controller for a scope that always disconnects on exit."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Close the connection without suppressing an application exception."""
        self.disconnect()
