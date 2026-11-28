# Filename: KinisiController.py
# Description: Serial protocol-v2 sessions, correlated replies, and background time sync.
"""Keep a synchronous command API while servicing controller messages continuously."""

import math
import struct
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from enum import IntEnum

import serial

from .KinisiCommands import (
    KinisiCommands, InitResponse, ErrorCode, INIT, READY, ERROR,
    INITIALIZE_MECANUM_PLATFORM_ORIENTED, MecanumRollerOrientation,
    TIME_SYNC_REQUEST, TIME_SYNC_RESPONSE, START_ENCODER_ODOMETRY,
    GET_ENCODER_ODOMETRY, START_PLATFORM_ODOMETRY, GET_PLATFORM_ODOMETRY,
    PING, SET_HEARTBEAT_CONFIG, SUBSCRIBE_ODOMETRY, UNSUBSCRIBE_ODOMETRY,
    ENCODER_ODOMETRY_EVENT, PLATFORM_ODOMETRY_EVENT,
    EncoderOdometrySample, PlatformOdometrySample, GET_TEMPERATURE, GET_IMU, GET_MOTOR_CURRENT, GET_MOTOR_FAULTS, CLEAR_MOTOR_FAULTS, GET_PWM_FREQUENCY, SET_PWM_FREQUENCY, GET_MOTOR_CURRENT_LIMIT, SET_MOTOR_CURRENT_LIMIT,
    FeedbackSourceVectorKind, FeedbackSourceStatus, FeedbackSourceVector,
    GET_FEEDBACK_SOURCE_HEALTH, GET_FEEDBACK_IMU_CHANNELS,
    SET_IMU_MOUNT, GET_IMU_MOUNT, SET_BOARD_MOUNT, GET_BOARD_MOUNT,
    SET_FEEDBACK_SOURCE, GET_FEEDBACK_SOURCE_STATUS, GET_FEEDBACK_SOURCE_VECTOR, SET_FEEDBACK_FUSION_MOUNT, GET_FEEDBACK_FUSION_MOUNT,
    SET_IMU_TRACKING, GET_IMU_TRACKING_STATUS, GET_IMU_TRACKING_VECTOR, GET_IMU_TRACKING_CONFIGURATION,
    SET_FUSION_EXPERIMENT, GET_FUSION_STATUS, GET_FUSION_VECTOR, GET_FUSION_CONFIGURATION,
    SAVE_CONFIGURATION, GET_CONFIGURATION_FIELD, SET_DRIVER_SLEEP, GET_DRIVER_SLEEP, SET_BOARD_IDLE, GET_BOARD_IDLE,
    SUBSCRIBE, SUBSCRIBE_WITH_MODE, SUBSCRIBE_CONFIGURED, UNSUBSCRIBE, TELEMETRY_EVENT, Topic, SubscriptionMode, SubscriptionTrigger, SubscriptionDelivery, validate_topic, decode_telemetry_event,
)
from .board_profile import get_board_profile, supports_board_sensor, supports_motor_current, supports_motor_faults, get_motor_current_limit_options, supports_motor_current_limit, get_pwm_frequency_options, supports_pwm_frequency
from ._version import VERSION
from .KinisiCommands import (
    INITIALIZE_MOTOR_POSITION_CONTROLLER, RESET_MOTOR_POSITION, SET_MOTOR_POSITION,
    GET_MOTOR_POSITION, INITIALIZE_PLATFORM_POSITION_CONTROLLER,
    RESET_PLATFORM_POSITION, SET_PLATFORM_POSITION,
    INITIALIZE_MOTOR_POSITION_PID_CONTROLLER, INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER,
)

from .errors import ConnectionClosedError, ControllerError, KinisiError, ProtocolError, RequestTimeoutError

_UNSET = object()

MIN_PROTOCOL_VERSION = (2, 1, 0)
POSITION_COMMANDS = frozenset((INITIALIZE_MOTOR_POSITION_CONTROLLER, RESET_MOTOR_POSITION,
    SET_MOTOR_POSITION, GET_MOTOR_POSITION, INITIALIZE_PLATFORM_POSITION_CONTROLLER,
    RESET_PLATFORM_POSITION, SET_PLATFORM_POSITION))
POSITION_PID_COMMANDS = frozenset((INITIALIZE_MOTOR_POSITION_PID_CONTROLLER,
    INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER))
SENSOR_COMMANDS = {GET_TEMPERATURE: ("temperature", 4), GET_IMU: ("imu", 5)}


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
    request_payload: bytes = b""


@dataclass
class _SubscriptionEntry:
    topic: int
    index: int
    callback: object
    session: int
    latest: object = None
    closing: bool = False


class Subscription:
    """One subscription's ownership; an older handle cannot cancel its replacement."""

    def __init__(self, owner, entry):
        self._owner, self._entry = owner, entry

    @property
    def active(self):
        owner, entry = self._owner, self._entry
        with owner._lock:
            return (not entry.closing and entry.session == owner._session and
                    owner._telemetry.get((entry.topic, entry.index)) is entry)

    def unsubscribe(self):
        owner, entry = self._owner, self._entry
        with owner._telemetry_operation_lock:
            if not self.active:
                return
            with owner._subscription_session(entry.session):
                owner.unsubscribe(entry.topic, entry.index)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.unsubscribe()


@dataclass(frozen=True)
class FeedbackSourceSnapshot:
    """One verified board observation in native relative-world and body frames."""
    status: FeedbackSourceStatus
    pose: 'FeedbackSourceVector | None'
    twist: 'FeedbackSourceVector | None'


@dataclass(frozen=True)
class BoardMountGeometry:
    """Board-to-chassis angles in degrees and PCB underside-center position in chassis metres."""
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    x_m: float
    y_m: float
    z_m: float
    configured: bool


@dataclass(frozen=True)
class ImuMountGeometry:
    """Board-to-chassis angles in degrees and origin-to-chip distances in chassis metres."""
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    x_m: float
    y_m: float
    z_m: float
    configured: bool


def _mount_units(value, limit, scale, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not -limit <= value <= limit:
        raise ValueError(f'{name} must be a finite number in [{-limit}, {limit}]')
    fraction, whole = math.modf(abs(value) * scale)
    magnitude = int(whole) + (fraction >= 0.5)
    return -magnitude if value < 0 else magnitude


def _mount_arguments(roll_deg, pitch_deg, yaw_deg, x_m, y_m, z_m):
    return (
        _mount_units(roll_deg, 180, 100, 'roll_deg'),
        _mount_units(pitch_deg, 90, 100, 'pitch_deg'),
        _mount_units(yaw_deg, 180, 100, 'yaw_deg'),
        _mount_units(x_m, 10, 1_000_000, 'x_m'),
        _mount_units(y_m, 10, 1_000_000, 'y_m'),
        _mount_units(z_m, 10, 1_000_000, 'z_m'),
    )


class KinisiController(KinisiCommands):
    """A protocol-v2 serial client; disconnect or use a context manager when finished."""

    def __init__(self, *, request_timeout=1.0, init_timeout=5.0, wall_clock=True, heartbeat_timeout_ms=500):
        """Configure finite command/setup deadlines and whether Unix time is available."""
        super().__init__()
        for name, value in (("request_timeout", request_timeout), ("init_timeout", init_timeout)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.request_timeout = float(request_timeout)
        self.init_timeout = float(init_timeout)
        self.wall_clock = bool(wall_clock)
        if heartbeat_timeout_ms is not None and (type(heartbeat_timeout_ms) is not int or not 100 <= heartbeat_timeout_ms <= 60000):
            raise ValueError("heartbeat_timeout_ms must be 100..60000, or None to disable")
        self.heartbeat_timeout_ms = heartbeat_timeout_ms
        self._heartbeat_idle_s = None
        self._heartbeat_thread = None
        self._last_sent = 0
        self._subscription_samples = {}
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
        self._session = 0
        self._telemetry = {}
        self._telemetry_candidates = {}
        self._telemetry_operation_lock = threading.RLock()
        self._telemetry_context = threading.local()
        self._telemetry_wakeup = threading.Event()
        self._telemetry_thread = None
        self.last_telemetry_error = None

    @property
    def ready(self):
        """True only after a compatible identity and matching READY have arrived."""
        with self._lock:
            return self.serial is not None and self._ready

    def get_pwm_frequency_options(self):
        with self._lock:
            return get_pwm_frequency_options(self.board_info)

    def supports_pwm_frequency(self):
        with self._lock:
            return supports_pwm_frequency(self.board_info)

    def supports_configuration_save(self):
        """Return whether the connected board can save its setup to FRAM."""
        with self._lock:
            info = self.board_info
            return (self._ready and self.board_profile is not None and
                    (info.board_model, info.board_major, info.board_minor, info.board_patch) == (1, 0, 4, 0) and
                    info.protocol_major == 2 and info.protocol_minor >= 14)

    def get_mecanum_roller_orientation(self):
        """Read the applied layout, or the fixed STANDARD layout on pre-2.25 firmware."""
        value = self.get_configuration_field(7, 0)
        try:
            return MecanumRollerOrientation(value)
        except ValueError as exc:
            raise ProtocolError('Invalid mecanum roller orientation in configuration response') from exc

    def supports_driver_sleep(self):
        """Require READY and the minimum protocol for the exact supported board."""
        with self._lock:
            info = self.board_info
            profile = self.board_profile
            if not self.ready or info is None or profile is None or info.protocol_major != 2:
                return False
            identity = (info.board_model, info.board_major, info.board_minor, info.board_patch)
            minimum = {(1, 0, 3, 0): 17, (1, 0, 3, 1): 17, (1, 0, 4, 0): 15}.get(identity)
            return (minimum is not None and profile.get('power', {}).get('driver_sleep') is True and
                    info.protocol_minor >= minimum)

    def supports_fusion_experiment(self):
        """Opt-in experiment on exact v0.4.0 and compatible protocol 2.19+."""
        with self._lock:
            info = self.board_info
            profile = self.board_profile
            return bool(self.ready and info is not None and profile is not None and
                        (info.board_model, info.board_major, info.board_minor, info.board_patch) == (1, 0, 4, 0) and
                        info.protocol_major == 2 and info.protocol_minor >= 19 and
                        profile.get('capabilities', {}).get('fusion_experiment') is True)

    def _supports_feedback_capability(self, capability, minor):
        with self._lock:
            info = self.board_info
            profile = self.board_profile
            return bool(self.ready and info and profile and
                        (info.board_model, info.board_major, info.board_minor, info.board_patch) == (1, 0, 4, 0) and
                        info.protocol_major == 2 and minor <= info.protocol_minor <= 255 and
                        profile.get('capabilities', {}).get(capability) is True)

    def supports_feedback_source_selector(self):
        """Require the exact board capability and READY protocol 2.21+."""
        return self._supports_feedback_capability('feedback_source_selector', 21)

    def supports_feedback_source_health(self):
        """Whether the board supports read-only source health and BMI088 channel times."""
        return self._supports_feedback_capability('feedback_source_health', 22)

    def supports_board_mounting(self):
        """Whether this READY session supports the PCB underside-center mounting configuration."""
        return self._supports_feedback_capability('imu_mounting', 24)

    def set_board_mount_geometry(self, roll_deg, pitch_deg, yaw_deg, x_m, y_m, z_m):
        """Set Rz(yaw) Ry(pitch) Rx(roll) and the chassis-origin to PCB underside-center offset."""
        self.set_board_mount(*_mount_arguments(roll_deg, pitch_deg, yaw_deg, x_m, y_m, z_m))

    def get_board_mount_geometry(self) -> BoardMountGeometry:
        """Read the quantized PCB underside-center pose and the explicit configuration flag."""
        mount = self.get_board_mount()
        return BoardMountGeometry(mount.roll_cdeg / 100, mount.pitch_cdeg / 100, mount.yaw_cdeg / 100,
                                  mount.x_um / 1_000_000, mount.y_um / 1_000_000, mount.z_um / 1_000_000,
                                  mount.configured)

    def supports_imu_mounting(self):
        """Whether this READY session supports the legacy chip-referenced configuration."""
        return self._supports_feedback_capability('imu_mounting', 23)

    def set_imu_mount_geometry(self, roll_deg, pitch_deg, yaw_deg, x_m, y_m, z_m):
        """Deprecated chip-reference setter. Offsets locate the BMI088 in chassis metres."""
        self.set_imu_mount(*_mount_arguments(roll_deg, pitch_deg, yaw_deg, x_m, y_m, z_m))

    def get_imu_mount_geometry(self) -> ImuMountGeometry:
        """Deprecated chip-reference getter. Read the applied BMI088 location in chassis metres."""
        mount = self.get_imu_mount()
        return ImuMountGeometry(mount.roll_cdeg / 100, mount.pitch_cdeg / 100, mount.yaw_cdeg / 100,
                                mount.x_um / 1_000_000, mount.y_um / 1_000_000, mount.z_um / 1_000_000,
                                mount.configured)

    def read_feedback_source_snapshot(self):
        """Bracket pose and twist with board status. Invalid status has no vectors."""
        session = self._session
        fields = ('requested', 'active', 'origin_id')
        for _ in range(3):
            first = self.get_feedback_source_status()
            if session != self._session or not self.ready:
                raise ConnectionClosedError('Controller session changed')
            if not first.valid:
                return FeedbackSourceSnapshot(first, None, None)
            pose = twist = None
            try:
                pose = self.get_feedback_source_vector(FeedbackSourceVectorKind.POSE, first.origin_id, first.sequence)
                twist = self.get_feedback_source_vector(FeedbackSourceVectorKind.TWIST, first.origin_id, first.sequence)
            except ControllerError as error:
                if error.error_code != ErrorCode.SAMPLE_NOT_AVAILABLE:
                    raise
            status = self.get_feedback_source_status()
            if session != self._session or not self.ready:
                raise ConnectionClosedError('Controller session changed')
            if not status.valid:
                return FeedbackSourceSnapshot(status, None, None)
            if (pose is None or twist is None or pose.sequence != first.sequence or twist.sequence != first.sequence or
                    any(getattr(first, field) != getattr(status, field) for field in fields)):
                continue
            advance = (status.sequence - first.sequence) & 0xffffffff
            tick = status.timestamp_us - first.timestamp_us
            if advance >= 0x80000000 or tick < 0 or (advance == 0) != (tick == 0):
                continue
            age_at_check = tick + status.age_us
            elapsed = age_at_check - first.age_us
            if 0 <= elapsed <= 50000 and age_at_check <= 1000 * min(first.max_age_ms, status.max_age_ms):
                first.age_us = age_at_check
                return FeedbackSourceSnapshot(first, pose, twist)
        raise ProtocolError('Feedback source changed during read. No coherent snapshot available.')

    def supports_imu_tracking(self):
        """Opt-in experiment on exact v0.4.0 and compatible protocol 2.20+."""
        with self._lock:
            info = self.board_info
            profile = self.board_profile
            return bool(self.ready and info is not None and profile is not None and
                        (info.board_model, info.board_major, info.board_minor, info.board_patch) == (1, 0, 4, 0) and
                        info.protocol_major == 2 and info.protocol_minor >= 20 and
                        profile.get('capabilities', {}).get('imu_tracking') is True)

    def supports_board_idle(self):
        """Require READY and the board-specific minimum protocol for shallow idle."""
        with self._lock:
            info = self.board_info
            profile = self.board_profile
            if not self.ready or info is None or profile is None or info.protocol_major != 2:
                return False
            identity = (info.board_model, info.board_major, info.board_minor, info.board_patch)
            minimum = {(1, 0, 3, 0): 18, (1, 0, 3, 1): 18, (1, 0, 4, 0): 16}.get(identity)
            return (minimum is not None and profile.get('power', {}).get('board_idle') is True and
                    info.protocol_minor >= minimum)

    def get_motor_current_limit_options(self, index=0):
        """Read the selected motor's supported thresholds and disable capability."""
        with self._lock:
            return get_motor_current_limit_options(self.board_info, index)

    def supports_motor_current_limit(self, index=0):
        """Check hardware and protocol support for current-limit configuration."""
        with self._lock:
            return supports_motor_current_limit(self.board_info, index)

    @property
    def board_profile(self):
        """Bundled hardware profile selected from INIT, or None for an unknown board."""
        with self._lock:
            return get_board_profile(self.board_info)

    def supports_sensor(self, kind, index=0):
        """Check hardware presence and firmware support without performing I/O."""
        with self._lock:
            return supports_board_sensor(self.board_info, kind, index)

    def supports_motor_current(self, index=0):
        """Check hardware presence and firmware support without performing I/O."""
        with self._lock:
            return supports_motor_current(self.board_info, index)

    def supports_motor_faults(self, index=0):
        """Check hardware presence and firmware support without performing I/O."""
        with self._lock:
            return supports_motor_faults(self.board_info, index)

    def supports_topic(self, topic, index=0):
        """Check protocol and known hardware; running odometry is checked by firmware."""
        try:
            validate_topic(topic, index)
        except ValueError:
            return False
        with self._lock:
            info, profile = self.board_info, self.board_profile
            if not info or info.protocol_major != 2 or info.protocol_minor < 6 or profile is None:
                return False
            if topic == Topic.ENCODER_ODOMETRY:
                return index < profile['resources']['encoders']
            if topic == Topic.PLATFORM_ODOMETRY:
                return True
            if topic == Topic.MOTOR_FAULTS:
                return self.supports_motor_faults(index)
            if topic == Topic.MOTOR_CURRENT:
                return self.supports_motor_current(index)
            if topic in (Topic.TEMPERATURE, Topic.IMU):
                return self.supports_sensor('temperature' if topic == Topic.TEMPERATURE else 'imu', index)
            return False

    def subscribe(self, topic, callback, *, index=0, interval_ms=None, mode=_UNSET,
                  trigger=None, delivery=None, poll_interval_ms=None, max_silence_ms=None):
        """Subscribe to typed samples and return an ownership handle.

        Callbacks run on a worker, separately from serial reads and heartbeat.
        Slow callbacks receive the latest sample; intermediate samples are coalesced.
        """
        configured = any(v is not None for v in (trigger, delivery, poll_interval_ms, max_silence_ms))
        if configured and (mode is not _UNSET or interval_ms is not None):
            raise ValueError('Use poll_interval_ms with trigger/delivery settings')
        mode = SubscriptionMode.PERIODIC if mode is _UNSET else mode
        trigger = SubscriptionTrigger.PERIODIC if trigger is None else trigger
        delivery = SubscriptionDelivery.ALL if delivery is None else delivery
        max_silence_ms = 0 if max_silence_ms is None else max_silence_ms
        if trigger == SubscriptionTrigger.ON_EVENT and poll_interval_ms is None:
            poll_interval_ms = 0
        validate_topic(topic, index)
        if isinstance(mode, bool) or not isinstance(mode, int) or mode not in tuple(SubscriptionMode):
            raise ValueError('Invalid subscription mode')
        if not callable(callback):
            raise TypeError('callback must be callable')
        key = (topic, index)
        with self._telemetry_operation_lock:
            with self._lock:
                entry = _SubscriptionEntry(topic, index, callback, self._session)
                self._telemetry_candidates[key] = entry
            try:
                with self._subscription_session(entry.session):
                    if configured:
                        super().subscribe_configured(topic, index, poll_interval_ms, trigger, delivery, max_silence_ms)
                    elif mode == SubscriptionMode.PERIODIC:
                        super().subscribe(topic, index, interval_ms)
                    else:
                        super().subscribe_with_mode(topic, index, interval_ms, mode)
            except RequestTimeoutError:
                # A lost ACK may hide a successful subscription. Cancel it; if that
                # cannot be confirmed, terminate the session and its watchdog.
                with self._lock:
                    same_session = entry.session == self._session
                    transport = self.serial
                if same_session and transport is not None:
                    try:
                        with self._subscription_session(entry.session):
                            super().unsubscribe(topic, index)
                    except Exception as error:
                        self._fail_session(error, transport)
                raise
            finally:
                with self._lock:
                    if self._telemetry_candidates.get(key) is entry:
                        del self._telemetry_candidates[key]
            return Subscription(self, entry)

    @contextmanager
    def _subscription_session(self, session):
        """Keep an operation bound to its connection even if another thread reconnects."""
        previous = getattr(self._telemetry_context, 'session', None)
        self._telemetry_context.session = session
        try:
            yield
        finally:
            self._telemetry_context.session = previous

    def unsubscribe(self, topic, index=0):
        """Remove the selected topic/resource and its callback after acknowledgement."""
        with self._telemetry_operation_lock:
            with self._lock:
                transport = self.serial
            try:
                return super().unsubscribe(topic, index)
            except RequestTimeoutError as error:
                if transport is not None:
                    self._fail_session(error, transport)
                raise

    def _clear_telemetry(self):
        """Invalidate handles under the session lock."""
        self._session += 1
        for entry in (*self._telemetry.values(), *self._telemetry_candidates.values()):
            entry.closing = True
        self._telemetry.clear()
        self._telemetry_candidates.clear()

    def _telemetry_loop(self, transport, stop, wakeup):
        while not stop.is_set():
            wakeup.wait(0.05)
            wakeup.clear()
            with self._lock:
                if self.serial is not transport:
                    return
                entries = list(self._telemetry.items())
            for key, entry in entries:
                with self._lock:
                    if stop.is_set() or self._telemetry.get(key) is not entry or entry.closing:
                        continue
                    sample, entry.latest = entry.latest, None
                if sample is not None:
                    try:
                        entry.callback(sample)
                    except Exception as error:
                        with self._lock:
                            if self.serial is transport and entry.session == self._session:
                                self.last_telemetry_error = error

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
                    self.last_telemetry_error = None
                    self.board_info = self.clock_mode = None
                    self._ready = False
                    self._pending.clear()
                    self._retired_ids.clear()
                    self._message_id = 0
                    self._wall_clock_active = self.wall_clock
                    self._stop = threading.Event()
                    self._telemetry_wakeup = threading.Event()
                    self._telemetry_thread = threading.Thread(target=self._telemetry_loop,
                        args=(transport, self._stop, self._telemetry_wakeup), name="pykinisi-telemetry", daemon=True)
                    self._telemetry_thread.start()
                    self._reader = threading.Thread(target=self._reader_loop,
                        args=(transport, self._stop), name="pykinisi-reader", daemon=True)
                    self._reader.start()
                # Advertise streaming support independently of wall-clock availability.
                self.init(1, *VERSION, *MIN_PROTOCOL_VERSION, int(self.wall_clock) | 2)
                if self.heartbeat_timeout_ms is not None:
                    self.set_heartbeat_config(True, self.heartbeat_timeout_ms)
                self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop,
                    args=(transport, self._stop), name="pykinisi-heartbeat", daemon=True)
                self._heartbeat_thread.start()
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
            if command in (SUBSCRIBE, SUBSCRIBE_WITH_MODE, SUBSCRIBE_CONFIGURED, UNSUBSCRIBE):
                expected_session = getattr(self._telemetry_context, 'session', None)
                if expected_session is not None and expected_session != self._session:
                    raise ConnectionClosedError('Subscription connection changed before transmission')
                if (not self._ready or self.board_info is None or
                        self.board_info.protocol_major != 2 or self.board_info.protocol_minor < 6):
                    raise ProtocolError('Subscriptions require a ready protocol 2.6+ controller')
                if command == SUBSCRIBE_CONFIGURED and self.board_info.protocol_minor < 12:
                    raise ProtocolError('Subscription settings require firmware protocol 2.12 or newer')
                if command == SUBSCRIBE_WITH_MODE and self.board_info.protocol_minor < 9:
                    raise ProtocolError('Subscription modes require firmware protocol 2.9 or newer')
                if payload[0] == Topic.MOTOR_FAULTS and self.board_info.protocol_minor < 8:
                    raise ProtocolError('Motor faults require firmware protocol 2.8 or newer')
                if payload[0] == Topic.MOTOR_CURRENT and self.board_info.protocol_minor < 7:
                    raise ProtocolError('Motor current requires a ready protocol 2.7+ controller')
                if command in (SUBSCRIBE, SUBSCRIBE_WITH_MODE, SUBSCRIBE_CONFIGURED) and self.board_profile is not None and not self.supports_topic(payload[0], payload[1]):
                    raise ProtocolError('This board does not support the selected telemetry source')
            orientation_field = command == GET_CONFIGURATION_FIELD and payload == b'\x07\x00'
            if orientation_field or command == INITIALIZE_MECANUM_PLATFORM_ORIENTED:
                if not self._ready or self.board_info is None or self.board_info.protocol_major != 2:
                    raise ProtocolError('Mecanum roller configuration requires a ready protocol 2.x controller')
                if self.board_info.protocol_minor < 25:
                    if orientation_field:
                        return struct.pack('<d', MecanumRollerOrientation.STANDARD)
                    raise ProtocolError('Oriented mecanum initialization requires protocol 2.25 or newer')
            if command in (SAVE_CONFIGURATION, GET_CONFIGURATION_FIELD) and not orientation_field and not self.supports_configuration_save():
                raise ProtocolError('Configuration storage requires a ready v0.4 controller running protocol 2.14 or newer')
            if command in (SET_DRIVER_SLEEP, GET_DRIVER_SLEEP) and not self.supports_driver_sleep():
                raise ProtocolError('Driver sleep requires READY and a supported board: v0.3/Nano protocol 2.17+, v0.4 protocol 2.15+')
            if command in (SET_BOARD_MOUNT, GET_BOARD_MOUNT) and not self.supports_board_mounting():
                raise ProtocolError('Board-center mounting requires READY, exact v0.4.0 capability and protocol 2.24+')
            if command in (SET_IMU_MOUNT, GET_IMU_MOUNT) and not self.supports_imu_mounting():
                raise ProtocolError('IMU mounting requires READY, exact v0.4.0 capability and protocol 2.23+')
            if command in (GET_FEEDBACK_SOURCE_HEALTH, GET_FEEDBACK_IMU_CHANNELS) and not self.supports_feedback_source_health():
                raise ProtocolError('Feedback source health requires READY, exact v0.4.0 capability and protocol 2.22+')
            if command in (SET_FEEDBACK_SOURCE, GET_FEEDBACK_SOURCE_STATUS, GET_FEEDBACK_SOURCE_VECTOR, SET_FEEDBACK_FUSION_MOUNT, GET_FEEDBACK_FUSION_MOUNT) and not self.supports_feedback_source_selector():
                raise ProtocolError('Feedback source selector requires READY, exact v0.4.0 capability and protocol 2.21+')
            if command in (SET_IMU_TRACKING, GET_IMU_TRACKING_STATUS, GET_IMU_TRACKING_VECTOR, GET_IMU_TRACKING_CONFIGURATION) and not self.supports_imu_tracking():
                raise ProtocolError('IMU tracking requires READY, exact v0.4.0 capability and protocol 2.20+')
            if command in (SET_FUSION_EXPERIMENT, GET_FUSION_STATUS, GET_FUSION_VECTOR, GET_FUSION_CONFIGURATION) and not self.supports_fusion_experiment():
                raise ProtocolError('Fusion experiment requires READY, exact v0.4.0 capability and protocol 2.19+')
            if command in (SET_BOARD_IDLE, GET_BOARD_IDLE) and not self.supports_board_idle():
                raise ProtocolError('Board idle requires READY and a supported profile: v0.3/Nano protocol 2.18+, v0.4 protocol 2.16+')
            if command in (GET_PWM_FREQUENCY, SET_PWM_FREQUENCY):
                if not self._ready or self.board_info is None or self.board_info.protocol_major != 2 or self.board_info.protocol_minor < 13:
                    raise ProtocolError('PWM frequency configuration requires a ready protocol 2.13 or newer controller')
                options = self.get_pwm_frequency_options()
                if self.board_profile is not None and options is None:
                    raise ProtocolError('This board has no configurable PWM frequency')
                if command == SET_PWM_FREQUENCY and options is not None and int.from_bytes(payload, 'little') not in options['supported_hz']:
                    raise ValueError('PWM frequency is not allowed by this board')
            if command in (GET_MOTOR_CURRENT_LIMIT, SET_MOTOR_CURRENT_LIMIT):
                if (self.board_info is None or self.board_info.protocol_major != 2 or self.board_info.protocol_minor < 11):
                    raise ProtocolError('Current-limit setup requires firmware protocol 2.11 or newer')
                options = self.get_motor_current_limit_options(payload[0])
                if self.board_profile is not None and options is None:
                    raise ProtocolError('This motor has no configurable current limit')
                if command == SET_MOTOR_CURRENT_LIMIT and options is not None:
                    threshold = int.from_bytes(payload[2:4], 'little')
                    if threshold not in options['supported_thresholds_ma'] or (not payload[1] and not options['can_disable']):
                        raise ValueError('Current-limit setting is not allowed by this board')
            if command == CLEAR_MOTOR_FAULTS and (self.board_info is None or self.board_info.protocol_minor < 10):
                raise ProtocolError('Clearing motor faults requires firmware protocol 2.10 or newer')
            if command in (GET_MOTOR_FAULTS, CLEAR_MOTOR_FAULTS):
                if (not self._ready or self.board_info is None or
                        self.board_info.protocol_major != 2 or self.board_info.protocol_minor < 8):
                    raise ProtocolError('Motor faults require a ready protocol 2.8+ controller')
                if self.board_profile is not None and not self.supports_motor_faults(payload[0]):
                    raise ProtocolError('This motor has no fault reporting')
            if command == GET_MOTOR_CURRENT:
                if (not self._ready or self.board_info is None or
                        self.board_info.protocol_major != 2 or self.board_info.protocol_minor < 7):
                    raise ProtocolError('Motor current requires a ready protocol 2.7+ controller')
                if self.board_profile is not None and not self.supports_motor_current(payload[0]):
                    raise ProtocolError('This motor has no current sensor')
            if command in SENSOR_COMMANDS:
                kind, sensor_minor = SENSOR_COMMANDS[command]
                if (not self._ready or self.board_info is None or
                        self.board_info.protocol_major != 2 or self.board_info.protocol_minor < sensor_minor):
                    raise ProtocolError(f"{kind.upper() if kind == 'imu' else 'Temperature'} readings require a ready protocol 2.{sensor_minor}+ controller")
                if self.board_profile is not None and not self.supports_sensor(kind, payload[0]):
                    raise ProtocolError(f"This board has no {kind} sensor at that index")
            required_minor = 3 if command in POSITION_PID_COMMANDS else 2 if command in POSITION_COMMANDS else 0
            if required_minor and (not self._ready or self.board_info is None or
                    self.board_info.protocol_major != 2 or self.board_info.protocol_minor < required_minor):
                raise ProtocolError(f"Position command requires a ready protocol 2.{required_minor}+ controller")
            if command in (START_ENCODER_ODOMETRY, GET_ENCODER_ODOMETRY,
                           START_PLATFORM_ODOMETRY, GET_PLATFORM_ODOMETRY, SUBSCRIBE_ODOMETRY) and not self._ready:
                raise ProtocolError("Odometry requires a completed INIT/READY exchange")
            if command == INIT and any(p.command == INIT for p in self._pending.values()):
                raise ProtocolError("INIT is already in progress")
            message_id = self._next_id()
            pending = _PendingRequest(command, response_length, request_payload=bytes(payload))
            if command == INIT:
                if len(payload) != 8:
                    raise ProtocolError("INIT requires an eight-byte payload")
                pending.clock_mode = int(bool(payload[7] & 1))
                self._wall_clock_active = bool(payload[7] & 1)
                self._ready = False
                self._heartbeat_idle_s = None
                self._subscription_samples.clear()
                self._clear_telemetry()
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
            with self._lock:
                self._last_sent = time.monotonic()
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

    def _heartbeat_loop(self, transport, stop):
        """Send a correlated PING only when other outgoing traffic has been idle."""
        while not stop.wait(0.005):
            with self._lock:
                interval = self._heartbeat_idle_s
                due = (self.serial is transport and self._ready and interval is not None
                       and time.monotonic() - self._last_sent >= interval)
            if not due:
                continue
            try:
                self.ping()
            except Exception as error:
                self._fail_session(error, transport)
                return

    def get_subscription_sample(self, source):
        """Return the latest streamed sample (0..3 encoder, 4 platform), or None.

        Samples replace earlier values in a bounded cache. Reading it never sends a
        command or blocks the receive thread; each sample retains its measurement time.
        Board idle pauses events without clearing this cache: a cached sample is
        not proof of a fresh measurement, even immediately after waking.
        """
        if type(source) is not int or not 0 <= source <= 4:
            raise ValueError("source must be 0..4")
        with self._lock:
            return self._subscription_samples.get(source)

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
        if command == TELEMETRY_EVENT:
            if message_id:
                raise ProtocolError('Telemetry events must use message ID zero')
            try:
                event = decode_telemetry_event(payload)
            except ValueError as error:
                raise ProtocolError(str(error)) from error
            if event is not None:
                topic, index, sample = event
                with self._lock:
                    entry = self._telemetry.get((topic, index))
                    if self.serial is transport and self._ready and entry is not None and not entry.closing:
                        entry.latest = sample
                        self._telemetry_wakeup.set()
            return
        if command in (ENCODER_ODOMETRY_EVENT, PLATFORM_ODOMETRY_EVENT):
            encoder = command == ENCODER_ODOMETRY_EVENT
            if message_id or len(payload) != (19 if encoder else 34) or (encoder and payload[0] > 3):
                raise ProtocolError("Malformed odometry event")
            source = payload[0] if encoder else 4
            sample = EncoderOdometrySample.decode(payload[1:]) if encoder else PlatformOdometrySample.decode(payload)
            if sample.clock_mode not in (0, 1) or sample.clock_quality not in (1, 2):
                raise ProtocolError("Invalid odometry event clock metadata")
            with self._lock:
                if self.serial is transport and self._ready:
                    self._subscription_samples[source] = sample
            return
        with self._lock:
            # Recheck under the mutation lock: reconnect may have replaced this transport.
            if self.serial is not transport:
                return
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
                    self._heartbeat_idle_s = None
                    self._subscription_samples.clear()
                    self._clear_telemetry()
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
                if (identity.protocol_major != MIN_PROTOCOL_VERSION[0] or
                        identity.protocol_minor < MIN_PROTOCOL_VERSION[1]):
                    raise ProtocolError("The controller reported an incompatible protocol version")
                self.board_info = identity
                pending.payload = payload
                return # READY is the final response to INIT.
            # Commit acknowledged settings in wire order before waking callers. Their
            # scheduling order must not overwrite newer settings or a new session.
            if command == SET_HEARTBEAT_CONFIG:
                enabled, timeout_ms = struct.unpack("<BI", pending.request_payload)
                self._heartbeat_idle_s = timeout_ms / 5000 if enabled else None
                if not enabled:
                    self._subscription_samples.clear()
                    self._clear_telemetry()
            elif command in (SUBSCRIBE, SUBSCRIBE_WITH_MODE, SUBSCRIBE_CONFIGURED, UNSUBSCRIBE):
                key = tuple(pending.request_payload[:2])
                old = self._telemetry.pop(key, None)
                if old is not None:
                    old.closing = True
                if command in (SUBSCRIBE, SUBSCRIBE_WITH_MODE, SUBSCRIBE_CONFIGURED):
                    entry = self._telemetry_candidates.get(key)
                    if entry is None or entry.session != self._session or entry.closing:
                        raise ProtocolError('Subscription acknowledged after session invalidation; reconnect')
                    self._telemetry[key] = entry
                if key[0] == Topic.ENCODER_ODOMETRY:
                    self._subscription_samples.pop(key[1], None)
                elif key[0] == Topic.PLATFORM_ODOMETRY:
                    self._subscription_samples.pop(4, None)
            elif command in (SUBSCRIBE_ODOMETRY, UNSUBSCRIBE_ODOMETRY):
                source = pending.request_payload[0]
                key = (Topic.PLATFORM_ODOMETRY, 0) if source == 4 else (Topic.ENCODER_ODOMETRY, source)
                entry = self._telemetry.pop(key, None)
                if entry is not None:
                    entry.closing = True
                self._subscription_samples.pop(source, None)
            pending.payload = payload
            pending.event.set()

    def _fail_session(self, error, transport):
        """Fail waiting callers and close only the session that experienced the failure."""
        with self._lock:
            if self.serial is not transport:
                return
            self.serial = None
            self._ready = False
            self.board_info = self.clock_mode = None
            self._heartbeat_idle_s = None
            self._subscription_samples.clear()
            self._clear_telemetry()
            self._stop.set()
            self._telemetry_wakeup.set()
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
        heartbeat = self._heartbeat_thread
        if heartbeat is not None and heartbeat is not threading.current_thread():
            heartbeat.join(timeout=self.request_timeout + 0.2)

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
