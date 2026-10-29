# Messages API v2

pykinisi includes messages API **2.11.0** and connects to firmware **2.1+** within
protocol major 2, gating newer commands separately. API 1.x firmware and clients are
incompatible with the new framing and responses; update firmware and the SDK
together. Existing motor and GPIO method arguments remain the same. Methods
that set values now wait for the controller's acknowledgement and can raise a
controller error or timeout.

The protocol uses `major.minor.patch`. Major changes break the existing wire
interface and require matching supported majors. Minor versions add compatible
commands while preserving existing IDs, layouts, units and meanings. Patch
versions provide compatible fixes and do not block connection. An incompatible
replacement command needs a new ID while retaining the original, or a major
version bump. Protocol versions are independent of board revisions and SDK
package versions. INIT does not translate between protocol majors.

## Connect and identify the board

```python
from pykinisi import KinisiController

controller = KinisiController(request_timeout=1.0, init_timeout=5.0)
if not controller.connect("COM3"):
    raise SystemExit(f"Connection failed: {controller.last_error}")

try:
    board = controller.board_info
    print(f"Board: {board.board_major}.{board.board_minor}.{board.board_patch}")
    print(controller.get_time_status())
finally:
    controller.disconnect()
```

Timeouts are in seconds. `connect()` returns `True` only after receiving both
the INIT identity response and READY. Opening the serial port alone does not
complete a connection. Open or handshake failures return `False`, with the
exception in `controller.last_error`. `controller.ready` and
`controller.clock_mode` expose the resulting session state. Reconnect with
`connect()` to establish a fresh session.

If a command reports `INIT_REQUIRED` or `CLOCK_NOT_READY`, `ready` becomes false
and further odometry requests are blocked until a new INIT/READY exchange succeeds.

The SDK identifies itself as Python, reports its library version and protocol
version, and declares whether it can provide valid wall-clock time. INIT does
not enable motors or start odometry.

`controller.board_info` contains these fields from the board:

| Fields | Meaning |
| --- | --- |
| `board_model` | Board model identifier; 1 is the Kinisi motor controller. |
| `board_major`, `board_minor`, `board_patch` | Board revision selected in the firmware build; V3 is `0.3.0`. |
| `protocol_major`, `protocol_minor`, `protocol_patch` | Firmware messages API version. |
| `firmware_build_high`, `firmware_build_low` | Two words holding the first 16 hexadecimal characters of the firmware Git commit. |

Format the build words as `f"{board.firmware_build_high:08x}{board.firmware_build_low:08x}"`.
This identifies a source commit, not a semantic release version, and does not
identify uncommitted changes. A build without Git metadata reports zero.

## Board profiles and sensors

The SDK includes the public profiles generated from the firmware board JSON.
After INIT, `controller.board_profile` selects an exact match for the board model
and complete hardware revision. The profile is a read-only mapping with `board`,
`resources`, `motor_features` and `sensors`; nested arrays are immutable tuples.
It includes resource counts and sensor IDs/features, and omits wiring, MCU, bus,
timer and component details. It is cleared on disconnect, connection failure and
the start of a new INIT. No runtime download or custom profile loading is needed.

```python
from pykinisi import KinisiController

with KinisiController() as controller:
    if not controller.connect("COM3"):
        raise RuntimeError(controller.last_error)
    profile = controller.board_profile
    if profile is not None:
        print(profile["resources"]["motors"])
    if controller.supports_sensor("temperature", 0):
        print(controller.get_temperature(0))  # degrees Celsius
    if controller.supports_sensor("imu", 0):
        sample = controller.get_imu(0)
        print(sample.acceleration_z, sample.angular_velocity_z)
```

`supports_sensor(kind, index=0)` combines known hardware presence with firmware
protocol support. It describes availability, not session readiness or sensor
health. The exported helpers `get_board_profile(info)`,
`has_board_sensor(info, kind, index=0)` and
`supports_board_sensor(info, kind, index=0)` also accept an INIT response or a
mapping of its fields. Physical presence can be checked independently of protocol.

An unknown board has no profile and `supports_sensor` returns `False`, but it can
still connect and use compatible commands. Direct sensor reads on unknown boards
rely on firmware availability errors. A known absent sensor or invalid sensor ID
is rejected before I/O. No profile is borrowed from a different board revision.
Per-motor `current_sense`, `current_limit`, and `fault_reporting` flags describe fitted hardware;
current reads and subscriptions require protocol 2.7. Use
`supports_motor_current(index)` to check availability and see
[motor current](connection-monitoring.md) for usage.
Adjustable-current commands remain unimplemented in firmware.

| Method | Minimum protocol | Result |
| --- | --- | --- |
| `get_temperature(sensor_index)` | 2.4 | Finite float in degrees Celsius |
| `get_imu(sensor_index)` | 2.5 | `ImuSample` with six finite floats |

Both require a completed INIT/READY exchange. Sensor indices are integers from
0 to 255; the board profile determines which indices are fitted. Temperature uses
an 8-byte little-endian double. IMU returns six doubles (48 bytes):
`acceleration_x`, `acceleration_y`, `acceleration_z` in m/s², and
`angular_velocity_x`, `angular_velocity_y`, `angular_velocity_z` in rad/s.
Acceleration includes gravity. Values use native sensor axes; accelerometer and
gyroscope samples are independent, with no orientation, fusion or calibration.

Firmware can return `SAMPLE_NOT_AVAILABLE` during sensor startup/recovery,
`SENSOR_NOT_AVAILABLE` for an absent sensor, or `INTERNAL_ERROR` for sensor I/O
or configuration failure. These are `ControllerError` replies and leave the
session usable. See [Sensors.py](../examples/Sensors.py) for a runnable example
that handles sensor errors without enabling motors.

Maintainers update the bundled files explicitly using the local or branch import
commands in the [README](../README.md#package-development). Wheels and source
distributions include `pykinisi/board_profiles.json`; source distributions also
include the command schema snapshot and update scripts. Rejecting an old schema
that lacks required SDK exports does not prevent connecting to older firmware.

## Clock setup and periodic sync

The default `wall_clock=True` advertises that the host can provide valid Unix
time. After INIT, the controller sends time-sync requests. The SDK's background
reader responds with host receive/send timestamps and the matching message ID.
The controller selects a valid timing sample, installs its clock mapping, and
sends READY. This exchange does not require application calls.

The controller repeats synchronization every **30 seconds** by default,
including while the application is idle. The reader remains active until
`disconnect()`. To change the interval for the current connection, call
`controller.set_time_sync_interval(interval_ms=30000)` after connecting.
Allowed intervals are 1,000 through 3,600,000 milliseconds. A new INIT restores
the default.

Use `KinisiController(wall_clock=False)` when the host has no valid wall clock.
The controller then sends READY in uptime mode and skips wall-clock exchanges.
The choice is explicit and does not depend on the host platform.

`get_time_status()` returns `clock_mode`, `clock_quality`, `interval_ms`, and
`last_sync_age_us`. Mode 0 means microseconds since controller boot; mode 1 means
Unix microseconds. Quality 0 is unready, 1 is valid, and 2 is stale. Sync age is
zero in uptime mode. A failed periodic sync retains the previous mapping;
quality becomes stale after the interval plus three seconds without a
successful update. Initial sync failure makes `connect()` fail.

## Timestamped odometry

Initialize the encoder or platform, then start odometry before reading it.
`connect()` has already waited for READY, so no additional clock handshake is
needed when starting odometry.

| Method | Result fields |
| --- | --- |
| `get_encoder_odometry(encoder_index)` | `timestamp_us`, `clock_mode`, `clock_quality`, `angle` |
| `get_platform_odometry()` | `timestamp_us`, `clock_mode`, `clock_quality`, `x`, `y`, `t` |

Encoder odometry now returns an `EncoderOdometrySample` instead of a scalar;
read `.angle` for the angle in radians. Platform odometry returns a
`PlatformOdometrySample`; existing `.x`, `.y`, and `.t` fields remain available
(metres and radians). The timestamp identifies the controller's saved
measurement, rather than when Python received the reply. Repeated GETs may
return the same sample. Clock quality describes the mapping at reply time.

Immediately after start or reset, GET can return `SAMPLE_NOT_AVAILABLE` until
the first new measurement is captured. Reading odometry that is stopped or
not started returns `ODOMETRY_NOT_INITIALIZED`. See
[`PlatformOdometry.py`](../examples/PlatformOdometry.py) for polling and cleanup.

Wall-clock timestamps can move forward or backward when synchronization
updates the mapping. Re-reading a cached sample after sync can therefore
produce a slightly different wall timestamp. Microsecond units do not imply
microsecond accuracy. Use a monotonic host clock for elapsed-time deadlines.

## Errors and request matching

```python
from pykinisi import ControllerError, ErrorCode

try:
    sample = controller.get_platform_odometry()
except ControllerError as error:
    if error.error_code == ErrorCode.SAMPLE_NOT_AVAILABLE:
        sample = None  # Wait for the next polling interval.
    else:
        raise
```

`ControllerError` represents an ERROR reply from the board and exposes
`error_code`, `command`, and `message_id`. `RequestTimeoutError` reports a
request that did not receive a matching response before its deadline;
`ProtocolError` reports invalid protocol traffic; `ConnectionClosedError`
reports an unavailable connection. These exception types are exported from
`pykinisi`. Invalid frame parsing closes the session instead of continuing on
a potentially misaligned stream.

A timeout does not prove a command was rejected: the command may have executed
before its reply was lost. Decide whether retrying a particular operation is
appropriate; the SDK does not automatically repeat motor commands.

All frames use `[length:u8][command:u8][message_id:u16][payload]`, with
little-endian multi-byte fields and length excluding itself. Client message
IDs increment and wrap from 65535 to 1. Replies echo the request ID and command;
ERROR additionally identifies the failed command. Controller-initiated sync
requests have their own IDs and are handled independently of ordinary replies.
Successful setters have an empty ACK payload. Application code should use the
SDK methods rather than read or write the serial port alongside its reader.
Timed-out IDs are skipped until reconnect so delayed replies cannot complete a
new request. If all IDs have been retired or remain outstanding, reconnect to
start a fresh session.

The firmware's generated [command reference](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/commands.md)
defines command arguments and their possible controller errors. See
[connection monitoring and streamed odometry](connection-monitoring.md) for
heartbeat configuration, latest-sample access and subscription lifecycle.

## Development validation

Run `python -m unittest discover -s tests` from the project root. Tests cover
fragmented serial traffic, initialization, idle sync, errors, concurrent callers,
timeouts, cancellation, and generated payloads. When the sibling firmware checkout
and a host C compiler are available, the suite also builds its production protocol
and time-sync code for Python/C interoperability tests. Set `CC` to the host GCC
or Clang executable to enable those tests; otherwise they are skipped.
Physical serial timing and behavior still require validation on a board.

## Velocity and position control

Connections accept protocol 2.1+. Position commands require READY and protocol
2.2+, and full position PID initialization requires 2.3+. Unsupported position
calls are rejected locally before transmission. SDK version and minimum
supported firmware protocol are separate.

Both velocity initialization methods default `integral_limit` to **100 PWM
percentage points** when omitted. Total PWM remains bounded to +/-100% with
firmware anti-windup. Ki=0 or an integral limit of zero disables integration on
firmware 2.3.1. Gains from earlier firmware must be retuned; the examples use
Kp=1, Ki=1, Kd=0 only as starting values.

Initialize velocity control before position control. Motor position is
continuous radians; platform position is world-frame x/y in meters and heading
in radians. Position integral limits are velocity contributions (rad/s or m/s),
not PWM limits. Reset changes the origin and clears the target. Velocity
commands override position mode; stop/brake or reinitialization can require
position setup again. See [motor](../examples/MotorPositionController.py) and
[platform](../examples/PlatformPositionController.py) examples for initialization,
reset, target and readback sequences.

## Motor current limits

Current-limit configuration requires protocol 2.11 and fitted hardware. The public
profile supplies numeric `supported_thresholds_ma` and `can_disable`; options are
hardware metadata, while the support helper also checks firmware availability.
Unknown/unsupported motors have no options. Values use integer milliamps and SDKs
reject settings outside the known profile before sending. Firmware validates them
independently. No board-specific enum is required.

```python
options = controller.get_motor_current_limit_options(0)
if controller.supports_motor_current_limit(0):
    controller.initialize_motor(0, False)
    applied = controller.set_motor_current_limit(0, True, 5400)
    actual = controller.get_motor_current_limit(0)
    print(options["supported_thresholds_ma"], applied.threshold_ma, actual.enabled)
```

Both GET and SET require an initialized, awake motor. SET stops the motor and its
controller, verifies the applied setting, and leaves the motor stopped even on
I/O failure. Platform-owned motors reject SET; release ownership first. GET does
not change motor state. Settings are volatile: motor initialization reapplies
board defaults, and driver power loss requires reinitialization. Disabling current
regulation leaves overcurrent protection unchanged.

An unknown board profile does not advertise support; explicit requests with a
compatible protocol can still reach firmware. No custom profiles are loaded.
See [firmware current-limit behavior](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/docs/motor-current-limit.md)
for errors, persistence, register behavior, and wire sizes.


## Motor PWM frequency

PWM frequency is one setting for all motors. It requires protocol 2.13 and a ready
connection; motors need not be initialized. Supported enum values are `KHZ_10`,
`KHZ_20`, `KHZ_25`, `KHZ_50`, and `KHZ_100`, represented in hertz on the wire.
Use the selected board profile's `pwm_frequency.supported_hz` to populate choices.
The default is available as `pwm_frequency.default_hz`.

Applying a frequency stops every motor and motor/platform controller and leaves
them stopped. It preserves duty-cycle resolution, motor direction configuration,
and driver current-limit settings. Restart motion explicitly. The selected frequency
survives motor initialization and disconnect/reconnect; reboot restores the default.
GET reads the board's active frequency selection without changing outputs.
Unsupported values are rejected; they are never silently rounded.

```python
from pykinisi import PwmFrequency

if controller.supports_pwm_frequency():
    options = controller.get_pwm_frequency_options()
    applied_hz = controller.set_pwm_frequency(PwmFrequency.KHZ_25)
    current_hz = controller.get_pwm_frequency()
```

Unknown boards have no advertised options; explicit commands on compatible firmware
still reach the board for validation. Older firmware is rejected locally before I/O.
