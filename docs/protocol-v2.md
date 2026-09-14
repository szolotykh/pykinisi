# Messages API v2

pykinisi uses messages API **2.0.0**. API 1.x firmware and clients are
incompatible with the new framing and responses; update firmware and the SDK
together. Existing motor and GPIO method arguments remain the same. Methods
that set values now wait for the controller's acknowledgement and can raise a
controller error or timeout.

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
defines command arguments and their possible controller errors. This change
implements initialization, ordinary request/response commands, and time sync;
it does not add subscriptions or a motor-disconnect watchdog.

## Development validation

Run `python -m unittest discover -s tests` from the project root. Tests cover
fragmented serial traffic, initialization, idle sync, errors, concurrent callers,
timeouts, cancellation, and generated payloads. When the sibling firmware checkout
and a host C compiler are available, the suite also builds its production protocol
and time-sync code for Python/C interoperability tests. Set `CC` to the host GCC
or Clang executable to enable those tests; otherwise they are skipped.
Physical serial timing and behavior still require validation on a board.
