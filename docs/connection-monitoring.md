# Heartbeat and subscriptions

`connect()` completes INIT and clock setup, then enables a 500 ms connection watchdog. A background worker sends PING after 100 ms without outgoing traffic. Ordinary commands and clock replies replace pings. To opt out, construct `KinisiController(heartbeat_timeout_ms=None)`.

Use `set_heartbeat_config(True, timeout_ms)` to change monitoring after connection. The worker adjusts its idle interval to one fifth of the new timeout after the controller acknowledges it. `set_heartbeat_config(False, 500)` disables monitoring and clears subscriptions. `get_heartbeat_config()` reads the board settings. Missing heartbeat replies close the SDK session and populate `last_error`.

```python
from pykinisi import KinisiController, Topic
import time

controller = KinisiController()
if not controller.connect("COM3"):
    raise controller.last_error

controller.initialize_encoder(0, 1425.1, False)
controller.start_encoder_odometry(0)
try:
    with controller.subscribe(Topic.ENCODER_ODOMETRY,
                              lambda sample: print(sample.timestamp_us, sample.angle),
                              index=0, interval_ms=100) as subscription:
        time.sleep(1)  # The SDK services serial traffic and heartbeat in the background.
finally:
    controller.disconnect()
```

`subscribe(topic, callback, *, index=0, interval_ms=...)` returns a `Subscription` with `active` and `unsubscribe()`. It also works as a context manager. A replacement owns the same topic/index; an old handle cannot unsubscribe the replacement. `controller.unsubscribe(topic, index=0)` removes that source directly. GET commands remain available.

| Topic | Index | Sample | Minimum interval |
| --- | --- | --- | --- |
| `Topic.ENCODER_ODOMETRY` | Encoder index | `EncoderOdometrySample` | Twice the calculation period |
| `Topic.PLATFORM_ODOMETRY` | 0 | `PlatformOdometrySample` | Twice the calculation period |
| `Topic.TEMPERATURE` | Sensor index | `TemperatureSample`, including `celsius` | 250 ms |
| `Topic.IMU` | Sensor index | `ImuTelemetrySample`, acceleration and angular velocity | 10 ms |
| `Topic.MOTOR_CURRENT` | Motor index (0–3) | `MotorCurrentSample`, including `amps` | 20 ms |

Motor current requires protocol 2.7 and fitted sensing hardware. Pull and streamed
values are nonnegative filtered high-side current estimates in amperes. PWM
affects the reading; it is not signed winding or battery current. Reading never
enables a motor. Saturation returns `SAMPLE_OUT_OF_RANGE`; a failed streamed read
pauses delivery until recovery.

```python
if controller.supports_motor_current(0):
    print(controller.get_motor_current(0))  # amperes
    with controller.subscribe(Topic.MOTOR_CURRENT,
                              lambda sample: print(sample.amps, sample.timestamp_us),
                              index=0, interval_ms=100):
        time.sleep(1)
```

Generic subscriptions require protocol 2.6 or newer within major 2. `supports_topic(topic, index=0)` combines protocol support with the bundled board profile. Unknown boards return false from this query; explicit requests defer hardware validation to firmware. Older firmware can still use GET methods and the retired odometry API.

Callbacks run on one dedicated worker, separately from serial reads and heartbeat. A slow callback delays other callbacks and coalesces intermediate samples into the latest value. Callback exceptions are stored in `last_telemetry_error`. A callback already executing may finish after unsubscribe or disconnect, but queued samples and handles are cleared. Timestamps preserve integer microseconds and clock metadata; odometry timestamps mark acquisition, while sensor timestamps mark register-read completion.

`subscribe_odometry(source, interval_ms)`, `unsubscribe_odometry(source)`, and their event messages are retired since protocol 2.6 and remain supported throughout major 2. Their removal is eligible in major 3. Use `get_subscription_sample(source)` only with this legacy API: sources 0–3 are encoders and 4 is the platform.

Odometry subscription intervals must be at least twice the calculation period: the default 50 ms calculation period permits 100 ms or longer subscriptions. Keep heartbeat monitoring enabled while subscribed. Disconnect or controller watchdog expiry clears subscriptions; reconnect and explicitly reconfigure motion before resuming.

Heartbeat traffic demonstrates that the SDK connection is alive, not that the application has issued a fresh motor setpoint. Motor stop behavior on connection loss is implemented by firmware.

## Motor faults

Fault reporting requires protocol 2.8 and fitted hardware. Read `status_valid`
before interpreting diagnostic flags; an asleep or unavailable driver must not
be shown as healthy. Reads do not wake the driver, initialize the motor, clear
latched faults, or change the outputs. Missing motor power cannot be distinguished
from communication or driver failure by an unavailable response alone. Existing
motor reinitialization clears latched driver faults. Fault subscriptions have a
100 ms minimum interval and report snapshots, not a fault-event history.

```python
if controller.supports_motor_faults(0):
    status = controller.get_motor_faults(0)
    print(status.overcurrent if status.status_valid else 'Diagnostics unavailable')
    with controller.subscribe(Topic.MOTOR_FAULTS,
                              lambda sample: print(sample.status_valid, sample.fault_active),
                              index=0, interval_ms=250):
        time.sleep(1)
```

See the [firmware fault reference](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/docs/motor-faults.md) for every diagnostic field and hardware limitations.


## Subscription delivery modes

`PERIODIC` (0) sends the latest sample at each interval. `ON_CHANGE` (1)
sends an initial snapshot, then another when the encoded value fields change.
Timestamp and clock metadata are excluded from the comparison. Fault appearance,
clearance, sleep, and diagnostic validity changes are included in the value fields.
Numeric measurements use exact encoded comparisons, with no deadband.

The interval controls how often firmware checks for changes; topic minimums still
apply. Faults can be checked every 100 ms. This is sampled detection, not an
interrupt-driven guarantee. A quiet change subscription remains active, and the
connection heartbeat runs independently. Hardware protection remains independent
of telemetry delivery.

Each connection keeps its own baseline. Replacing a subscription requests a fresh
initial snapshot. Failed reads pause delivery; the next successful read sends a
snapshot even if the value matches the previous one. A failed transmit does not
advance the baseline. Slow transports receive the latest state and can skip
intermediate transitions; subscriptions are not a fault history. On I2C, the
master must continue servicing `poll()` to retrieve queued samples.

Delivery modes require protocol 2.9. The SDK keeps the original `SUBSCRIBE`
request for periodic mode, so existing applications and older compatible firmware
keep working. Other modes use `SUBSCRIBE_WITH_MODE` (0x83), with the same topic,
index and uint32 interval followed by a uint8 mode. Events and unsubscribe keep
their existing formats. Unsupported modes are rejected before transmission.

```python
from pykinisi import Topic, SubscriptionMode

faults = controller.subscribe(
    Topic.MOTOR_FAULTS, print, index=0, interval_ms=100,
    mode=SubscriptionMode.ON_CHANGE,
)
# Continue application work; callbacks run when samples arrive.
faults.unsubscribe()
```


## Clearing motor faults

`clear_motor_faults(motor_index)` requires protocol 2.10 and returns fresh
motor-fault status. Firmware stops the motor and its active controller, clears
latched flags, and leaves the output disabled with zero PWM. Read and record
diagnostics first when investigating an issue. Existing active conditions may
still be reported after the clear; inspect the response before restarting.

The motor must already be initialized and its driver awake. Platform-owned motors
are rejected with `MOTOR_OWNED`; initialization is checked next. SPI or verification
failures return `INTERNAL_ERROR` and leave the motor stopped once execution has
begun. No automatic initialization, wake-up, retry, or motor restart is performed.
The existing per-motor `fault_reporting` profile flag identifies the supported
driver hardware, while protocol 2.10 identifies command availability.

See [firmware fault reporting](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/docs/motor-faults.md)
for register behavior and the meaning of each returned flag.

```python
status = controller.clear_motor_faults(0)
print(status.status_valid, status.fault_active, status.overcurrent)
```

## Trigger and delivery settings

`SUBSCRIBE_CONFIGURED` (protocol 2.12) separates when to read from what to send.
It sends an initial snapshot, then applies these settings:

| Setting | Values | Meaning |
| --- | --- | --- |
| `trigger` | `PERIODIC` (0), `ON_EVENT` (1) | Read on the polling schedule or on a source notification. |
| `delivery` | `ALL` (0), `ON_CHANGE` (1) | Send every triggered update, or only changed value fields. |
| `poll_interval_ms` | Integer milliseconds | Required for periodic triggering; zero for event triggering. Topic minimums apply. |
| `max_silence_ms` | Integer milliseconds | For ON_CHANGE, send a fresh snapshot after this long without a successful send, even if unchanged. Zero disables refresh. |

Nonzero intervals must meet the topic minimum and cannot exceed 3,600,000 ms.
ALL requires zero maximum silence. Timestamp and clock metadata do not count as
value changes. Odometry retains its acquisition timestamp; refresh does not start
or restart calculation. Heartbeat monitoring remains independent and required.

Only MOTOR_FAULTS currently supports ON_EVENT. The fault GPIO interrupt records a
notification; the command task reads diagnostics and sends them as soon as it can.
A 100 ms background check catches additional diagnostic changes while the fault
pin is already asserted, and recovery conditions without an interrupt. Driver
initialization enables thermal-warning reporting on the fault pin while preserving
protection settings. Current-regulation reporting on that pin remains unchanged.

ON_EVENT + ALL sends each observed notification batch, including unchanged values,
and changes found by reconciliation. An unchanged background check is not an event.
ON_EVENT + ON_CHANGE filters repeated values and can refresh after maximum silence.
The firmware stores latest state, not a lossless event queue: bursts can coalesce,
transient conditions may be missed, and bus/task delays affect delivery. Failed
reads do not resend old data. Backpressure does not acknowledge a change until sent.
Maximum silence is a best-effort reporting deadline, not a connection watchdog or
a guarantee during failed reads, unavailable measurements, or a blocked transport.

The request payload is `uint8 topic_id`, `uint8 index`, `uint32 poll_interval_ms`,
`uint8 trigger`, `uint8 delivery`, `uint32 max_silence_ms`, in that order. Events,
unsubscribe, session cleanup and capacity limits remain shared with the existing
subscription commands. Existing interval/mode APIs retain their wire behavior.
Configured subscriptions require 2.12 and are rejected locally on older firmware.

On I2C, the board cannot initiate transfers. Continue calling Arduino `poll()`;
event subscriptions request service every 10 ms (divided among active streams).
USB receives notifications without client polling. Hardware protection acts
independently of subscription and transport timing.

```python
from pykinisi import Topic, SubscriptionTrigger, SubscriptionDelivery

subscription = controller.subscribe(
    Topic.MOTOR_FAULTS, print, index=0,
    trigger=SubscriptionTrigger.ON_EVENT,
    delivery=SubscriptionDelivery.ON_CHANGE,
    max_silence_ms=5000,
)
# Later: subscription.unsubscribe()
```

For periodic triggering, supply `poll_interval_ms`. Do not mix these settings
with legacy `interval_ms`/`mode` options. Callbacks use the bounded telemetry worker.
