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

Generic subscriptions require protocol 2.6 or newer within major 2. `supports_topic(topic, index=0)` combines protocol support with the bundled board profile. Unknown boards return false from this query; explicit requests defer hardware validation to firmware. Older firmware can still use GET methods and the retired odometry API.

Callbacks run on one dedicated worker, separately from serial reads and heartbeat. A slow callback delays other callbacks and coalesces intermediate samples into the latest value. Callback exceptions are stored in `last_telemetry_error`. A callback already executing may finish after unsubscribe or disconnect, but queued samples and handles are cleared. Timestamps preserve integer microseconds and clock metadata; odometry timestamps mark acquisition, while sensor timestamps mark register-read completion.

`subscribe_odometry(source, interval_ms)`, `unsubscribe_odometry(source)`, and their event messages are retired since protocol 2.6 and remain supported throughout major 2. Their removal is eligible in major 3. Use `get_subscription_sample(source)` only with this legacy API: sources 0–3 are encoders and 4 is the platform.

Subscription intervals must be at least twice the calculation period: the default 50 ms calculation period permits 100 ms or longer subscriptions. Keep heartbeat monitoring enabled while subscribed. Disconnect or controller watchdog expiry clears subscriptions; reconnect and explicitly reconfigure motion before resuming.

Heartbeat traffic demonstrates that the SDK connection is alive, not that the application has issued a fresh motor setpoint. Motor stop behavior on connection loss is implemented by firmware.
