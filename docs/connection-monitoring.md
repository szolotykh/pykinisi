# Heartbeat and streamed odometry

SDK 2.1 requires protocol 2.1 firmware. `connect()` completes INIT and clock setup, then enables a 500 ms connection watchdog. A background worker sends PING after 100 ms without outgoing traffic. Ordinary commands and clock replies replace pings. To opt out, construct `KinisiController(heartbeat_timeout_ms=None)`.

Use `set_heartbeat_config(True, timeout_ms)` to change monitoring after connection. The worker adjusts its idle interval to one fifth of the new timeout after the controller acknowledges it. `set_heartbeat_config(False, 500)` disables monitoring and clears subscriptions. `get_heartbeat_config()` reads the board settings. Missing heartbeat replies close the SDK session and populate `last_error`.

```python
from pykinisi import KinisiController

controller = KinisiController()
if not controller.connect("COM3"):
    raise controller.last_error

controller.initialize_encoder(0, 1425.1, False)
controller.start_encoder_odometry(0)
controller.subscribe_odometry(0, 100)

# In the application's update loop; None until the first event arrives.
sample = controller.get_subscription_sample(0)
if sample is not None:
    print(sample.timestamp_us, sample.angle)

controller.unsubscribe_odometry(0)
controller.disconnect()
```

Sources 0–3 select encoder odometry; source 4 selects platform odometry. Each source has a single latest-sample cache, so a slow consumer cannot accumulate an unbounded history. Cache reads do not send commands. GET commands remain available for polling. Sample timestamps describe measurement time, using the same clock mode and quality as GET responses.

Subscription intervals must be at least twice the calculation period: the default 50 ms calculation period permits 100 ms or longer subscriptions. Keep heartbeat monitoring enabled while subscribed. Disconnect or controller watchdog expiry clears subscriptions; reconnect and explicitly reconfigure motion before resuming.

Heartbeat traffic demonstrates that the SDK connection is alive, not that the application has issued a fresh motor setpoint. Motor stop behavior on connection loss is implemented by firmware.
