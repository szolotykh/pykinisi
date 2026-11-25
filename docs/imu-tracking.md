# IMU-only relative tracking

Protocol 2.20 adds a volatile relative tracking experiment on exact model 1,
hardware 0.4.0 with `capabilities.imu_tracking: true`. The SDK requires READY
and a compatible protocol before I/O. Older boards and unknown revisions remain
unsupported; their existing commands continue to work.

This estimator does not require encoders, platform initialization or fusion.
It never sends motion targets, starts motor controllers, or selects feedback for
control. It is not absolute localization, and accuracy is not physically qualified.
The motion-onset diagnostic covers a one-second source interval beginning at the
first qualified source estimate. Read `GET_IMU_TRACKING_CONFIGURATION.window_us`
for the connected board horizon; this v0.4 release returns 1,000,000 microseconds.
The interval is `[origin, origin + H)`: include origin and exclude the end. At
100 Hz, it contains 100 eligible source stamps. No estimate with a source watermark
at or beyond the end is published.

A delayed in-window sample may remain readable after the wall clock reaches the
source end. If tracking remains healthy, it becomes EXPIRED when the first source
watermark at or beyond the end is consumed, or by the first serviced status,
vector or configuration read at or after `end + 50,000 microseconds`. Missing,
stale or invalid source data may fault earlier. This guarantees observable expiry
on a serviced read; it does not guarantee hard real-time task or transport timing.
Status retains the last eligible joint watermark. `elapsed_us` is wall elapsed
time since the source origin, capped at H, and is distinct from that watermark.
It freezes on fault or expiry; STOP clears it.
Another window requires a deliberate stationary restart.

## Deliberate stationary origin

For a rotated or displaced board, enter the [PCB underside-center mounting geometry](imu-mounting.md) on protocol 2.24+.
With legacy yaw-only setup, use a rigid, level mount with **sensor +Z up** and the IMU centered at the chassis
reference point. Chassis axes are forward, left, up. Declare `mounting_yaw_rad`
as the angle of **sensor +X**, counterclockwise about chassis +Z, in [-pi, pi].
The transform is `Rz(yaw) * (sensor_x, sensor_y, sensor_z)`. This declaration uses
sensor axes, not the board outline or the fusion experiment's board-axis angle.
A backwards sensor requires pi radians in this legacy setup.

`SET_IMU_TRACKING(action, stationary_mount_confirmed, mounting_yaw_rad)` supports:

- START (0), RESET (2), RECALIBRATE (3): each performs full cold calibration and
  re-anchors the relative origin from any state. All discard prior bias and pose,
  require confirmation exactly 1 and finite yaw, and require the installation to
  remain at true zero velocity during calibration.
- STOP (1): requires confirmation 0 and yaw 0; ends integration without commanding
  motors. Read state after the ACK.

An ACK confirms the action, not a valid estimate. IMU stationarity checks cannot
distinguish true rest from constant velocity. Caller confirmation is a deliberate
attestation of rest and the supported mounting, not a measured guarantee.
Faults, interruptions and expiry require an explicit restart; reconnect and reads
never start or re-anchor tracking. SAVE_CONFIGURATION can persist the explicit
mount geometry. Tracking state, bias and origin are volatile.

## Diagnostics and freshness

Status contains state, fault, flags, sequence, monotonic joint sample-watermark
`timestamp_us`, tracking `elapsed_us`, and separate accel/gyro ages. UINT32_MAX
means an unavailable age. A vector read returns its sequence plus x/y/z:

- POSE: relative x/y metres and yaw radians.
- TWIST: body-frame vx/vy metres per second and yaw rate radians per second.
- BIAS: accel-X/accel-Y metres per second squared and gyro-Z radians per second.

Only combine status and vector reads with equal sequences. Different sequences
are different snapshots; refresh explicitly rather than inventing a common time.
Require TRACKING and ESTIMATE_VALID for a usable current estimate; software health
is not an accuracy guarantee. Unavailable vectors return SAMPLE_NOT_AVAILABLE.
Status reads may enforce tracking expiry. Cached snapshots continue to age even
when polling is off and must not be presented as live tracking.

Status and vector responses each contain 28 payload bytes plus the four-byte
envelope, fitting the default Arduino Wire32 buffer. Configuration is 12 payload
bytes. Requests and responses are packed little endian; timestamps are uint64.

## Python

```python
from pykinisi import ImuTrackingAction, ImuTrackingVectorKind, ImuTrackingState, ImuTrackingFlag

if controller.supports_imu_tracking():
    # Only after verifying true rest and the supported mounting.
    controller.set_imu_tracking(ImuTrackingAction.START, 1, 0.0)
    status = controller.get_imu_tracking_status()
    if status.state == ImuTrackingState.TRACKING and status.flags & ImuTrackingFlag.ESTIMATE_VALID:
        pose = controller.get_imu_tracking_vector(ImuTrackingVectorKind.POSE)
        same_snapshot = status.sequence == pose.sequence
    # Otherwise read status again later; do not send another START automatically.
```

Timestamps remain exact Python integers. Invalid declarations fail before I/O;
ControllerError preserves device rejection. Retry actions only deliberately.
