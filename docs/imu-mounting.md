# Install the IMU on a planar chassis

Protocol 2.23 adds one rigid mounting configuration shared by IMU-only tracking
and fused tracking on the Kinisi Mini v0.4.0. Set it with the robot stopped and
disarmed. The SDK checks the READY board identity, protocol and `imu_mounting` capability.

## Measure the installation

Use the platform's tracking origin, such as the midpoint of a differential axle
or the center of the omni or mecanum wheel geometry. Measure **from this point to
the BMI088 package U601** in chassis axes. X is forward, Y is left and Z is up. A
chip 100 mm right of the origin has Y = −0.100 m at every board orientation.

Board +X points away from the USB-C edge (J501), board +Y points toward the long
edge nearest U601, and board +Z points out of the component face. At zero angles
these axes align with chassis X, Y and Z. Use right-handed angles in degrees.
The rotation from board to chassis for a column vector is
`Rz(yaw) Ry(pitch) Rx(roll)`. Positive yaw turns board +X left. A +90° roll
points board +Y up and the component face to the right. A 180° roll points the
component face down. Firmware accounts for the sensing axes inside the package.

## Configure and verify a vertical right-side installation

```python
if controller.supports_imu_mounting():
    # Only after stopping and disarming the chassis.
    controller.set_imu_mount_geometry(90, 0, 0, 0, -0.100, 0)
    mount = controller.get_imu_mount_geometry()
    assert mount.configured and mount.roll_deg == 90 and mount.y_m == -0.100
```

The geometry helper raises `ValueError` for an invalid value before writing.
A firmware refusal raises `ControllerError`; its error code identifies the reason.

The six arguments are roll, pitch, yaw in degrees and X, Y, Z in metres.
Limits are inclusive: ±180° roll and yaw, ±90° pitch and ±10 m per axis.
For the raw interface, `set_imu_mount(roll_cdeg, pitch_cdeg, yaw_cdeg, x_um, y_um, z_um)`
uses signed centidegrees and micrometres. `get_imu_mount()` returns the applied
integer values and `configured`. The example is `(9000, 0, 0, 0, -100000, 0)`.
Use `SAVE_CONFIGURATION` to persist an installation. A changed mounting or reboot
requires fresh tracking initialization. Perform stationary calibration at true rest.
Motion targets, bias and tracking origins are volatile.

## Upgrade a yaw-only installation

With `configured=false`, existing start and setup calls use their original yaw.
IMU-only uses **sensor +X** and fusion uses **board +X**. To reproduce a centered,
level legacy mount explicitly, set roll, pitch and offsets to zero. For fusion use
the previous board yaw converted to degrees. For IMU-only convert the old sensor
yaw to degrees, add 180° and wrap to [−180°, +180°]. Once configured, the stored
board-to-chassis rotation takes precedence over both legacy yaw arguments.
