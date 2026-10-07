# pykinisi
Python package for kinisi motor controller. This package is used to control the kinisi motor controller via serial interface.\
Description of the commands can be found in [Kinisi Motion Controller framework documentation](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/README.md)

This version uses messages API **2.3.1**, which is incompatible with API 1.x firmware and clients. Connecting exchanges board identity and completes clock setup before returning. See [initialization, time sync, and errors](docs/protocol-v2.md) for details.

## Installation
Install pykinisi with pip:
```bash
pip install pykinisi
```

## Controlling Single Motor
This example shows how to connect to the motor controller and control a single motor.\
```python
import time
from pykinisi import *

port = "COM3"

controller = KinisiController()
if not controller.connect(port):
    print(f"Can't connect to {port}: {controller.last_error}")
    exit()

# Motor test
motor_index = MotorIndex.Motor0
speed = 40 # Signed PWM percentage, -100..100

# Is motor reversed 
is_reversed = False
 
try:
    controller.initialize_motor(motor_index, is_reversed)
    # Set motor speed to speed
    controller.set_motor_speed(motor_index, speed)
    time.sleep(5)
finally:
    try:
        controller.stop_motor(motor_index)
    finally:
        controller.disconnect()
```

## More Examples
There are several examples in the examples folder which show how to use the pykinisi package to control the kinisi motor controller.\
Run the examples with:
```bash
cd examples
python <example_file> <serial_port>
```

`Initialization.py` reports board identity and clock status without enabling motors. Use `python Initialization.py <serial_port> --uptime` when the host has no valid wall clock.

## Package Development
Install pykinisi for development:
```bash
pip install -e .
```

Update KinisiCommands.py file:
```bash
cd tools
python update-commands.py --schema ../../kinisi-motor-controller-firmware/commands.json
```

The command above uses a sibling firmware checkout. Use `--branch=<firmware-branch>` to fetch a firmware branch containing the API v2 schema instead.

Run tests from the project root with `python -m unittest discover -s tests`.

## Links
- [Kinisi Motion Controller firmware](https://github.com/szolotykh/kinisi-motor-controller-firmware)
- [Kinisi Motion Controller hardware](https://github.com/szolotykh/kinisi-motor-controller-board)
- [JavaScipt package for kinisi motor controller](https://github.com/szolotykh/jskinisi)
- [Python package for kinisi motor controller](https://github.com/szolotykh/pykinisi)

## Velocity and position control (2.3.1)

Both velocity initialization methods default `integral_limit` to **100 PWM
percentage points** when omitted. Total PWM remains bounded to +/-100% with
firmware anti-windup. Ki=0 or an integral limit of zero disables integration on
firmware 2.3.1. Gains from earlier firmware must be retuned; the examples use
Kp=1, Ki=1, Kd=0 only as starting values.

Connections still accept protocol 2.1+. Position commands require 2.2+, and
full position PID initialization requires 2.3+. Unsupported position calls are
rejected locally before transmission. SDK version and minimum supported
firmware protocol are separate.

Initialize velocity control before position control. Motor position is
continuous radians; platform position is world-frame x/y in meters and heading
in radians. Position integral limits are velocity contributions (rad/s or m/s),
not PWM limits. Reset changes the origin and clears the target. Velocity
commands override position mode; stop/brake or reinitialization can require
position setup again. See [motor](examples/MotorPositionController.py) and
[platform](examples/PlatformPositionController.py) examples for initialization,
reset, target and readback sequences.
