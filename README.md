# pykinisi
Python package for kinisi motor controller. This package is used to control the kinisi motor controller via serial interface.\
Description of the commands can be found in [Kinisi Motion Controller framework documentation](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/README.md)

This version uses messages API **2.0.0**, which is incompatible with API 1.x firmware and clients. Connecting exchanges board identity and completes clock setup before returning. See [initialization, time sync, and errors](docs/protocol-v2.md) for details.

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
speed = 40 # Signed PWM value, -840..840

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
