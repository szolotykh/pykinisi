# pykinisi
Python package for kinisi motor controller. This package is used to control the kinisi motor controller via serial interface.\
Description of the commands can be found in [Kinisi Motion Controller framework documentation](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/README.md)

API v2 is incompatible with API v1. See [protocol details](docs/protocol-v2.md).

The SDK bundles public profiles for all four supported boards. After connection,
`controller.board_profile` contains the automatically selected hardware profile,
and `controller.supports_sensor('imu')` checks both hardware and firmware support.
Profiles are included in package releases; normal use requires no download.
See [board profiles and sensors](docs/protocol-v2.md#board-profiles-and-sensors).

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
See the [motor position](examples/MotorPositionController.py) and [platform position](examples/PlatformPositionController.py) examples for position control.

There are several examples in the examples folder which show how to use the pykinisi package to control the kinisi motor controller.\
Run the examples with:
```bash
cd examples
python <example_file> <serial_port>
```

`Initialization.py` reports board identity and clock status without enabling motors. Use `python Initialization.py <serial_port> --uptime` when the host has no valid wall clock.
`Sensors.py` reads the available temperature and IMU sensors using the selected profile.

## Package Development
Install pykinisi for development:
```bash
pip install -e .
```

Update commands and board profiles from the project root:
```bash
python tools/update-commands.py --schema ../kinisi-motor-controller-firmware/commands.json
python tools/update-board-profiles.py --profiles ../kinisi-motor-controller-firmware/include/board_profiles.json
```

Each script also accepts `--branch=<firmware-branch>` to explicitly download its
source from that firmware branch. Use `--check` with either source option to
detect stale generated files without writing. Commands are saved as
`tools/commands.json` and generated into `pykinisi/KinisiCommands.py`; public
profiles are saved as `pykinisi/board_profiles.json` and bundled in wheels and
source releases. Invalid input is rejected before replacing existing outputs.
Custom runtime profile loading is not supported.

Run tests from the project root with `python -m unittest discover -s tests`.

## Links
- [Kinisi Motion Controller firmware](https://github.com/szolotykh/kinisi-motor-controller-firmware)
- [Kinisi Motion Controller hardware](https://github.com/szolotykh/kinisi-motor-controller-board)
- [JavaScipt package for kinisi motor controller](https://github.com/szolotykh/jskinisi)
- [Python package for kinisi motor controller](https://github.com/szolotykh/pykinisi)
