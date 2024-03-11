# pykinisi
Python package for kinisi motor controller. This package is used to control the kinisi motor controller via serial interface.\
Description of the commands can be found in [Kinisi Motion Controller framework documentation](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/README.md)

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
    print(f"Can't open serial connection. Port: {port}")
    exit()

# Motor test
motor_index = MotorIndex.Motor0
speed = 40 # Speed in percentage

# Is motor reversed 
is_reversed = False
 
controller.initialize_motor(motor_index, is_reversed)

# Set motor speed to speed
controller.set_motor_speed(motor_index, speed)
time.sleep(5)

# Stop motor
controller.stop_motor(motor_index)
```

## More Examples
There are several examples in the examples folder which show how to use the pykinisi package to control the kinisi motor controller.\
Run the examples with:
```bash
cd examples
python <example_file> <serial_port>
```

## Package Development
Install pykinisi for development:
```bash
pip install -e .
```

Update KinisiCommands.py file:
```bash
cd tools
pip install -r requirements.txt
python update-commands.py --branch=main
```

## Links
- [Kinisi Motion Controller firmware](https://github.com/szolotykh/kinisi-motor-controller-firmware)
- [Kinisi Motion Controller hardware](https://github.com/szolotykh/kinisi-motor-controller-board)
- [JavaScipt package for kinisi motor controller](https://github.com/szolotykh/jskinisi)
- [Python package for kinisi motor controller](https://github.com/szolotykh/pykinisi)