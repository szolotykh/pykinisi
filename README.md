# pykinisi
Python package for kinisi motor controller. This package is used to control the kinisi motor controller via serial interface.\
Discription of the commands can be found in [Kinisi Motion Controller framework documentation](https://raw.githubusercontent.com/szolotykh/kinisi-motor-controller-firmware/command-script/commands.md)

## Installation
Install pykinisi with pip:
```bash
pip install pykinisi
```

## Examples
There are several examples in the examples folder which show how to use the pykinisi package to control the kinisi motor controller.\
Run the examples with:
```bash
cd examples
python <example_file> <serial_port>
```

## Development

Install pykinisi for development:
```bash
pip install -e .
```

Update KinisiCommands.py file:
```bash
cd scripts
python generate_commands.py
```

## Links
- [Kinisi Motion Controller firmware](https://github.com/szolotykh/kinisi-motor-controller-firmware)
- [Kinisi Motion Controller hardware](https://github.com/szolotykh/kinisi-motor-controller-board)
- [JavaScipt package for kinisi motor controller](https://github.com/szolotykh/jskinisi)
- [Python package for kinisi motor controller](https://github.com/szolotykh/pykinisi)