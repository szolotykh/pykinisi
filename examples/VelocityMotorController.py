# Filename: VelocityMotorController.py
# Description: Example of velocity motor controller

import time
from pykinisi import *
from examples.Core import *

controller = InitTest()

targetSpeed = 50
targetDirection = True

motorIndex = MotorIndex.Motor0

# Blink status LED
controller.toggle_status_led()
time.sleep(1)
controller.toggle_status_led()
time.sleep(1)

# Set motor controller
# No need to initialize motor and encoder
controller.initialize_motor_controller(motorIndex, 0.3, 0.2, 0.1)

# Set target velocity
controller.set_motor_target_velocity(motorIndex, targetDirection, 30)
time.sleep(5)

# Delete motor controller
controller.delete_motor_controller(motorIndex)