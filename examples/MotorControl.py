# Filename: MotorControl.py
# Description: Example of motor control

import time
from Core import *

controller = InitTest()

# Motor test
motorIndex = MotorIndex.Motor0
speed1 = 40
speed2 = 80
speed3 = -40

is_reverse = False

controller.initialize_motor(motorIndex, is_reverse)

# Set motor speed to speed1
controller.set_motor_speed(motorIndex, speed1)
time.sleep(5)

# Set motor speed to speed2
controller.set_motor_speed(motorIndex, speed2)
time.sleep(5)

# Set motor speed to speed1 in reverse
controller.set_motor_speed(motorIndex, speed3)
time.sleep(5)

# Stop motor
controller.stop_motor(motorIndex)