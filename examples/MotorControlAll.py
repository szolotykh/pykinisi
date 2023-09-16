# Filename: MotorControlAll.py
# Description: Example of motor control for all motors

import time
from Core import *

controller = InitTest()

speed1 = 100
speed2 = 60

# Initialize all motors
controller.initialoze_motor_all()

# Set motors speed to speed1
controller.set_motor_speed(MotorIndex.Motor0, True, speed1)
controller.set_motor_speed(MotorIndex.Motor1, True, speed1)
controller.set_motor_speed(MotorIndex.Motor2, True, speed1)
controller.set_motor_speed(MotorIndex.Motor3, True, speed1)
time.sleep(5)

# Set motors speed to speed2
controller.set_motor_speed(MotorIndex.Motor0, True, speed2)
controller.set_motor_speed(MotorIndex.Motor1, True, speed2)
controller.set_motor_speed(MotorIndex.Motor2, True, speed2) 
controller.set_motor_speed(MotorIndex.Motor3, True, speed2)
time.sleep(3)

# Stop all motors
controller.stop_motor_all()
time.sleep(3)

# Set motors speed to speed1 in reverse
controller.set_motor_speed(MotorIndex.Motor0, False, speed1)
controller.set_motor_speed(MotorIndex.Motor1, False, speed1)
controller.set_motor_speed(MotorIndex.Motor2, False, speed1)
controller.set_motor_speed(MotorIndex.Motor3, False, speed1)
time.sleep(3)

# Stop all motors
controller.stop_motor_all()