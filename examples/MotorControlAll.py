# Filename: MotorControlAll.py
# Description: Example of motor control for all motors

import time
from Core import *

controller = InitTest()

speed1 = 80
speed2 = 40

# Initialize all motors
controller.initialize_motor(MotorIndex.Motor0, False)
controller.initialize_motor(MotorIndex.Motor1, False)
controller.initialize_motor(MotorIndex.Motor2, False)
controller.initialize_motor(MotorIndex.Motor3, False)


# Set motors speed to speed1
controller.set_motor_speed(MotorIndex.Motor0, speed1)
controller.set_motor_speed(MotorIndex.Motor1, speed1)
controller.set_motor_speed(MotorIndex.Motor2, speed1)
controller.set_motor_speed(MotorIndex.Motor3, speed1)
time.sleep(5)

# Set motors speed to speed2
controller.set_motor_speed(MotorIndex.Motor0, speed2)
controller.set_motor_speed(MotorIndex.Motor1, speed2)
controller.set_motor_speed(MotorIndex.Motor2, speed2) 
controller.set_motor_speed(MotorIndex.Motor3, speed2)
time.sleep(3)

# Stop all motors
controller.stop_motor(MotorIndex.Motor0)
controller.stop_motor(MotorIndex.Motor1)
controller.stop_motor(MotorIndex.Motor2)
controller.stop_motor(MotorIndex.Motor3)
time.sleep(3)

# Set motors speed to speed1 in reverse
controller.set_motor_speed(MotorIndex.Motor0, -speed1)
controller.set_motor_speed(MotorIndex.Motor1, -speed1)
controller.set_motor_speed(MotorIndex.Motor2, -speed1)
controller.set_motor_speed(MotorIndex.Motor3, -speed1)
time.sleep(3)

# Stop all motors
controller.stop_motor(MotorIndex.Motor0)
controller.stop_motor(MotorIndex.Motor1)
controller.stop_motor(MotorIndex.Motor2)
controller.stop_motor(MotorIndex.Motor3)