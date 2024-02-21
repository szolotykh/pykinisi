# Filename: MotorVelocityController.py
# Description: Example of motor velocity control.

import time
from Core import *

controller = InitTest()

speed = 1.5 # rad/s

motorIndex = MotorIndex.Motor0

# Initialize motor controller
controller.initialize_motor_controller(
    motor_index = motorIndex, # Motor index
    is_reversed = False, # Motor and encoder direction
    encoder_index = EncoderIndex.Encoder0, # Encoder index
    encoder_resolution = 1992.6, # ticks per revolution
    kp = 1, # Proportional gain
    ki = 0.1, # Integral gain
    kd = 0, # Derivative gain
    integral_limit = 30 # Absolute value of integral value.
)
# Output of the controller is motor speed in PWM from -100 to 100. Integral limit value should be in this range.

# Set motor target speed
# It is not permitted to use set_motor_speed command after controller initialization,
# but it may lead to unexpected behavior. It is advised to use set_motor_target_speed instead.
print(f"Set motor target speed to {speed} rad/s")
controller.set_motor_target_speed(motorIndex, speed)
time.sleep(10)

# Stopping motor
print("Stopping motor")
controller.set_motor_target_speed(motorIndex, 0)
time.sleep(3)

# Stop motor
# After motor controller is deleted the motor will stop.
print("Stop motor")
controller.delete_motor_controller(motorIndex)