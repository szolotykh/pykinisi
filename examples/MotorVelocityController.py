# Filename: MotorVelocityController.py
# Description: Example of motor velocity control.

import time
from Core import *

controller = InitTest()
info = controller.board_info
if (info.protocol_major, info.protocol_minor, info.protocol_patch) < (2, 3, 1):
    controller.disconnect()
    raise RuntimeError("These starting gains require firmware 2.3.1 or newer; retune for your motor.")

speed = 1.5 # rad/s

motorIndex = MotorIndex.Motor0

# Initialize motor controller
controller.initialize_motor_controller(
    motor_index = motorIndex, # Motor index
    is_reversed = False, # Motor direction
    encoder_index = EncoderIndex.Encoder0, # Encoder index
    is_encoder_reversed = False, # Encoder direction (independent of motor; flip if the controller runs away)
    encoder_resolution = 1425.1, # ticks per revolution
    kp = 1, # Proportional gain
    ki = 1, # Integral gain
    kd = 0, # Derivative gain
    integral_limit = 100 # Integral contribution limit in PWM percentage points.
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