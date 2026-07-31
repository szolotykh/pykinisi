# Filename: LoopFrequency.py
# Description: Example of reading and setting the global controller and odometry
#              loop frequencies. Valid range is 1 to 1000 Hz; values outside the
#              range are clamped and 0 is ignored.

from Core import *

controller = InitTest()

# Read the current loop frequencies.
print(f"Controller frequency: {controller.get_controller_frequency()} Hz")
print(f"Odometry frequency: {controller.get_odometry_frequency()} Hz")

# Update the loop frequencies (Hz, 1-1000).
controller.set_controller_frequency(100)
controller.set_odometry_frequency(50)

# Read back to confirm.
print(f"New controller frequency: {controller.get_controller_frequency()} Hz")
print(f"New odometry frequency: {controller.get_odometry_frequency()} Hz")
