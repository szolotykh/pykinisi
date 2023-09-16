# Filename: Platform.py
# Description: Example of platform control

import time
from pykinisi import *
from examples.Core import *

controller = InitTest()

result = controller.toggle_status_led()
print(f"Result: {result}")
time.sleep(1) # 1s
result = controller.toggle_status_led()
print(f"Result: {result}")

# Initialize platform
controller.initialize_platform()

# Set three platform velocity components
controller.set_platform_velocity_input(0, 0, 50)
time.sleep(5) # 5s
controller.set_platform_velocity_input(0, 0, 0)