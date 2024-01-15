# Filename: Platform.py
# Description: Example of platform control

import time
from pykinisi import *
from examples.Core import *

controller = InitTest()

result = controller.toggle_status_led_state()
time.sleep(0.5) # 1s
result = controller.toggle_status_led_state()

# Initialize platform
controller.initialize_mecanum_platform(True, True, True, True)

# Set three platform velocity components
controller.set_platform_velocity_input(100, 0, 50)
time.sleep(5) # 5s
controller.set_platform_velocity_input(-100, 0, 0)
time.sleep(5) # 5s
controller.set_platform_velocity_input(0, 0, 0)