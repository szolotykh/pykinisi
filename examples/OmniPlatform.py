# Filename: Platform.py
# Description: Example of platform control

import time
from pykinisi import *
from Core import *

controller = InitTest()

result = controller.toggle_status_led_state()
time.sleep(0.5) # 1s
result = controller.toggle_status_led_state()

# Initialize platform
controller.initialize_omni_platform(
    is_reversed_0=False,
    is_reversed_1=False,
    is_reversed_2=False,
    wheels_diameter=1,
    robot_radius=1,
    encoder_resolution=0
)

# Set three platform velocity components
controller.set_platform_velocity(40, 0, 0)
time.sleep(5) # 5s
controller.set_platform_velocity(0, 40, 0)
time.sleep(5) # 5s
controller.set_platform_velocity(0, 0, 0)