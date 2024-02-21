# Filename: PlatformPWM.py
# Description: Example of platform control

import time
from pykinisi import *
from Core import *

controller = InitTest()

result = controller.toggle_status_led_state()
time.sleep(0.5) # 1s
result = controller.toggle_status_led_state()

# Initialize platform
platform_type = "omni"

if platform_type == "omni":
    controller.initialize_omni_platform(
        is_reversed_0=False,
        is_reversed_1=False,
        is_reversed_2=False,
        wheels_diameter=1,
        robot_radius=1,
        encoder_resolution=0
    )
elif platform_type == "mecanum":
    controller.initialize_mecanum_platform(
        is_reversed_0=False,
        is_reversed_1=False,
        is_reversed_2=False,
        is_reversed_3=False,
        length=0.5, # 50 cm 
        width=0.4, # 40 cm
        wheels_diameter=0.1, # 10 cm
        encoder_resolution=0
    )
else:
    print("Unknown platform type")
    exit()

# Set three platform velocity components
controller.set_platform_velocity(40, 0, 0)
time.sleep(5) # 5s
controller.set_platform_velocity(0, 40, 0)
time.sleep(5) # 5s
controller.set_platform_velocity(0, 0, 0)