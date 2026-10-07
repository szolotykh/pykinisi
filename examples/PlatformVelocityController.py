# Filename: PlatformPWM.py
# Description: Example of platform control

import time
from pykinisi import *
from Core import *

controller = InitTest()
info = controller.board_info
if (info.protocol_major, info.protocol_minor, info.protocol_patch) < (2, 3, 1):
    controller.disconnect()
    raise RuntimeError("These starting gains require firmware 2.3.1 or newer; retune for your motor.")

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
        is_encoder_reversed_0=False,
        is_encoder_reversed_1=False,
        is_encoder_reversed_2=False,
        wheels_diameter=0.1, # 10 cm
        robot_radius=0.15, # 15 cm
        encoder_resolution=1425.1, # ticks per revolution
    )
elif platform_type == "mecanum":
    controller.initialize_mecanum_platform(
        is_reversed_0=False,
        is_reversed_1=False,
        is_reversed_2=False,
        is_reversed_3=False,
        is_encoder_reversed_0=False,
        is_encoder_reversed_1=False,
        is_encoder_reversed_2=False,
        is_encoder_reversed_3=False,
        length= 0.5, # 50 cm 
        width= 0.4, # 40 cm
        wheels_diameter=0.1, # 10 cm
        encoder_resolution=1425.1, # ticks per revolution
    )
else:
    print("Unknown platform type")
    exit()

# Set three platform velocity components
controller.start_platform_controller(
    kp=1, # Proportional gain
    ki=1, # Integral gain
    kd=0, # Derivative gain
    integral_limit=100 # Integral contribution limit in PWM percentage points.
)

print("Set platform target velocity to 0.4 m/s in x direction")
controller.set_platform_target_velocity(0.4, 0, 0)
time.sleep(5) # 5s

print("Stoppping platform.")
controller.set_platform_target_velocity(0, 0, 0)
time.sleep(3) # 3s

# Stop platform
controller.stop_platform_controller()
print("Stop platform")
