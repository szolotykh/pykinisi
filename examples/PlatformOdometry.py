# Filename: PlatformOdometry.py
# Description: Example of platform odometry. Script will capture odometry data for 10 seconds.

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
        wheels_diameter=0.1, # 10 cm
        robot_radius=0.15, # 15 cm
        encoder_resolution=0
    )
elif platform_type == "mecanum":
    controller.initialize_mecanum_platform(
        is_reversed_0=False,
        is_reversed_1=False,
        is_reversed_2=False,
        is_reversed_3=False,
        length= 0.5, # 50 cm
        width= 0.4, # 40 cm
        wheels_diameter=0.1, # 10 cm
        encoder_resolution=0
    )
else:
    print("Unknown platform type")
    exit()

time.sleep(1) # 1s

# Start odometry
controller.start_platform_odometry()

# Set three platform velocity components
controller.set_platform_velocity(40, 0, 0)

# Capture odometry data for 10 seconds
start_time = time.time()
while time.time() - start_time < 10:
    odometry_data = controller.get_platform_odometry()
    print(f"X: {round(odometry_data.x, 2)} m, Y: {round(odometry_data.y, 2)} m, Theta: {round(odometry_data.t, 2)} rad")
    time.sleep(0.5)

# Stop odometry
controller.stop_platform_odometry()

# Stop platform
controller.set_platform_velocity(0, 0, 0)