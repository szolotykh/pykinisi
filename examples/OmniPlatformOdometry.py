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
    wheels_diameter=0.1, # 10 cm
    robot_radius=0.15, # 15 cm
    # Must not be zero to enable odometry
    encoder_resolution=1992.6 # 1992.6 ticks per revolution
)

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