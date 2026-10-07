# Position control requires firmware 2.3 or newer; these velocity gains require 2.3.1.
# Tune gains, encoder direction and geometry for your hardware before running.
import time
from Core import InitTest

controller = InitTest()
try:
    info = controller.board_info
    if (info.protocol_major, info.protocol_minor, info.protocol_patch) < (2, 3, 1):
        raise RuntimeError("This example requires firmware 2.3.1 or newer")
    controller.initialize_omni_platform(False, False, False, False, False, False, 0.1, 0.15, 1425.1)
    controller.start_platform_controller(1, 1, 0, integral_limit=100)
    controller.initialize_platform_position_pid_controller(
        linear_kp=1, angular_kp=2, max_linear_speed=0.2, max_angular_speed=0.5,
        position_tolerance=0.01, heading_tolerance=0.03,
        linear_ki=0, linear_kd=0, linear_integral_limit=0.2,
        angular_ki=0, angular_kd=0, angular_integral_limit=0.5)
    controller.reset_platform_position()
    # Wait for a fresh odometry sample after changing the origin.
    time.sleep(0.1)
    controller.get_platform_odometry()
    controller.set_platform_position(0.1, 0, 0.2)  # World-frame x/y in meters, heading in radians.
    for _ in range(20):
        time.sleep(0.25)
        pose = controller.get_platform_odometry()
        print(pose.x, pose.y, pose.t)
finally:
    try:
        controller.stop_platform_controller()
    finally:
        controller.disconnect()
