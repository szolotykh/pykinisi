# Position control requires firmware 2.3 or newer; these velocity gains require 2.3.1.
# Tune gains, encoder direction and geometry for your hardware before running.
import time
from Core import InitTest

controller = InitTest()
try:
    info = controller.board_info
    if (info.protocol_major, info.protocol_minor, info.protocol_patch) < (2, 3, 1):
        raise RuntimeError("This example requires firmware 2.3.1 or newer")
    controller.initialize_motor_controller(0, False, 0, False, 1425.1, 1, 1, 0, integral_limit=100)
    controller.initialize_motor_position_pid_controller(
        motor_index=0, kp=2, max_speed=1, tolerance=0.02, ki=0, kd=0, integral_limit=1)
    controller.reset_motor_position(0)
    controller.set_motor_position(0, 1.0)  # Continuous radians from the reset origin.
    for _ in range(20):
        time.sleep(0.25)
        print("Position (rad):", controller.get_motor_position(0))
finally:
    try:
        controller.delete_motor_controller(0)
    finally:
        controller.disconnect()
