# Filename: GPIOWrite.py
# Description: Example of GPIO write. Script will write to GPIO1 every second.

import time
from Core import *

# Initialize connection to the controller
controller = InitTest()

pin = GPIOIndex.GPIO1
controller.initialize_gpio_pin(pin, GPIOMode.OUTPUT)

while(True):
    controller.set_gpio_pin_state(pin, State.HIGH)
    time.sleep(1) # 1000ms
    controller.set_gpio_pin_state(pin, State.LOW)
    time.sleep(1) # 1000ms