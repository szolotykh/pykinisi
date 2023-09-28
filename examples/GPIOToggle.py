# Filename: GPIOToggle.py
# Description: Example of GPIO toggle. Script will toggle GPIO1 every second.

import time
from Core import *

# Initialize connection to the controller
controller = InitTest()

pin = GPIOIndex.GPIO1
controller.initialize_gpio_pin(pin, GPIOMode.OUTPUT)

while(True):
    controller.toggle_gpio_pin_state(pin)
    time.sleep(1) # 1000ms