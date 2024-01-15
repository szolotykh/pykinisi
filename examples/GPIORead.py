# Filename: GPIORead.py
# Description: Example of GPIO read. Script will read GPIO pin state every second.

import time
from Core import *

# Initialize connection to the controller
controller = InitTest()

pin = GPIOIndex.GPIO0
controller.initialize_gpio_pin(pin, GPIOMode.INPUT_PULLUP)

while(True):
    value = controller.get_gpio_pin_state(pin)
    print(value)
    time.sleep(1) # 1000ms