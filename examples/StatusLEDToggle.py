# Filename: StatusLEDToggle.py
# Description: Example of status LED toggle. Script will toggle status LED every 100ms.

import time
from pykinisi import *
from Core import *

# Initialize connection to the controller
controller = InitTest()

while(True):
    controller.toggle_status_led()
    time.sleep(0.1) # 100ms