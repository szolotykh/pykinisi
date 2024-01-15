# Filename: EncoderRead.py
# Description: Example of reading encoder value. Script will print encoder value every second.

import time
from pykinisi import *
from Core import *

controller = InitTest()

# Encoder test
encoder = EncoderIndex.Encoder0
controller.initialize_encoder(encoder)
while(True):
    value = controller.get_encoder_value(encoder)
    print(value)
    time.sleep(1)