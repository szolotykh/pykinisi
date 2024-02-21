# Filename: EncoderRead.py
# Description: Example of reading encoder value. Script will print encoder value every second.

import time
from Core import *

controller = InitTest()

# Encoder test
encoder = EncoderIndex.Encoder0
controller.initialize_encoder(encoder, 0, True)
while(True):
    value = controller.get_encoder_value(encoder)
    print(f"Encoder value: {value}")
    time.sleep(1)