import time
from Controller import *
import sys

if(len(sys.argv) < 2):
    print(F"Incorrect number of arguments. {sys.argv[0]} <COM Port.>")

port = sys.argv[1]
controller = Controller()
if not controller.Connect(port):
    print(f"Can't open serial connection. Port: {port}")
    exit()

# Encoder test
encoder = Encoder0
controller.InitializeEncoder(encoder)
while(True):
    value = controller.GetEncoderValue(encoder)
    print(int.from_bytes(value, "little"))
    time.sleep(0.5)