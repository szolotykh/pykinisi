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

# Status LED test
while(True):
    controller.GPIOToggleStatusLED()
    time.sleep(0.1)