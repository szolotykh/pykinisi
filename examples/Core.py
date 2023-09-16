import sys
from pykinisi import *

def InitTest():
    if(len(sys.argv) < 2):
        print(F"Incorrect number of arguments. {sys.argv[0]} <COM Port.>")

    port = sys.argv[1]
    controller = KinisiController()
    if not controller.connect(port):
        print(f"Can't open serial connection. Port: {port}")
        exit()

    return controller