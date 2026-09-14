"""Shared connection setup for the serial-port examples."""

import sys
from pykinisi import *

def InitTest():
    """Connect to the command-line port and wait for board identity and READY."""
    if(len(sys.argv) < 2):
        print(F"Incorrect number of arguments. {sys.argv[0]} <COM Port.>")
        exit()

    port = sys.argv[1]
    controller = KinisiController()
    if not controller.connect(port):
        print(f"Can't connect to {port}: {controller.last_error}")
        raise SystemExit(1)

    return controller
