from __future__ import print_function
from inputs import get_gamepad
import time
from Controller import *
import sys

speed = 50
controller = Controller()

def main():
    if(len(sys.argv) < 2):
        print(F"Incorrect number of arguments. {sys.argv[0]} <COM Port.>")

    port = sys.argv[1]
    if not controller.Connect(port):
        print(f"Can't open serial connection. Port: {port}")
        exit()

    controller.InitializeAllMotors()

    while 1:
        events = get_gamepad()
        for event in events:
            if(event.ev_type == "Sync"):
                continue
            
            if("ABS_HAT0Y" in event.code):
                print(event.ev_type, event.code, event.state)
                if(event.state == 0):
                    controller.StopAllMotors()
                    continue

                direction = event.state == -1
                controller.SetMotorSpeed(Motor0, direction, speed)
                controller.SetMotorSpeed(Motor1, direction, speed)
                controller.SetMotorSpeed(Motor2, not direction, speed)
                controller.SetMotorSpeed(Motor3, not direction, speed)

            if("ABS_HAT0X" in event.code):
                print(event.ev_type, event.code, event.state)
                if(event.state == 0):
                    controller.StopAllMotors()
                    continue

                direction = event.state == -1
                controller.SetMotorSpeed(Motor0, direction, speed)
                controller.SetMotorSpeed(Motor1, not direction, speed)
                controller.SetMotorSpeed(Motor2, not direction, speed)
                controller.SetMotorSpeed(Motor3, direction, speed)

            if("ABS_HAT0" in event.code):
                print(event.ev_type, event.code, event.state)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        controller.StopAllMotors()