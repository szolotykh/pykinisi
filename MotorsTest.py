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

# Motor test
speed1 = 100
speed2 = 60
controller.InitializeAllMotors()

controller.SetMotorSpeed(Motor0, True, speed1)
controller.SetMotorSpeed(Motor1, True, speed1)
controller.SetMotorSpeed(Motor2, True, speed1)
controller.SetMotorSpeed(Motor3, True, speed1)
time.sleep(10)

controller.SetMotorSpeed(Motor0, True, speed2)
controller.SetMotorSpeed(Motor1, True, speed2)
controller.SetMotorSpeed(Motor2, True, speed2)
controller.SetMotorSpeed(Motor3, True, speed2)
time.sleep(5)

controller.StopAllMotors()
time.sleep(3)

controller.SetMotorSpeed(Motor0, False, speed1)
controller.SetMotorSpeed(Motor1, False, speed1)
controller.SetMotorSpeed(Motor2, False, speed1)
controller.SetMotorSpeed(Motor3, False, speed1)
time.sleep(5)

controller.StopAllMotors()