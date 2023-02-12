import time
from Controller import *
from Core import *

controller = InitTest()

mIndex = 0
controller.GPIOToggleStatusLED()
time.sleep(1)
controller.GPIOToggleStatusLED()
time.sleep(1)

controller.SetMotorController(mIndex, 0.15, 0, 0)
time.sleep(0.2)

print("40")
controller.SetMotorTargetVelocity(mIndex, 0, 30)
time.sleep(10)

print("Del")
controller.DelMotorController(mIndex)