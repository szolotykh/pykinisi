import time
from Controller import *
from Core import *

import math

def sign(x):
    if(x == 0): return 0
    return x/abs(x)

def normalize(x, y, t):
    #l = math.sqrt(x*x+y*y+t*t)
    l = abs(x)+abs(y)+abs(t)
    return [sign(x)*x*x/l, sign(y)*y*y/l, sign(t)*t*t/l]


r = normalize(-100, 0, 0)
print(f"Result: {r[0]}, {r[1]}, {r[2]}")



controller = InitTest()

time.sleep(1)
result = controller.GPIOToggleStatusLED()
print(f"Result: {result}")
time.sleep(0.5)
result = controller.GPIOToggleStatusLED()
print(f"Result: {result}")

controller.PlatformInitialize()
time.sleep(1)
controller.PlatformSetVelocityInput(0, 0, 100)
time.sleep(5)
controller.PlatformSetVelocityInput(0, 0, 0)

print("Done.")