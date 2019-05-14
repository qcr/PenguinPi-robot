#!usr/bin/python3

import penguinPi as ppi
import time
import cv2

mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)

print("STARTING")
ppi.init()
mA.set_power(0)
mB.set_power(0)

start = mB.get_ticks()

# while 

sp = -40
try:
    # for i in range(0,10):
    mA.set_power(sp)
    mB.set_power(sp)
    # count = 0
    # print(mA.get_ticks(), mB.get_ticks())
    # count += 1
    time.sleep(1)
    # sp += 1

except:
    mA.set_power(0)
    mB.set_power(0)

mA.set_power(0)
mB.set_power(0)


print(mB.get_ticks()- start)
