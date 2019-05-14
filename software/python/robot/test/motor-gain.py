#!/usr/bin/python3

import time
import penguinPi as ppi

"""
" Constant Variables
"""


"""
" Main exectution block
"""

mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)

#initialise serial, and retrieve initial values from the Atmega
ppi.init()
mA.get_all()
mB.get_all()

motor = mA;  # test motor A

T = 10

speeds = range(10,105,10)

for speed in speeds:
    motor.set_power(speed)
    time.sleep(1)
    e1 = motor.get_encoder()
    time.sleep(T)
    e2 = motor.get_encoder()
    print(speed, e1, e2, (e2-e1)/T)

motor.set_power(0)
