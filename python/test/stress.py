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

while True:
    mA.set_power(10)
    mB.set_power(-10)
    x = mA.get_encoder()
    x = mB.get_encoder()
    time.sleep(0.05)
