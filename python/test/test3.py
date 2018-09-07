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

motor = mA;  # test motor A

while True:
    e = motor.get_encoder();
    print('%04x' % e)
    time.sleep(1)
