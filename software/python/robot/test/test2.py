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


time.sleep(5)
motor.set_power(20)
time.sleep(2)
motor.set_power(0)
time.sleep(1)
