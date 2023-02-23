#!/usr/bin/python3

import time
import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
import penguinPi as ppi

"""
" Constant Variables
"""


"""
" Main exectution block
"""

mR = ppi.Motor('AD_MOTOR_R')
mL = ppi.Motor('AD_MOTOR_L')

#initialise serial, and retrieve initial values from the Atmega
ppi.init()
mR.get_all()
mL.get_all()

P, I, D = mR.get_PID()
print('P = ', P)
print('I = ', I)
print('D = ', D)
