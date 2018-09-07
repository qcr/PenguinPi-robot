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
display = ppi.Display(ppi.AD_DISPLAY_A)
display.set_mode('u');

#initialise serial, and retrieve initial values from the Atmega
ppi.init()
mA.get_all()
mB.get_all()

def setspeed(speed):
	display.set_value(abs(speed));
	mA.set_power(speed)
	mB.set_power(speed)

for speed in range(9,100,10):
	setspeed(speed)
	time.sleep(1)
setspeed(0)
time.sleep(1)

for speed in range(9,100,10):
	setspeed(-speed)
	time.sleep(1)
setspeed(0)
