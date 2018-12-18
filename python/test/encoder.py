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

#initialise serial, and retrieve initial values from the Atmega
ppi.init()
mA.get_all()
mB.get_all()

mAticks0 = mA.get_ticks()
mBticks0 = mB.get_ticks()

print('{:^13} | {:^13}'.format('absolute tick', 'relative tick') )
while True:
	mAticks = mA.get_ticks()
	mBticks = mB.get_ticks()
	print('{:6} {:6} | {:6} {:6}'.format(mAticks, mBticks, mAticks-mAticks0, mBticks-mBticks0) )
	time.sleep(0.25)

if 0:
	display.set_mode('d');
	for speed in range(9,100,10):
		display.setValue(speed);
		mA.set_power(speed)
		mB.set_power(speed)
	mA.set_power(0)
	mB.set_power(0)

	for speed in range(9,100,10):
		display.setValue(speed);
		mA.set_power(-speed)
		mB.set_power(-speed)


