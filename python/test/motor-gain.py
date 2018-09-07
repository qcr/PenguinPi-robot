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

motor = mA;  # test motor A

for speed in range(9,100,10):
    motor.set_power(speed)
    time.sleep(1)
    e1 = motor.get_encoder()
    time.sleep(5)
    e2 = motor.get_encoder()
    print('demand, actual ', speed, (e2-e1)/5)

motor.set_power(0)
