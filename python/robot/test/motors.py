#!usr/bin/env
'''
PenguinPi.py test script
'''

import penguinPi as ppi
import time

#Create our device objects
mR = ppi.Motor('AD_MOTOR_R')
mL = ppi.Motor('AD_MOTOR_L')

#initialise serial, and retrieve initial values from the Atmega
ppi.init()
#mR.get_all()
#mL.get_all()

print('right')
mR.set_speed(20)
time.sleep(5)
mR.set_speed(0)
time.sleep(1)
mR.set_speed(-20)
time.sleep(5)
mR.set_speed(0)

print('left')
mL.set_speed(20)
time.sleep(5)
mL.set_speed(0)
time.sleep(1)
mL.set_speed(-20)
time.sleep(5)
mL.set_speed(0)

ppi.close()
