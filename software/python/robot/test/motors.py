#!usr/bin/env
'''
PenguinPi.py test script
'''

import os
import sys
import time

sys.path.insert(1, os.path.join(sys.path[0], '..'))
import penguinPi as ppi

#Create our device objects
mR = ppi.Motor('AD_MOTOR_R')
mL = ppi.Motor('AD_MOTOR_L')

#initialise serial, and retrieve initial values from the Atmega
ppi.init()
#mR.get_all()
#mL.get_all()

print('right')
mR.set_velocity(20)
time.sleep(5)
mR.set_velocity(0)
time.sleep(1)
mR.set_velocity(-20)
time.sleep(5)
mR.set_velocity(0)

print('left')
mL.set_velocity(20)
time.sleep(5)
mL.set_velocity(0)
time.sleep(1)
mL.set_velocity(-20)
time.sleep(5)
mL.set_velocity(0)

ppi.close()
