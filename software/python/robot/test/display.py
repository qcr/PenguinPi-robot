#!/usr/bin/env python

'''
PenguinPi.py test script
'''

import time
import traceback
import socket
import sys
import threading
import os
import argparse

sys.path.insert(1, os.path.join(sys.path[0], '..'))
import penguinPi as ppi

display = ppi.Display(ppi.AD_DISPLAY_A)

#initialise serial, and retrieve initial values from the Atmega
print("ppi.init()")
ppi.init()

#print("display.get_all()")
#display.get_all()

#display.set_mode('x')
#for i in range(0,255):
#    display.set_value(i)
#    time.sleep(0.1)
#
#display.set_mode('u')
#for i in range(0,99):
#    display.set_value(i)
#    time.sleep(0.1)
#
#display.set_mode('d')
#for i in range(10,-11,-1):
#    display.set_value(i)
#    time.sleep(0.1)
	
	
	
display.set_digit0(6)	
display.set_digit1(15)	
	
time.sleep(5)

display.set_value(255)	

time.sleep(5)

ppi.close()
