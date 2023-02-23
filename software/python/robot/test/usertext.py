#!/usr/bin/env python

import os
import sys
import time

sys.path.insert(1, os.path.join(sys.path[0], '..'))
import penguinPi as ppi

#initialise serial, and retrieve initial values from the Atmega
ppi.init()

for i in range(8):
    s = 'line %d\n' % i
    ppi.uart.ser.write(s.encode('utf-8'))
    time.sleep(0.5)

ppi.uart.ser.write('\f'.encode('utf-8'))
time.sleep(1)
for i in range(8):
    s = 'line %d\n' % i
    ppi.uart.ser.write(s.encode('utf-8'))
    time.sleep(0.5)
