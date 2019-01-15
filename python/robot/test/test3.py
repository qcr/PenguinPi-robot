#!usr/bin/env python3
'''
PenguinPi.py test script
'''

import penguinPi as ppi
import time

ppi.init()

hat = ppi.Hat('AD_HAT');

b = hat.get_dip()
print('dip = %d' % b)

b = hat.get_ledarray()
print('led array = 0x%04x' % b)

for i in range(0,4):
    hat.set_screen(i)
    time.sleep(1)

hat.set_ledarray(0xffff)
time.sleep(0.5)
hat.set_ledarray(0)

hat.set_ledarray(0x1234)
b = hat.get_ledarray()
print('led array = 0x%04x' % b)
time.sleep(0.5)
hat.set_ledarray(0)


for i in range(0,16):
    hat.set_ledarray(1<<i)
    time.sleep(0.3)
hat.set_ledarray(0)

for i in range(0,5):
    print('button %d' % hat.get_button())
    time.sleep(2)

