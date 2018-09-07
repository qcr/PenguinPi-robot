#!usr/bin/python3

import penguinPi as ppi
import time
import cv2

ppi.init()
#ppi.clear_data()

ea, eb = ppi.motor_setget(20, 20)
print( 'encoders',  ea, eb )

time.sleep(20)

ea2, eb2 = ppi.motor_setget(0, 0)
print( 'encoders',  ea2, eb2 )

print( 'change', ea2-ea, eb2-eb)

time.sleep(2)
