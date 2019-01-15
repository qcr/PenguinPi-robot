#!usr/bin/python3

import penguinPi as ppi
import time
import cv2

ppi.init()
#ppi.clear_data()

mL = ppi.Motor('AD_MOTOR_L')
mR = ppi.Motor('AD_MOTOR_R')

el = mL.get_encoder()
er = mR.get_encoder()
print( 'encoders',  el, er )

time.sleep(5)

el, er = ppi.motor_setget(20, -20)
print( 'encoders',  el, er )

time.sleep(5)

el, er = ppi.motor_setget(0, 0)
print( 'encoders',  el, er )

el = mL.get_encoder()
er = mR.get_encoder()
print( 'encoders',  el, er )


