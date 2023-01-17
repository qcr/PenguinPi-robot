#!/usr/bin/env python

import os
import sys
import time

sys.path.insert(1, os.path.join(sys.path[0], '..'))
import penguinPi as ppi

ppi.init()
#ppi.clear_data()

mL = ppi.Motor('AD_MOTOR_L')
mR = ppi.Motor('AD_MOTOR_R')

el = mL.get_encoder()
er = mR.get_encoder()
print( 'encoders',  el, er )
