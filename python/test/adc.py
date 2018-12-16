#!usr/bin/env
'''
PenguinPi.py test script
'''

import penguinPi as ppi
import time

adcVoltage = ppi.AnalogIn('AD_ADC_V')
adcCurrent = ppi.AnalogIn('AD_ADC_C')


ppi.init()

#read ADC values and display
for i in range(0,10):
    print ( "Voltage: %5.2f mV, Current: %5.2f  mA" % ( adcVoltage.get_smooth(), adcCurrent.get_smooth() ))
    time.sleep(2)

ppi.close()
