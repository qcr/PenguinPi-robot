#!usr/bin/env
'''
PenguinPi.py test script
'''

import penguinPi as ppi
import time

#Create our device objects
mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)

adcVoltage = ppi.AnalogIn(ppi.AD_ADC_V)
adcCurrent = ppi.AnalogIn(ppi.AD_ADC_C)

display = ppi.Display(ppi.AD_DISPLAY_A)

#initialise serial, and retrieve initial values from the Atmega
ppi.init()
mA.get_all()
mB.get_all()
adcVoltage.get_all()
adcCurrent.get_all()
display.get_all()

#read ADC values and display
adcVoltage.get_value()
adcCurrent.get_value()
print ( "------------------------------------------" )
print ( "Press Enter to stop..." )
print ( "Voltage: %5.2f  V, Current: %5.2f  mA" % (adcVoltage.value, adcCurrent.value) )
# print("Voltage: {:5.2f} V".format(adcVoltage.value))
#alternatively:
print ( "Raw Vol: %5d, Raw Cur: %5d" % (adcVoltage.get_raw(), adcCurrent.get_raw()) )
# print("Raw Voltage: {:10} divs".format(adcVoltage.get_raw()))

#draw voltage to the screen
display.set_value(round(adcVoltage.value))
time.sleep(0.5)

mA.set_power(20)
mB.set_power(20)
time.sleep(5)
mA.set_power(0)
mB.set_power(0)

ppi.close()
