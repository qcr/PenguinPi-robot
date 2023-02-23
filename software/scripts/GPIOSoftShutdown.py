#!/usr/bin/env python

'''
A script to detect the soft shutdown button, and then initiate a shutdown
Can also detect a low battery warning from the atmega
'''
import wiringpi as wp
import os
import time

def init():
    #BCM numbering
    wp.wiringPiSetupGpio()
    wp.pinMode(22, 0)#input
    wp.pullUpDnControl(22, wp.PUD_UP)

    wp.pinMode(10, 0)
    wp.pullUpDnControl(10, wp.PUD_UP)

    wp.pinMode(11, 1)
    wp.digitalWrite(11, 1)

def shutdown():
    # tell the Atmel to display shutdown message
    wp.digitalWrite(11, 0)
    time.sleep(0.5)

    # shutdown the Pi, it will reboot...
    os.system("sudo halt")

def checkBUTTON():
    if wp.digitalRead(22) == 0:
        #wait for held down
        time.sleep(0.5)
        if wp.digitalRead(22) == 0:
            print("SDN button pressed (GPIO22) -- shutting down")
            shutdown()

def checkATMEGA():
    if wp.digitalRead(10) == 0:
        # long timer to prevent programming from falsely trigerring a
        # shutdown
        time.sleep(15)
        if wp.digitalRead(10) == 0:
            print("Atmel request (GPIO10) -- shutting down")
            shutdown()

if __name__ == '__main__':
    init()
    print('Init done')
    while True:
        checkBUTTON()
        checkATMEGA()
        time.sleep(0.2)
