import wiringpi as wp
import os
import time

#BCM numbering

wp.wiringPiSetupGpio()
wp.pinMode(11, 1)
while True:
    wp.digitalWrite(11, 1)
    print(0)
    time.sleep(1)

    wp.digitalWrite(11, 0)
    print(1)
    time.sleep(1)
