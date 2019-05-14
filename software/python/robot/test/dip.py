#!usr/bin/python3

import penguinPi as ppi
import time
import cv2

ppi.init()

print( 'DIP switch is set to',  ppi.get_dip() )
