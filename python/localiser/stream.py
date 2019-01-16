#!usr/bin/python3
import time
import picamera
#import sys

# Constants
# IM_WIDTH = 3280
# IM_HEIGHT =2464


IM_WIDTH = 640 
IM_HEIGHT = 480
camera = picamera.PiCamera()
# camera.rotation = 90
camera.resolution = (IM_WIDTH, IM_HEIGHT)
folder = input("Folder: ")
#name = input("Direction: ")

time.sleep(2)

for i in range(50):
    #for x in range(4):
    input("hit enter")
    camera.capture(folder + str(i) + '.jpg')
    
