#!usr/bin/python3

import time
import socket
import sys
import io
import argparse

import picamera

"""
" Main execution block
"""

# parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument("-a", "--auto", dest="awb", help="auto white balance", 
	action="store_const", const="auto")
parser.add_argument("-o", "--off", dest="awb", help="disable white balance", 
	action="store_const", const="off")
parser.add_argument("-t", "--tungsten", dest="awb", help="tungsten white balance", 
	action="store_const", const="tungsten")
parser.add_argument("-s", "--sun", dest="awb", help="sun white balance", 
	const="sunlight", action="store_const")
parser.add_argument("-g", "--gain", dest="gain", action="store", help="set white balance gain manually: rbgain OR rgain,bgain")
args = parser.parse_args()
if args.gain:
	args.awb = 'off'


# see http://picamera.readthedocs.io/en/release-1.10/api_camera.html for details

camera = picamera.PiCamera()
#camera.resolution = (IM_WIDTH, IM_HEIGHT)
camera.start_preview(alpha=80)

if args.awb:
	camera.awb_mode = args.awb
if args.gain:
	camera.awb_gains = tuple(float(x) for x in args.gain.split(','))
print('white balance mode is ', camera.awb_mode)
time.sleep(10)
