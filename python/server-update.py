#!/usr/bin/env python3

# note do not run as sudo
import stat
import os
import sys
import time

disk = '/dev/disk/by-label/PPI-UPDATE'
mnt = '/media/usb'
dest = '/home/pi/EGB439'

while True:
	time.sleep(2)

	try:
		if not stat.S_ISBLK(os.stat(disk).st_mode):
			continue;
	except:
		continue;

	print('disk present')

	os.system('sudo mount {} {}'.format(disk, mnt) );
	os.system('cp -r {}/EGB439/. {}'.format(mnt,dest))	
	os.system('sudo shutdown -h now')
