#!usr/bin/python3
'''
A script to detect the soft shutdown button, and then initiate a shutdown
Can also detect a low battery warning from the atmega
'''
import wiringpi as wp
import os
import time
import serial

ser = serial.Serial(port = '/dev/serial0',
					baudrate = 115200,
					parity = serial.PARITY_NONE,
					stopbits = serial.STOPBITS_ONE,
					bytesize = serial.EIGHTBITS,
					timeout = 1)

def init():
	#BCM numbering
	wp.wiringPiSetupGpio()
	wp.pinMode(22, 0)#input
	wp.pullUpDnControl(22, wp.PUD_UP)

	wp.pinMode(11, 0)
	wp.pullUpDnControl(11, wp.PUD_UP)

	if ser.isOpen() == False:
		ser.open()


def checkBUTTON():
	if wp.digitalRead(22) == 0:
		#wait for held down
		time.sleep(0.5)
		if wp.digitalRead(22) == 0:
			print("External Button Pressed, going down for shutdown!!")
			ser.write(b'\x11\x04\xff\xff\xd3')#all stop
			# ser.write(b'\x11\x05\x0c\x01\x01\x03')#Red LED
			os.system("sudo halt")			


def checkATMEGA():
	if wp.digitalRead(11) == 0:
		time.sleep(10) #long timer to prevent programming from falsely trigerring a shutdown
		if wp.digitalRead(11) == 0:
			print("Shutdown Request Received, going down for shutdown!!")
			#ser.write(b'\x11\x05\x0c\x01\x01\x03')#Red LED
			os.system("sudo shutdown -h now")			


if __name__ == '__main__':
	init()
	while True:
		checkBUTTON()
		checkATMEGA()
		time.sleep(0.1)
