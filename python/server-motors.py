#!usr/bin/python3

import time
import traceback
import socket
import sys
import threading
import os
import argparse

import penguinPi as ppi

"""
" Constant Variables
"""
IP_ADDRESS = '0.0.0.0'
PORT = 43900
CHUNK_SIZE = 128

FN_MOTOR_SPEEDS = 'setMotorSpeeds'
FN_MOTOR_SPEEDS_PROFILE = 'setMotorSpeedsProfile'
FN_MOTOR_TICKS = 'getMotorTicks'
FN_MOTOR_ENCODERS = 'getMotorEncoders'
FN_DISPLAY_VALUE = 'setDisplayValue'
FN_DISPLAY_MODE = 'setDisplayMode'
FN_ALL_STOP = 'stopAll'


"""
" Helper functions
"""
def setspeed(speed, fraction):
    global mA, mB

    mA.set_power(int(speed[0]*fraction))
    mB.set_power(int(speed[1]*fraction))

def stop_all():
    global mA, mB

    mA.set_power(0)
    mB.set_power(0)

    

def xfrange(start, stop, step):
    i = 0
    while start + i * step < stop:
        yield start + i * step
        i += 1

def executeRequestedFunction(requestData, connection):
	if args.debug:
		print('Function request: "{0}"' .format(requestData), file=sys.stderr)

	# Splice and dice
	data = requestData.decode("utf-8").split(',')
	fn = data[0]

	if args.debug:
		print('data contains ')
		print(data)

	# Decide what function should be run, and attempt to run it with the arguments
	if fn == FN_MOTOR_SPEEDS:
		# set motor speed
		if args.debug:
			print("set motor speed")
		motorA = int(data[1])
		motorB = int(data[2])

		mA.set_power(motorA)
		mB.set_power(motorB)

	elif fn == FN_MOTOR_SPEEDS_PROFILE:
		# move motors by specified amount
		if args.debug:
			print("set motor speed with profile:", data)

		# there are four arguments: speedA, speedB, duration, acceltime
		speed = [ int(x) for x in data[1:3] ]
		Ttotal = float(data[3])
		Taccel = float(data[4])

		dt = 0.05;
		if Taccel > 0:
			for t in xfrange(0, Taccel, dt):
				setspeed(speed, t/Taccel);
				time.sleep(dt)

		for t in xfrange(0, Ttotal-Taccel, dt):
			setspeed(speed, 1.0)
			time.sleep(dt)

		if Taccel > 0:
			for t in xfrange(0, Taccel, dt):
				setspeed(speed, (Taccel-t)/Taccel);
				time.sleep(dt)

		# all stop
		mA.set_power(0)
		mB.set_power(0)

		# signal that we're done here
		s = 'done\n'
		b = s.encode('utf-8')

		# send completion back over 'connection
		connection.sendall(b)

	elif fn == FN_MOTOR_TICKS:
		if args.debug:
			print("read ticks")
		mAticks = mA.get_ticks()
		mBticks = mB.get_ticks()
		s = str(mAticks) + ' ' + str(mBticks) + ' ' + '\n'

		if args.debug:
			print(s)

		b = s.encode('utf-8')

		# send ticks back over 'connection
		connection.sendall(b)

	elif fn == FN_MOTOR_ENCODERS:
		if args.debug:
			print("read encoders")
		mAenc = mA.get_encoder()
		mBenc = mB.get_encoder()
		s = str(mAenc) + ' ' + str(mBenc) + ' ' + '\n'

		if args.debug:
			print(s)

		b = s.encode('utf-8')

		# send ticks back over 'connection
		connection.sendall(b)

	# update the display
	elif fn == FN_DISPLAY_VALUE:
		#print('display value = ' + data[1]);
		display.set_value(int(data[1]));

	elif fn == FN_DISPLAY_MODE:
		#print('display mode = ' + data[1]);
		display.set_mode(data[1][0]);

    # all stop
	elif fn == FN_ALL_STOP:
		print('clear data');
		ppi.clear_data();

"""
" Heartbeat thread, pulse the green LED periodically
"""
def HeartBeat():

    led = ppi.LED(ppi.AD_LED_G)

    while True:
        led.set_state(1);
        led.set_count(1000);
        time.sleep(2);

"""
" Main execution block
"""
import argparse
# handle command line arguments
parser = argparse.ArgumentParser()
parser.add_argument("-d", "--debug", help="show debug information", 
	action="store_true")
args = parser.parse_args()

# Create a TCP/IP socket and bind the socket to the port
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = (IP_ADDRESS, PORT)
print('Starting up on {0}, port {1}' .format(server_address[0], server_address[1]), file=sys.stderr)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(server_address)

mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)
display = ppi.Display(ppi.AD_DISPLAY_A)

#initialise serial, and retrieve initial values from the Atmega
ppi.init()
mA.get_all()
mB.get_all()

# Initialise loop variables
connection = None
client_address = None

# initialize the heartbeat thread
heartbeat_thread = threading.Thread(target=HeartBeat, daemon=True)
heartbeat_thread.start()

# attempt to get the low byte of the WLAN IP address
try:
    f = os.popen('sudo ifconfig wlan0 | grep "inet\ addr" | cut -d: -f2 | cut -d" " -f1')
    ip = f.read()
    ip = bytes(map(int, ip.split('.')))
    display.set_mode('x')
    display.set_value(ip[3]);
except ValueError:
    display.set_mode('d')
    display.set_value(-1);

## handle incoming commands

#sock.setblocking(0)
sock.listen(1)

while True:
	connection, client_address = sock.accept()
	recv_str = connection.recv(CHUNK_SIZE)
	data = recv_str
	while not recv_str:
		recv_str = connection.recv(CHUNK_SIZE)
		data += recv_str
	# TODO should check if all data was received.... but too lazy

	# Send the data to the function processor
	executeRequestedFunction(data.rstrip(), connection)
