#!usr/bin/python3

import time
import socket
import sys
import io
import argparse

import picamera
#from BrickPi import *

"""
" Constant variables
"""
IP_ADDRESS = '0.0.0.0'
PORT = 43901
CHUNK_SIZE = 128

IM_WIDTH = 320
IM_HEIGHT = 240

FN_GET_IMAGE = 'getImageFromCamera'

debug = False;

"""
" Helper functions
"""
def executeRequestedFunction(requestData, connection):
	# Splice and dice
	data = requestData.decode("utf-8").split(',')
	if debug:
		print(data[1])
	fn = data[0]
	args = data[1:]

	# Decide what function should be run, and attempt to run it with the arguments
	if fn == FN_GET_IMAGE:
		if debug:
			print('getImageFromCamera() called received!', file=sys.stderr)

		# Ensure everything is in a safe state
		stream.seek(0) # go to start of stream
		stream.truncate() # Clears the stream and makes it safe

		# Capture the image
		camera.capture(stream, 'rgb', True, (320,240) )

		# Send the image over the connection
		stream.seek(0)
		connection.sendall(stream.read())


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

print(args)
print(args.awb)

# Create a TCP/IP socket and bind the socket to the port
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = (IP_ADDRESS, PORT)
print('Starting up on {0}, port {1}' .format(server_address[0], server_address[1]), file=sys.stderr) #>>sys.stderr, 'Starting up on %s, port %s' % server_address
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(server_address)

# Get the camera up and running
stream = io.BytesIO()

# see http://picamera.readthedocs.io/en/release-1.10/api_camera.html for details

camera = picamera.PiCamera()
camera.resolution = (IM_WIDTH, IM_HEIGHT)
#camera.start_preview()

if args.awb:
	camera.awb_mode = args.awb
if args.gain:
	camera.awb_gains = tuple(float(x) for x in args.gain.split(','))
print('white balance mode is ', camera.awb_mode)
time.sleep(2)

# Initialise loop variables
connection = None
client_address = None

# Start listening for incoming commands
sock.setblocking(0)
sock.listen(1)
while True:
	# Check if any new commands have been received
	try:
		connection, client_address = sock.accept()
		recv_str = connection.recv(CHUNK_SIZE)
		data = recv_str
		while not recv_str: # MAJOR BUG, if data size over than 128 enter infinite looping
		# (should be 'while recv_str')
			recv_str = connection.recv(CHUNK_SIZE)
			data += recv_str
		# TODO should check if all data was received... but too lazy

		# Send the data to the function processor
		executeRequestedFunction(data.rstrip(), connection)
	except socket.error as msg:
		pass
	finally:
		# Clean up the connection if it exists
		if connection is not None:
			connection.close()
