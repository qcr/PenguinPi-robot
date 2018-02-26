#!usr/bin/python3

import time
import socket
import sys
import io
import subprocess

"""
" Constant variables
"""
IP_ADDRESS = '0.0.0.0'
PORT = 43902
FN_GET_TAGS = 'getTags'
CHUNK_SIZE = 128
debug = False;
#debug = True;

"""
" Helper functions
"""
def executeRequestedFunction(requestData, connection):
    # Splice and dice
    data = requestData.decode("utf-8").split(',')
    if debug:
        print("from MatLab: " + data[0])
    fn = data[0]
    args = data[1:]

    # Decide what function should be run, and attempt to run it with the arguments
    if fn == FN_GET_TAGS:
        if debug:
            print('getImageFromCamera() called received!', file=sys.stderr)

        p = subprocess.Popen("./../apriltag/example/apriltag_demo", stdout=subprocess.PIPE)

        for line in iter(p.stdout.readline, "\n"):
            #print(line)
            if line.decode('utf-8') == "\n": 
                noData = "-1 -1 \n"
                b = noData.encode('utf-8')
                connection.sendall(b)
                break
            else:
                connection.sendall(line)
                break

#            print(line)
#            print(line.split(';'))
#            for tag in line.split(';'):
#                if tag == "\n":
#                    break
#                tags = [float(z.strip()) for z in tag.split(' ')]
#                print(tags[3]-tags[1], tags[4]-tags[2])

"""
" Main execution block
"""
# Create a TCP/IP socket and bind the socket to the port
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = (IP_ADDRESS, PORT)
print('Starting up on {0}, port {1}' .format(server_address[0], server_address[1]), file=sys.stderr) #>>sys.stderr, 'Starting up on %s, port %s' % server_address
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(server_address)

stream = io.BytesIO()
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
