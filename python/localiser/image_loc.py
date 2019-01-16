import socket
import math
import cv2
import numpy as np
import time
import sys
import threading
import logging
import signal
import io

from socket_helpers import *
import piVideoStream

logging.basicConfig(level=logging.DEBUG,
                      format='[%(levelname)s] (%(threadName)-9s) %(message)s',)

"""
" Constants
"""
PORT = 43905
IM_PORT = 43906
CHUNK_SIZE = 128
IM_WIDTH = 640
IM_HEIGHT = 480
FRAMERATE = 32


"""
" Variables
"""
x = 0
y = 0
angle = 0
pose_lock = threading.RLock()
shutdown_sig = False

src_points = np.array([[537, 1], [85, 1],[72, 467], [569, 450]])
dst_points = np.array([[0,0],[500,0],[500,500], [0,500]])
h, status = cv2.findHomography(src_points, dst_points)

def StartServer():
    # Starts the listener servers each in a thread
    server_thr = threading.Thread(name="ServerListener", target=Listen, args=(PORT,), daemon=True)
    im_server_thr = threading.Thread(name="ImServerListener", target=ImListen, args=(IM_PORT,), daemon=True)
    server_thr.start()
    im_server_thr.start()

def ImListen(port, num_of_connects=1):
    # Start a socket and bind to the listening port.
    # 0.0.0.0 is localhost but on all network interfaces.
    sock = socket.socket()
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allows immediate reconnection in the event of program forcequitting
    sock.bind(('0.0.0.0', port))
    logging.debug("Server started on %s. Listning on port %d...", socket.gethostname(), port)

    sock.listen(num_of_connects)

    while (not shutdown_sig):
        conn, addr = sock.accept()          # Establish connection with client.
        logging.debug("Got connection from %s:%d", addr[0], addr[1])
        thr = threading.Thread(name=addr, target=ImHandleClient, args=(conn,))
        thr.start()


def ImHandleClient(conn):
    '''
    To be run in a new thread when a client connects.
    Choose what to do with the client data here.
    '''
    logging.debug("Going to send Im")
    if (camera.frame_available):
        image = camera.read()
        #stream=io.BytesIO(image)
        #conn.sendall(image.tostring())

        warped_im = cv2.warpPerspective(image, h, (500,500))
        # cv2.imshow('im',warped_im)
        # cv2.waitKey(100)
        conn.sendall(warped_im)

        # SendImage(conn, warped_im)
        logging.debug("Sent Image")



def Listen(port, num_of_connects=20):
    # Start a socket and bind to the listening port.
    # 0.0.0.0 is localhost but on all network interfaces.
    sock = socket.socket()
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allows immediate reconnection in the event of program forcequitting
    sock.bind(('0.0.0.0', port))
    logging.debug("Server started on %s. Listning on port %d...", socket.gethostname(), port)

    sock.listen(num_of_connects)

    while (not shutdown_sig):
        conn, addr = sock.accept()          # Establish connection with client.
        logging.debug("Got connection from %s:%d", addr[0], addr[1])
        thr = threading.Thread(name=addr, target=HandleClient, args=(conn,))
        thr.start()


def HandleClient(conn):
    '''
    To be run in a new thread when a client connects.
    Choose what to do with the client data here.
    '''
    data = conn.recv(CHUNK_SIZE)
    logging.debug("Data received: %s", data.decode())
    #if data.decode().rstrip() == 'Pose Please!':
    logging.debug("Sending Pose!")

    pose_lock.acquire()
    pose = [x,y,angle]
    pose_lock.release()

    send_pose(pose, conn)


# Send the calculated pose over the network
def send_pose(pose, connection):
    # Convert pose to string and then byte encode for socket
    s = str(pose[0]) + " " + str(pose[1]) + " " + str(pose[2]) + '\n'
    b = s.encode('utf-8')

    # send ticks back over 'connection
    connection.sendall(b)


# Object Detection
class Box:
    def __init__(self):
        self.x = None
        self.y = None
        self.w = None
        self.h = None

    def __init__(self, xx, yy, ww, hh):
        self.x = xx
        self.y = yy
        self.w = ww
        self.h = hh
        self.cx = xx + (ww/2)
        self.cy = yy + (hh/2)
        # self.mx = mx
        # self.my = my

class Contours:
    def __init__(self):
        self.list_contours = []
        self.list_boxes = []
        self.dec = []
    def get_contours(self, contours):
        minimum_area = 5
        # Find all contours
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            area = w * h
            if area > minimum_area:
                self.list_contours.append(contour)
                self.list_boxes.append(Box(x,y,w,h))
            else:
                logging.debug("Out of bounds")


def ProcessImage(im1):
    global x
    global y
    global angle

    im1 = cv2.warpPerspective(im1, h, (500,500))
    mask = cv2.inRange(im1, bot_min, bot_max)
    # dilated = cv2.dilate(mask, kernel, iterations = 1)
    # eroded = cv2.erode(dilated, kernel, iterations = 1)
    eroded = mask
    sm = cv2.resize(eroded, (320,240))
    im2, robot_contours, hierarchy_rbt = cv2.findContours(eroded.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.imshow("contours", im2)
    # cv2.waitKey()
    conts = Contours()
    conts.get_contours(robot_contours)
    boxes = conts.list_boxes
    
    
    min_x = 100000
    max_x = 0
    min_y = 100000
    max_y = 0

    # checked = []

    for box in boxes:
        if box.cx < min_x:
            min_x = box.cx

        if box.cx > max_x:
            max_x = box.cx

        if box.cy < min_y:
            min_y = box.cy

        if box.cy > max_y:
            max_y = box.cy


    center_box = None

    for box in boxes:
        if box.cx < max_x and box.cx > min_x and box.cy < max_y and box.cy > min_y:
            center_box = box
    
    if center_box:   
        dists_boxes = []
        for box2 in boxes:
            if (center_box != box2):
                a = np.array([center_box.cx, center_box.cy])
                b = np.array([box2.cx, box2.cy])
                d = np.linalg.norm(a - b)
                # print(d)
                dist_box = (d, box2)
                dists_boxes.append(dist_box)

        dists_boxes = sorted(dists_boxes, key=lambda x: x[0])
            # print(dists_boxes)
        closest_two = dists_boxes[0:2]
        # print(closest_two)
        mid_point_x = (closest_two[0][1].cx + closest_two[1][1].cx)/2
        mid_point_y = (closest_two[0][1].cy + closest_two[1][1].cy)/2
        # Update the pose values
        pose_lock.acquire()


        angle = np.arctan2(center_box.cy-mid_point_y, center_box.cx-mid_point_x)
        angle = np.rad2deg(angle)
        x = (center_box.cx / 500)*2
        y = (center_box.cy / 500)*2



        pose_lock.release()

        logging.debug("Pose: %8.3f %8.3f %8.2f",  x, y, angle)
        
    else:
        logging.debug("Nothing found")


'''
Captures the cntl+C keyboard command to close the services and free 
resources gracefully.
Allows the sockets and camera to be immediately reopened on next run without
waiting for the OS to close them on us.
'''
def signal_handler(signal, frame):
    global shutdown_sig
    shutdown_sig = True
    logging.debug('Closing gracefully. Bye Bye!')


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    StartServer()

    # Image Processing Variables
    num_iter = 10
    kernel = np.ones((2,2), np.uint8)

    img_counter = 0
    
    bot_min = np.array([150,150,150])
    bot_max = np.array([255,255,255])

    camera = piVideoStream.PiVideoStream(
        resolution=(IM_WIDTH, IM_HEIGHT),
        framerate=FRAMERATE
    )
    camera.start()

    while (not shutdown_sig):
        start_time = time.time()
        if (camera.frame_available):
            im1 =  camera.read()
            ProcessImage(im1)
            # cv2.imshow("live", im1)
            # cv2.waitKey()

    camera.stop()
