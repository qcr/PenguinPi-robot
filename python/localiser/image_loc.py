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

src_points = np.array([[148, 1], [600, 1], [124, 471], [627, 471]])
dst_points = np.array([[0,0],[500,0],[0,500], [500,500]])
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

    def __init__(self, xx, yy, ww, hh, mx, my):
        self.x = xx
        self.y = yy
        self.w = ww
        self.h = hh
        self.cx = xx + (ww/2)
        self.cy = yy + (hh/2)
        self.mx = mx
        self.my = my

class Contours:
    def __init__(self):
        self.list_contours = []
        self.list_boxes = []
        self.dec = []
    def get_contours(self, contours):
        minimum_area = 20
        # Find all contours
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            area = w * h
            if area > minimum_area:
                if (x > 20 and x < 480) and (y > 20 and y < 480):
                    self.list_contours.append(contour)
                    M = cv2.moments(contour)
                    mx = int(M['m10']/M['m00'])
                    my = int(M['m01']/M['m00'])
                    self.list_boxes.append(Box(x,y,w,h,mx,my))
                else:
                    logging.debug("Out of bounds")


def ProcessImage(im1):
    global x
    global y
    global angle

    im1 = cv2.warpPerspective(im1, h, (500,500))
    hsv = cv2.cvtColor(im1, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, red_min, red_max)
    mask_robot = cv2.inRange(hsv, bot_min, bot_max)
    # cv2.imshow('Rob still smells', im1)
    # cv2.waitKey(1)
    dilated = cv2.dilate(mask_red, red_kernel, iterations = 5)
    eroded = cv2.erode(dilated, red_kernel, iterations = 3)

    robot_dilated = cv2.dilate(mask_robot, red_kernel, iterations = 5)
    robot_eroded = cv2.erode(robot_dilated, red_kernel, iterations = 3)

    im2, robot_contours, hierarchy_rbt = cv2.findContours(robot_eroded.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    im2, red_contours, hierarchy_rbt = cv2.findContours(eroded.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if type(red_contours) != list:
        red_contours = [red_contours]

    conts = Contours()
    conts.get_contours(robot_contours)
    boxes = conts.list_boxes

    red_conts = Contours()
    red_conts.get_contours(red_contours)
    red_boxes = red_conts.list_boxes

    if len(boxes) > 0 and len(red_boxes) > 0:
        for box in boxes:
            cv2.rectangle(im1,(box.x,box.y),(box.x+box.w,box.y+box.h),(255,100,0),5)

        for box in red_boxes:
            cv2.rectangle(im1,(box.x,box.y),(box.x+box.w,box.y+box.h),(0,100,255),5)
        robot = boxes[0]
        usb = red_boxes[0]

        # Update the pose values
        pose_lock.acquire()

        angle = -np.degrees(np.arctan2(robot.my - usb.cy, robot.mx - usb.cx))

        l =100
        ang = -np.radians(angle)
        cv2.arrowedLine(im1, (round(robot.mx), round(robot.my)), (round(robot.mx + l * math.cos(ang)), round(robot.my + l * math.sin(ang))), (0,0,255), 5)
        x = robot.mx/500 * 2
        y = robot.my/500 * 2
        #convert to right hand rule
        x = -(x - 1)   
        y = y - 1 
        angle = angle + 180

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
    kernel = np.ones((5,5), np.uint8)
    red_kernel = np.ones((4,4), np.uint8)

    img_counter = 0
    # Red
    channel1Min = 0.860;
    channel1Max = 0.980;

    channel2Min = 0.160;
    channel2Max = 0.656;

    channel3Min = 0.667;
    channel3Max = 0.965;

    red_min = np.array([channel1Min * 180, channel2Min * 255, channel3Min * 255])
    red_max = np.array([channel1Max * 180, channel2Max * 255, channel3Max * 255])

    # # BLUE 
    channel1Min = 0.45;
    channel1Max = 0.63;

    channel2Min = 0.20;
    channel2Max = 0.69;

    channel3Min = 0.44;
    channel3Max = 0.95;

    # Ro#
    bot_min = np.array([channel1Min * 180, channel2Min * 255, channel3Min * 255])
    bot_max = np.array([channel1Max * 180, channel2Max * 255, channel3Max * 255])

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


    camera.stop()
