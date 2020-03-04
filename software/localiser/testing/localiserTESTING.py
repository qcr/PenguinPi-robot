#!/usr/bin/env python3

import time
import sys
import threading
import os
import argparse
import io
import math
import json
import logging

import picamera
import cv2
import numpy as np
from flask import Flask, request, render_template, redirect, send_file

# set default values
log_level = logging.INFO
localizer_rate = 10 # Hz

IM_WIDTH = 640
IM_HEIGHT = 480

app = Flask(__name__)

# display_img = None
robot = False
# robot estimated pose
x = 0
y = 0
theta = 0
# grey = None
group_number = 0


# USER web page: home page
#
# a bit of everything
@app.route('/console', methods = ['GET'])
def home():
    
    # render the page
    state = {
        "pose_x": x,
        "pose_y": y,
        "pose_theta": theta,
        "group" : group_number
    }

    # get refresh
    refresh = request.args.get('refresh')
    if refresh:
            state['refresh'] = refresh

    return render_template('home.html', **state)

@app.route('/camera/get', methods = ['GET'])
def picam():

    global display_img,image
    if robot:
        # encode the grey scale image as PNG
        image_data = cv2.imencode('.png', display_img)[1]
    else:
        image_data = cv2.imencode('.png', image)[1]
    # make it streamable and return an HTTP image response
    return send_file(io.BytesIO(image_data), 'image/png')

@app.route('/pose/get', methods = ['GET'])
def poseget():

    t0 = time.time()
    global group_number

    group_number = request.args.get('group')

    # build a dictionary of stuff to return
    state = {
        'pose' : {
                'x'     : x,
                'y'     : y,
                'theta' : theta
                }
            }
    # return a JSON encoding
    
    t1 = time.time()

    #print("Time taken to get pose from shared memory before serving: {} sec".format(t1-t0))

    return json.dumps(state)


"""
" Localizer thread
"""
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
class Contours:
    def __init__(self):
        self.list_contours = []
        self.list_boxes = []
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
                log.debug("Out of bounds")



# Capture frames from the video camera repeatedly so the latest frame is always available
def VideoCaptureThread():
    stream = cv2.VideoCapture(0)
    while(True):
        ret, frame = stream.read()
        cv2.imshow('frame',frame)
        cv2.waitKey(1)

def LocalizerThread():
    global x, y, theta, display_img, image

    log.info('Localizer thread launched, running at %.1f Hz' % args.localizer_rate)
    rate = args.localizer_rate
    stream = io.BytesIO()
    camera = piVideoStream.PiVideoStream(
            resolution=(IM_WIDTH, IM_HEIGHT),
            framerate=32
        )
    camera.start()
  
    # Homography 
    src_points = np.array([[6, 499], [480, 499],[475, 30], [6, 25]])
    dst_points = np.array([[0,0],[500,0],[500,500], [0,500]])
    h, status = cv2.findHomography(src_points, dst_points)

    while True:

        t0 = time.time()

        if (not camera.frame_available):
            continue 
            
        log.debug('process frame')
        # Get the image, rectify and find contours 
        im1 =  camera.read()

        t1 = time.time()
        camera_time = t1-t0
        #print("Time taken to wait for and read from camera: {}".format(t1-t0))
        im1 = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
        img = cv2.warpPerspective(im1, h, (500,500))
        mask = cv2.inRange(img, 220,255)
        im2, robot_contours, hierarchy_rbt = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     

        img1 = cv2.flip(img.copy(),-1)
        image = cv2.flip(img1, 1)
    

        # cv2.imshow('im',mask)
        # cv2.waitKey()
        # Process contours into boxes 
        conts = Contours()
        conts.get_contours(robot_contours)
        boxes = conts.list_boxes
        
        
        min_x = 100000
        max_x = 0
        min_y = 100000
        max_y = 0


        # Find outer limits of IR LEDs
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

        # Find that center LED
        for box in boxes:
            if box.cx < max_x and box.cx > min_x and box.cy < max_y and box.cy > min_y:
                center_box = box
       
        # Find the 2 LEDs closest to center LED
        if center_box:  
            robot = True
            dists_boxes = []
            for box2 in boxes:
                if (center_box != box2):
                    a = np.array([center_box.cx, center_box.cy])
                    b = np.array([box2.cx, box2.cy])
                    d = np.linalg.norm(a - b)
                    dist_box = (d, box2)
                    dists_boxes.append(dist_box)

            dists_boxes = sorted(dists_boxes, key=lambda x: x[0])
            closest_two = dists_boxes[0:2]
            # display_img1 = im1 
            # Use point between the two closest LEDs and center of middle LED to find angle
            mid_point_x = (closest_two[0][1].cx + closest_two[1][1].cx)/2
            mid_point_y = (closest_two[0][1].cy + closest_two[1][1].cy)/2

            angle = np.arctan2(center_box.cy-mid_point_y, center_box.cx-mid_point_x)
            ang = angle.copy()
            l = 35
            cv2.arrowedLine(image, (round(center_box.cx), round(500-center_box.cy)), (round(center_box.cx + l * math.cos(ang)), round(500-center_box.cy + l * math.sin(np.pi+ang))), (200), 2)
            # cv2.arrowedLine(image, (round(center_box.cx), round(500-center_box.cy)), (round(center_box.cx + l * math.sin(ang)), round(500-center_box.cy + l * math.cos(ang))), (200), 2)
            # display_img1 = im1
            # display_img1 = cv2.flip(img.copy(),-1)
            # display_img = cv2.flip(display_img1, 1)
    
            # cv2.imshow("im",im1)
            # cv2.waitKey()

            angle = np.rad2deg(angle)
            x = (center_box.cx / 500)*2
            y = (center_box.cy / 500)*2


            log.debug("Pose: %8.3f %8.3f %8.2f",  x, y, angle)
            theta = angle
            
        else:
            log.debug("Nothing found")
            robot = False
            theta = 0
            x = 0
            y = 0

  # do clever stuff here
        #   if you want to create a cv2 image with overlayed graphics in it, I can make
        #   that appear in the /console view

        # sleep a bit
        t2 = time.time()
        vision_time = t2 - t0
        required_pause = (1./rate - vision_time)
        time.sleep(required_pause)
        t3 = time.time()
        localiser_time = t3 - t0
        print("Localiser thread time: {:.3f} sec of which {:.3f} is camera and {:.3f} is computer vision".format(localiser_time,camera_time,vision_time))


"""
" main execution block
"""
if __name__ == '__main__':

    # handle command line arguments
    parser = argparse.ArgumentParser(
        description="Web interface for PenguinPi robot",
        epilog="default logging level is {}".format(logging.getLevelName(log_level)))
    parser.set_defaults(
        log_level=log_level,
        localizer_rate = localizer_rate,
    )
    parser.add_argument("-d", "--debug",
        help="log DEBUG messages", 
        dest='log_level',
        action="store_const", const=logging.DEBUG)
    parser.add_argument("-i", "--info",
        help="log INFO messages", 
        dest='log_level',
        action="store_const", const=logging.INFO)
    parser.add_argument('--log-level',
        dest='log_level',
        type=lambda s: getattr(logging, s.upper()),
        nargs='?',
        help='Set the logging output level to DEBUG|INFO|WARNING|ERROR|CRITICAL')
    parser.add_argument("--localizer-rate", help="set localizer rate (Hz)",
        dest="localizer_rate", action="store", type=float)

    args = parser.parse_args()

    # everybody uses the Flask logger
    log = logging.getLogger('werkzeug')  # the Flask log channel
    log.setLevel(args.log_level)
    logging.basicConfig(format='%(asctime)s %(levelname)s  %(message)s')

    # Get the camera up and running
    # see http://picamera.readthedocs.io/en/release-1.10/api_camera.html for details
    # connect to the camera
    log.info('Connecting to the camera')
    # try:
    #     camera = picamera.PiCamera()
    # except:
    #     log.fatal("Couldn't open camera connection -- is it being used by another process?");
    # camera.resolution = (IM_WIDTH, IM_HEIGHT)
    # camera.rotation = 180
    # log.debug('camera running')
    #
    # launch the localizer thread
    localizer_thread = threading.Thread(target=LocalizerThread, daemon=True)
    #localizer_thread.start()

    # Start a video stream on default camera
    stream_thread = threading.Thread(target=VideoCaptureThread,  daemon=True)
    stream_thread.start()

    # fire up the webserver on a non-priviliged port
    app.jinja_env.lstrip_blocks = True
    app.jinja_env.trim_blocks = True
    app.jinja_env.line_statement_prefix = '#'

    app.run(host='0.0.0.0', port=8080)
