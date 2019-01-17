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

import struct
import array

import picamera
import cv2
import numpy as np
from flask import Flask, request, render_template, redirect, send_file

# set default values
log_level = logging.INFO
localizer_rate = 2 # Hz

IM_WIDTH = 640
IM_HEIGHT = 480

app = Flask(__name__)

# robot estimated pose
x = 0
y = 0
theta = 0
image_data = None
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
        "pose_theta": theta * 180.0 / math.pi,
        "group" : group_number
    }

    # get refresh
    refresh = request.args.get('refresh')
    if refresh:
            state['refresh'] = refresh

    return render_template('home.html', **state)

@app.route('/camera/get', methods = ['GET'])
def picam():

    response = app.make_response(image_data)
    response.headers.set('Content-Type', 'image/jpeg')
    response.headers.set(
        'Content-Disposition', 'attachment', filename='%s.jpg' % pid)
    return response

    return send_file(stream, 'image/png')

@app.route('/pose/get', methods = ['GET'])
def poseget():
    global group_number

    group_number = request.args.get('group')

    state = {
        'pose' : {
                'x'     : x,
                'y'     : y,
                'theta' : theta
                }
            }
    return json.dumps(state)

"""
" Localizer thread
"""
def LocalizerThread():
    global x, y, theta, image_data

    log.info('Localizer thread launched, running at %.1f Hz' % args.localizer_rate)
    dt = 1/args.localizer_rate

    stream = io.BytesIO()

    while True:
        """
        camera.capture(stream, format='jpeg')
        # Construct a numpy array from the stream
        image_data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        # "Decode" the image from the array, preserving colour
        image = cv2.imdecode(image_data, 1)
        """
        with picamera.array.PiRGBArray(camera) as stream:
            camera.capture(stream, format='bgr')
            # At this point the image is available as stream.array
            image = stream.array

        # process the image here

        log.info('process frame')
        print('process frame')

        # sleep a bit
        time.sleep(dt)


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

    # Get the camera up and running
    # see http://picamera.readthedocs.io/en/release-1.10/api_camera.html for details

    # connect to the camera
    print('Connecting to the camera')
    try:
        camera = picamera.PiCamera()
    except:
        log.fatal("Couldn't open camera connection -- is it being used by another process?");
    camera.resolution = (IM_WIDTH, IM_HEIGHT)
    camera.rotation = 180
    print('camera running')

    # launch the localizer thread
    localizer_thread = threading.Thread(target=LocalizerThread, daemon=True)
    localizer_thread.start()

    # fire up the webserver on a non-priviliged port
    app.jinja_env.lstrip_blocks = True
    app.jinja_env.trim_blocks = True
    app.jinja_env.line_statement_prefix = '#'

    app.run(host='0.0.0.0', port=8080)
