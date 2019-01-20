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
localizer_rate = 2 # Hz

IM_WIDTH = 640
IM_HEIGHT = 480

app = Flask(__name__)

# robot estimated pose
x = 0
y = 0
theta = 0
grey = None
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

    global grey

    if grey:
        # encode the grey scale image as PNG
        image_data = cv2.imencode('.png', grey)[1]
        # make it streamable and return an HTTP image response
        return send_file(io.BytesIO(image_data), 'image/png')

@app.route('/pose/get', methods = ['GET'])
def poseget():
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
    return json.dumps(state)

"""
" Localizer thread
"""
def LocalizerThread():
    global x, y, theta, grey

    log.info('Localizer thread launched, running at %.1f Hz' % args.localizer_rate)
    dt = 1/args.localizer_rate

    stream = io.BytesIO()

    while True:
        # grab a frame
        camera.capture(stream, format='png')
        # convert it to a grey scale image, which is global, used
        # when an image is requested
        image_data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        image = cv2.imdecode(image_data, 1)
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        log.debug('process frame')

        # do clever stuff here
        #   if you want to create a cv2 image with overlayed graphics in it, I can make
        #   that appear in the /console view

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
    logging.basicConfig(format='%(asctime)s %(levelname)s  %(message)s')

    # Get the camera up and running
    # see http://picamera.readthedocs.io/en/release-1.10/api_camera.html for details
    # connect to the camera
    log.info('Connecting to the camera')
    try:
        camera = picamera.PiCamera()
    except:
        log.fatal("Couldn't open camera connection -- is it being used by another process?");
    camera.resolution = (IM_WIDTH, IM_HEIGHT)
    camera.rotation = 180
    log.debug('camera running')

    # launch the localizer thread
    localizer_thread = threading.Thread(target=LocalizerThread, daemon=True)
    localizer_thread.start()

    # fire up the webserver on a non-priviliged port
    app.jinja_env.lstrip_blocks = True
    app.jinja_env.trim_blocks = True
    app.jinja_env.line_statement_prefix = '#'

    app.run(host='0.0.0.0', port=8080)
