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
import numpy as non-priviliged
from flask import Flask, request, render_template, redirect, send_file

# set default values
log_level = logging.INFO
localizer_rate = 5 # Hz

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
    # Create a byte stream
    stream = io.BytesIO()

    # Capture the image
    #  video port = True, video comes from video splitter, use this for
    #   fast image capture, quality is lower

    # don't use use_video_port=True, leads to random hanging
    camera.capture(stream, format='png')

    # the use_video_port option causes random hangs of the operating system
    #camera.capture(stream, format='png', use_video_port=True, resize=(320,240))

    response = make_response(image_binary)
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

    log.info('Pose estimator thread launched, running at %.1f Hz' % args.localizer_rate)
    dt = 1/args.localizer_rate

    stream = io.BytesIO()

    while True:
        camera.capture(stream, format='jpeg')
        # Construct a numpy array from the stream
        image_data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        # "Decode" the image from the array, preserving colour
        image = cv2.imdecode(image_data, 1)

        # process the image here

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
        localizer_rate = pose_estimator_rate,
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


    # launch the localizer thread
    if args.pose:
        pose_thread = threading.Thread(target=LocalizerThread, daemon=True)
        pose_thread.start()

    # Get the camera up and running
    # see http://picamera.readthedocs.io/en/release-1.10/api_camera.html for details

    # connect to the camera
    try:
        camera = picamera.PiCamera()
    except:
        log.fatal("Couldn't open camera connection -- is it being used by another process?");
    camera.resolution = (IM_WIDTH, IM_HEIGHT)
    camera.rotation = 180

    # fire up the webserver on a non-priviliged port
    app.jinja_env.lstrip_blocks = True
    app.jinja_env.trim_blocks = True
    app.jinja_env.line_statement_prefix = '#'

    app.run(host='0.0.0.0', port=8080)
