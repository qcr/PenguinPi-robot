#!/usr/bin/python3

import time
import traceback
import socket
import sys
import threading
import os
import argparse
import io
import math
import json
import logging

import penguinPi as ppi
import picamera

from flask import Flask, request, render_template, redirect, send_file

app = Flask(__name__)
IM_WIDTH = 320
IM_HEIGHT = 240

# robot estimated pose
x = 0
y = 0
theta = 0

# http://flask.pocoo.org/docs/1.0/quickstart/

# request.args is a MultiDict, dictionary subclass
# http://werkzeug.pocoo.org/docs/0.14/datastructures/#werkzeug.datastructures.ImmutableMultiDict

# button has
#  <input type = "submit" name = A  value = B />
#   B is the displayed label, default is submit
#   A is the name which is "in" the form

count = 0;

@app.route('/', methods = ['POST', 'GET'])
def home():
    if request.method == 'POST':
        if "refresh" in request.form:
            app.logging.debug("refresh")
        elif "test_l" in request.form:
            app.logging.debug("testL")
        elif "test_r" in request.form:
            app.logging.debug("testR")

    # read the robot state
    ea = mLeft.get_encoder()
    eb = mRight.get_encoder()
    v = "%.2f" % voltage.get_value()

    # get info about the Raspberry Pi
    with open('/sys/firmware/devicetree/base/model') as f:
        model = f.read()
    for _ in (True,):
        with open('/etc/os-release') as f:
            line = f.readline()
            distro = line.split('=')[1];
            break
    with open('/proc/version') as f:
        kernel = f.read()

    # render the page
    state = {
            "enc_l": ea,
            "enc_r": eb,
            "volts": v,
            "pose_x": x,
            "pose_y": y,
            "pose_theta": theta,
            "model": model,
            "distro": distro,
            "kernel": kernel,
            "refresh": 5
            }

    # get refresh
    refresh = request.args.get('refresh');
    if refresh:
            state['refresh'] = refresh

    return render_template('home.html', **state)

@app.route('/voltage')
def voltage():
    return str( voltage.get_value() )

sp1 = 0
sp2 = 0

# push stop (STOP, stop) 
# push submit (Set, submit)
# (Left, value)
# (Right, value)
@app.route('/speed', methods = ['POST', 'GET'])
def speed():
    global sp1, sp2
    if args.debug:
        app.logging.debug('--- set velocity\n');
    if request.method == 'POST':
        if "Set" in request.form:
            sp1 = request.form['Left']
            sp2 = request.form['Right']
            sp1 = int(sp1)
            sp2 = int(sp2)
            mLeft.set_speed(sp1);
            mRight.set_speed(sp2);
        elif "STOP" in request.form:
            sp1 = 0
            sp2 = 0
            mLeft.set_speed(sp1);
            mRight.set_speed(sp2);
    return render_template('speed.html', speed_l=sp1, speed_r=sp2);

@app.route('/camera', methods = ['POST', 'GET'])
def camera():

    def update_int(s):
        if request.form[s] != camera_state[s]:
            setattr(camera, s, int(request.form[s]))
            camera_state[s] = request.form[s]
            app.logging.debug('Updating camera parameter %s' % s)
    def update(s):
        if request.form[s] != camera_state[s]:
            setattr(camera, s, request.form[s])
            camera_state[s] = request.form[s] 
            app.logging.debug('Updating camera parameter %s' % s)
            
    if request.method == 'POST':
        app.logging.debug('Camera POST', request.form)
        update_int('rotation')
        update('awb_mode')
        #update('dynamic_range')
        update_int('iso')
        update_int('brightness')

    # get refresh
    refresh = request.args.get('refresh');
    if refresh:
            camera_state['refresh'] = refresh

    return render_template('camera.html', **camera_state)

@app.route('/get/camera')
def picam():
    # Create a byte stream
    stream = io.BytesIO()

    # Capture the image
    #  video port = True, video comes from video splitter, use this for
    #   fast image capture, quality is lower
    camera.capture(stream, format='png', use_video_port=True)
    #camera.capture(stream, format='png', use_video_port=True, resize=(320,240))

    # Send the image over the connection
    stream.seek(0)

    return send_file(stream, 'image/png')

@app.route('/get/encoders')
def getencoders():
    global mLeft, mRight
    ea = mLeft.get_encoder()
    eb = mRight.get_encoder()
    if args.debug:
        app.logging.debug('--- get encoders: %d %d\n' % (ea,eb));
    return "%d,%d" % (ea, eb)

@app.route('/set/motors')
def motors():

    # TODO: the trajectory could be done by the pose estimation thread
    dt = 0.05;

    # coroutine to do a floating point version of xrange
    def xfrange(start, stop, step):
        i = 0
        while start + i * step < stop:
            yield start + i * step
            i += 1


    # set motor speed using GET side effects
    speeds = request.args.get('speed')
    if speeds:
        try:
            speeds = [int(x) for x in speeds.split(',')]
        except:
            return "bad speeds given"

        duration = request.args.get('time')
        if duration:
            # a duration was given

            # get duration
            try:
                Ttotal = float(duration)
            except:
                return "bad time given"

            # get optional acceleration
            accel = request.args.get('accel')
            if accel:
                try:
                    Taccel = float(accel)
                except:
                    return "bad acceleration given"
            else:
                Taccel = 0.0

            if Ttotal <= Taccel*2:
                return "acceleration time too long"

            if Taccel > 0:
                    for t in xfrange(0, Taccel, dt):
                            setspeed(speeds, t/Taccel);
                            time.sleep(dt)

            for t in xfrange(0, Ttotal-Taccel, dt):
                    setspeed(speeds, 1.0)
                    time.sleep(dt)

            if Taccel > 0:
                    for t in xfrange(0, Taccel, dt):
                            setspeed(speeds, (Taccel-t)/Taccel);
                            time.sleep(dt)
            # all stop
            stop_all()

        else:
            # no duration given

            setspeed(speeds)

    return robot_state_json()


@app.route('/reset')
def reset():
    global x, y, theta

    x = 0
    y = 0
    theta = 0
    return robot_state_json()

@app.route('/stop')
def stop():
    stop_all()
    return robot_state_json()

"""
" Helper functions
"""
def setspeed(speed, fraction=1.0):
    global mLeft, mRight

    mLeft.set_speed(int(speed[0]*fraction))
    mRight.set_speed(int(speed[1]*fraction))

def stop_all():
    global mLeft, mRight

    mLeft.set_speed(0)
    mRight.set_speed(0)

def robot_state_json():
    state = { 'encoder' : {
                    'left'  : mLeft.get_encoder(),
                    'right' : mRight.get_encoder()
                    },
              'pose' : {
                    'x'     : x, 
                    'y'     : y,
                    'theta' : theta
                   }
              }
    return json.dumps(state)


"""
" Heartbeat thread, pulse the red LED periodically
"""
def HeartBeat():

    led = ppi.LED('AD_LED_R')

    while True:
        led.set_count(200);
        time.sleep(5);

"""
" Pose estimation thread
"""
def PoseEstimator():
    global x, y, theta

    dt = 0.2   # sample interval
    W = 0.156  # lateral wheel separation
    wheelDiam = 0.065;
    encScale = math.pi * wheelDiam /384 

    # read the initial encoders 
    left = mLeft.get_encoder()
    right = mRight.get_encoder()

    def encoder_difference(a, b):
        d = a - b
        if d > 32000:
            d = 0x10000 - d
        elif d < -32000:
            d += 0x10000
        return d

    while True:
        # read the encoders 
        new_left = mLeft.get_encoder()
        new_right = mRight.get_encoder()

        # check if bad read value
        if new_left is None or new_right is None:
            app.logging.error('bad encoder read')
            continue

        # compute the difference since last sample and handle 16-bit wrapping
        dL = encoder_difference(new_left, left)
        dR = encoder_difference(new_right, right)
        left = new_left
        right = new_right

        # compute average and differential wheel motion
        #  this is average and differential wheel speed * dt
        avg = encScale * (dL + dR) / 2
        diff = encScale * (dL - dR)

        # update the state
        #   no need to multiply by dt, it's included already
        theta_old = theta
        theta += diff / W;          # update theta
        theta_avg = (theta + theta_old)/2   # average theta over the interval
        x += avg * math.cos(theta_avg)      # update position
        y += avg * math.sin(theta_avg)
        #print('Estimated pose %f %f %f (enc=%f %f)' % (x,y,theta,left,right))
        while theta > 2*math.pi:
            theta -= 2*math.pi
        while theta < -2*math.pi:
            theta += 2*math.pi

        # sleep a bit
        time.sleep(dt);


"""
" main execution block
"""
if __name__ == '__main__':

    # handle command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", help="show debug information", 
            action="store_true")
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

    mLeft = ppi.Motor('AD_MOTOR_L')
    mRight = ppi.Motor('AD_MOTOR_R')
    voltage = ppi.AnalogIn('AD_ADC_V')

    #initialise serial, and retrieve initial values from the Atmega
    ppi.init()
    mLeft.get_all()
    mRight.get_all()

    # launch the heartbeat thread
    heartbeat_thread = threading.Thread(target=HeartBeat, daemon=True)
    heartbeat_thread.start()

    # launch the pose estimation thread
    pose_thread = threading.Thread(target=PoseEstimator, daemon=True)
    pose_thread.start()

    # Get the camera up and running
    # see http://picamera.readthedocs.io/en/release-1.10/api_camera.html for details

    camera = picamera.PiCamera()
    camera.resolution = (IM_WIDTH, IM_HEIGHT)

    camera_state = {
            "rotation": str(camera.rotation),
            "awb_mode": camera.awb_mode,
            "dynamic_range": camera.drc_strength,
            "iso": str(camera.iso),
            "brightness": str(camera.brightness),
            "exposure_speed": camera.exposure_speed,
            "shutter_speed": camera.shutter_speed,
            "meter_mode": camera.meter_mode,
            "zoom": camera.zoom
            }
    logging.debug(camera_state)
    #camera.start_preview()

    if args.awb:
            camera.awb_mode = args.awb
    if args.gain:
            camera.awb_gains = tuple(float(x) for x in args.gain.split(','))
    logging.debug('white balance mode is ', camera.awb_mode)

    # open a non-priviliged port
    app.jinja_env.lstrip_blocks = True
    app.jinja_env.trim_blocks = True
    app.jinja_env.line_statement_prefix = '#'

    log = logging.getLogger('werkzeug')  # the Flask log channel
    #log.setLevel(logging.ERROR)

    app.run(host='0.0.0.0', port=8080)
