#!usr/bin/python3

import time
import traceback
import socket
import sys
import threading
import os
import argparse
import io
import math

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
        print("POST")
        if "refresh" in request.form:
            print("refresh")
        elif "test_l" in request.form:
            print("testL")
        elif "test_r" in request.form:
            print("testR")
    else:
        print("GET")

    ea = mA.get_encoder()
    eb = mB.get_encoder()
    v = "%.2f" % voltage.get_value()
    with open('/sys/firmware/devicetree/base/model') as f:
        model = f.read()
    state = {
            "enc_l": ea,
            "enc_r": eb,
            "volts": v,
            "pose_x": x,
            "pose_y": y,
            "pose_theta": theta,
            "model": model
            }
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
        print('--- set velocity\n');
    print(request.form)
    if request.method == 'POST':
        if "Set" in request.form:
            sp1 = request.form['Left']
            sp2 = request.form['Right']
            sp1 = int(sp1)
            sp2 = int(sp2)
            print(sp1, sp2)
            mA.set_power(sp1);
            mB.set_power(sp2);
            print("submit")
        elif "STOP" in request.form:
            sp1 = 0
            sp2 = 0
            mA.set_power(sp1);
            mB.set_power(sp2);
            print("stop")
    print("speeds:", sp1, sp2)
    return render_template('speed.html', speed_l=sp1, speed_r=sp2);

@app.route('/camera', methods = ['POST', 'GET'])
def camera():

    def update_int(s):
        if request.form[s] != camera_state[s]:
            setattr(camera, s, int(request.form[s]))
            camera_state[s] = request.form[s]
            print('Updating camera parameter %s' % s)
    def update(s):
        if request.form[s] != camera_state[s]:
            setattr(camera, s, request.form[s])
            camera_state[s] = request.form[s] 
            print('Updating camera parameter %s' % s)
            
    if request.method == 'POST':
        print('Camera POST', request.form)
        update_int('rotation')
        update('awb_mode')
        #update('dynamic_range')
        update_int('iso')
        update_int('brightness')

    print('Camera GET', camera_state)
    return render_template('camera.html', **camera_state)

@app.route('/picam')
def picam():
    # parameters
    # rotation
    # resolution=0,90,180,270
    # awb =off,auto,sunlight,cloudy,shade,tungsten,fluorescent,incandescent,flash,horizon
    # brightness=0 to 100, default 50
    # exposure_mode
    # image_effect
    # iso
    # meter_mode
    # iso
    # zoom 0 to 1
    # preview=0 or 1, start/stop_preview

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

@app.route('/set')
def set():
    # extra args: default, type to convert to
    speeds = request.args.get('speed');
    if speeds:
        print(speeds);
        speeds = speeds.split(',')
        comms_mutex.acquire()
        mA.set_power(int(speeds[0]));
        mB.set_power(int(speeds[1]));
        comms_mutex.release()

        ea = mA.get_encoder()
        eb = mB.get_encoder()
        return "%d,%d" % (ea, eb)
    else:
        return render_template('speed.html');

@app.route('/stop')
def stop():
    stop_all()
    return ''

@app.route('/getencoders')
def getencoders():
    global mA, mB
    comms_mutex.acquire()
    ea = mA.get_encoder()
    eb = mB.get_encoder()
    comms_mutex.release()
    if args.debug:
        print('--- get encoders: %d %d\n' % (ea,eb));
    return "%d,%d" % (ea, eb)

"""
" Helper functions
"""
def setspeed(speed, fraction):
    global mA, mB

    comms_mutex.acquire()
    mA.set_power(int(speed[0]*fraction))
    mB.set_power(int(speed[1]*fraction))
    comms_mutex.release()

def stop_all():
    global mA, mB

    comms_mutex.acquire()
    mA.set_power(0)
    mB.set_power(0)
    comms_mutex.release()

"""
" Heartbeat thread, pulse the green LED periodically
"""
def HeartBeat():

    led = ppi.LED(ppi.AD_LED_R)

    while True:
        comms_mutex.acquire()
        led.set_state(1);
        led.set_count(1000);
        comms_mutex.release()
        time.sleep(2);

"""
" Pose estimation thread
"""
def PoseEstimator():
    global x, y, theta

    dt = 1   # sample interval
    W = 0.12   # lateral wheel separation
    comms_mutex.acquire()
    left = mA.get_encoder()
    right = mB.get_encoder()
    comms_mutex.release()
    while True:
        comms_mutex.acquire()
        new_left = mA.get_encoder()
        new_right = mB.get_encoder()
        comms_mutex.release()
        dL = new_left - left
        dR = new_right - right
        left = new_left
        right = new_right

        avg = (dL + dR) / 2
        diff = (dL - dR)
        theta = theta + dt * diff / W;
        x = x + dt * avg * math.cos(theta)
        y = y + dt * avg * math.sin(theta)
        print('Estimated pose %f %f %f (enc=%f %f)' % (x,y,theta,left,right))
        time.sleep(dt);


"""
" Main execution block
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

    mA = ppi.Motor(ppi.AD_MOTOR_A)
    mB = ppi.Motor(ppi.AD_MOTOR_B)
    display = ppi.Display(ppi.AD_DISPLAY_A)
    voltage = ppi.AnalogIn(ppi.AD_ADC_V)

    #initialise serial, and retrieve initial values from the Atmega
    ppi.init()
    mA.get_all()
    mB.get_all()

    # create a comms mutex
    comms_mutex = threading.Lock()

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
    print(camera_state)
    #camera.start_preview()

    if args.awb:
            camera.awb_mode = args.awb
    if args.gain:
            camera.awb_gains = tuple(float(x) for x in args.gain.split(','))
    print('white balance mode is ', camera.awb_mode)

    # open a non-priviliged port
    app.run(host='0.0.0.0', port=8080)
