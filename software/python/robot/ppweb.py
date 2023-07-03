#!/usr/bin/env python

import time
import traceback
import socket
import sys
import signal
import threading
import os
import argparse
import io
import math
import json
import logging
from threading import Condition

import socket
import fcntl
import struct
import array

from netifaces import interfaces, ifaddresses, AF_INET

import penguinPi as ppi
import libcamera
import picamera2

from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput


from flask import Flask, Response, request, render_template, redirect, send_file, jsonify


def on_shutdown(signal, frame):
    print('Received shutdown')
    global is_shutdown
    is_shutdown = True

    global picam2
    picam2.stop_recording()

    # global pose_thread, heartbeat_thread
    if pose_thread is not None:
        print('Waiting for pose_thread to join')
        pose_thread.join()
    # if heartbeat_thread is not None:
    #     print('Waiting for heartheat_thead to join')
    #     heartbeat_thread.join()
    print('Stop ppi')
    global mLeft, mRight
    mLeft.set_velocity(0)
    mRight.set_velocity(0)
    ppi.close()
    print('Bye!')
    sys.exit(0)


signal.signal(signal.SIGINT, on_shutdown)

# TODO:
# All the camera options are broken
# They changed from "simple" single values to triplets (min, max, default)

IM_WIDTH = 320
IM_HEIGHT = 240

# set default values
log_level = logging.INFO
pose_estimator_rate = 5 # Hz
heartbeat_interval = 5  # sec

app = Flask(__name__)

# robot estimated pose
x = 0
y = 0
theta = 0
pose_reset = False

is_shutdown = False

# http://flask.pocoo.org/docs/1.0/quickstart/

# request.args is a MultiDict, dictionary subclass
# http://werkzeug.pocoo.org/docs/0.14/datastructures/#werkzeug.datastructures.ImmutableMultiDict

# button has
#  <input type = "submit" name = A  value = B />
#   B is the displayed label, default is submit
#   A is the name which is "in" the form

count = 0

# USER web page: home page
#
# a bit of everything
@app.route('/', methods = ['POST', 'GET'])
def home():
    if request.method == 'POST':
        if "refresh" in request.form:
            log.debug("refresh")
        elif "test_l" in request.form:
            log.debug("testL")
            mLeft.set_velocity(20)
            time.sleep(2)
            mLeft.set_velocity(0)
        elif "test_r" in request.form:
            log.debug("testR")
            mRight.set_velocity(20)
            time.sleep(2)
            mRight.set_velocity(0)

    # read the robot state
    ea = mLeft.get_encoder()
    eb = mRight.get_encoder()
    v = "%.3f" % (voltage.get_smooth()/1000.0)
    c = "%.f" % current.get_smooth()

    # get info about the Raspberry Pi
    with open('/sys/firmware/devicetree/base/model') as f:
        model = f.read()
    for _ in (True,):
        with open('/etc/os-release') as f:
            line = f.readline()
            distro = line.split('=')[1]
            break
    with open('/proc/version') as f:
        kernel = f.read()

    # render the page
    state = {
            "enc_l": ea,
            "enc_r": eb,
            "volts": v,
            "current": c,
            "pose_x": x,
            "pose_y": y,
            "pose_theta": theta*180.0/math.pi,
            "model": model,
            "distro": distro,
            "kernel": kernel,
            "refresh": 5,
            "camera_revision": camera_revision
            }

    # get refresh
    refresh = request.args.get('refresh')
    if refresh:
            state['refresh'] = refresh

    return render_template('home.html', **state)

sp1 = 0
sp2 = 0

# USER web page for speed control
#
# push stop (STOP, stop) 
# push submit (Set, submit)
# (Left, value)
# (Right, value)
@app.route('/speed', methods = ['POST', 'GET'])
def speed():
    global sp1, sp2
    log.debug('--- set velocity\n')
    if request.method == 'POST':
        if "Set" in request.form:
            sp1 = request.form['Left']
            sp2 = request.form['Right']
            sp1 = int(sp1)
            sp2 = int(sp2)
            mLeft.set_velocity(sp1)
            mRight.set_velocity(sp2)
        elif "STOP" in request.form:
            sp1 = 0
            sp2 = 0
            mLeft.set_velocity(sp1)
            mRight.set_velocity(sp2)
    return render_template('speed.html', speed_l=sp1, speed_r=sp2)

# USER web page to control camera parameters
@app.route('/camera', methods = ['POST', 'GET'])
def camera():
    def update_int(s):
        log.debug('Checking int camera parameter %s: %s -> %s' % (s, request.form[s], camera_state[s]))
        if not s in request.form:
            return
        if int(request.form[s]) != camera_state[s]:
            log.debug('Updating camera parameter %s' % s)
            setattr(camera, s, int(request.form[s]))
            camera_state[s] = request.form[s]

    def update(s):
        log.debug('Checking camera parameter %s: %s -> %s' % (s, request.form[s], camera_state[s]))
        if not s in request.form:
            return
        if request.form[s] != camera_state[s]:
            log.debug('Updating camera parameter %s' % s)
            setattr(camera, s, request.form[s])
            camera_state[s] = request.form[s] 
            
    if request.method == 'POST':
        # NOTE that the request.form multidict may not contain all items
        # in the form
        log.debug('Camera POST' + str(request.form.to_dict(flat=False)))
        # update('Rotation', type='properties', isint=True)
        # update_int('iso')
        # update_int('brightness')
        # update_int('exposure_speed')
        # update_int('shutter_speed')
        # update('AwbMode', type='controls', isint=False)
        # update('meter_mode')
        # update('drc_strength')

        # if "zoom" in request.form:
        #     if request.form["zoom"] != camera_state["zoom"]:
        #         camera_state["zoom"] = request.form["zoom"]
        #         camera.zoom = eval(camera_state["zoom"])

        if "preview" in request.form:
            # preview
            if request.form["preview"] != camera_state["preview"]:
                camera_state["preview"] = request.form["preview"]
                if camera_state["preview"] == "on":
                    picam2.start_preview(True)
                elif camera_state["preview"] == "off":
                    picam2.stop_preview()

    # get refresh
    # refresh = request.args.get('refresh')
    # if refresh:
    #         camera_state['refresh'] = refresh

    return render_template('camera.html', **camera_state)

# USER web page for various other settings
@app.route('/settings', methods = ['POST', 'GET'])
def settings():
    def update(s, typ, func):
        if not s in request.form:
            return
        x = typ(request.form[s])  # convert to numeric
        if x != ppi_state[s]:
            log.debug('Updating ppi parameter {} to {}'.format(s, x))
            func(x)
            ppi_state[s] = x
            
    if request.method == 'POST':
        log.debug('Settings POST' + str(request.form.to_dict(flat=False)))
        update('kvp', int, lambda x: [mLeft.set_kvp(x), mRight.set_kvp(x)])
        update('kvi', int, lambda x: [mLeft.set_kvi(x), mRight.set_kvi(x)])
        update('adcpole', float, lambda x: [voltage.set_pole(x), current.set_pole(x)])
        update('led', lambda x: int(x,16), lambda x: hat.set_ledarray(x))

        update('led2', int, lambda x: led2.set_state(x))
        update('led3', int, lambda x: led3.set_state(x))
        update('led4', int, lambda x: led4.set_state(x))


    # get refresh
    refresh = request.args.get('refresh')
    if refresh:
            ppi_state['refresh'] = refresh

    ppi_state["dip"] = hat.get_dip()
    ppi_state["button"] = hat.get_button()

    return render_template('settings.html', **ppi_state)

#### RESTful web services ###

class InvalidCommand(Exception):
    status_code = 400

    def __init__(self, message, status_code=None, payload=None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.payload = payload

    def to_dict(self):
        rv = dict(self.payload or ())
        rv['message'] = self.message
        return rv

@app.errorhandler(InvalidCommand)
def handle_invalid_command(error):
    response = jsonify(error.to_dict())
    response.status_code = error.status_code
    print('ERROR ', error)
    return response

def gather_img():
    while True:
        global stream_output
        with stream_output.condition:
            stream_output.condition.wait()
            frame = stream_output.frame
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


@app.route('/camera/get')
def picam():
    return Response(gather_img(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/battery/get/voltage')
def voltage():
    return str( voltage.get_smooth() )

@app.route('/battery/get/current')
def current():
    return str( current.get_smooth() )

@app.route('/adc/get/value')
def adcvalue():
    try:
        id = int(request.args.get('id'))
        return str( adc[id].get_value() )
    except:
        raise InvalidCommand('Bad id')


@app.route('/adc/get/smooth')
def adcsmooth():
    try:
        id = int(request.args.get('id'))
        return str( adc[id].get_smooth() )
    except:
        raise InvalidCommand('Bad id')

## HAT related

@app.route('/hat/dip/get')
def dip():
    return str( hat.get_dip() )

@app.route('/hat/button/get')
def button():
    return str( hat.get_button() )

@app.route('/hat/ledarray/set')
def ledarray():
    try:
        value = int(request.args.get('value'),0)
        hat.set_ledarray(value)
    except:
        raise InvalidCommand('Bad value')
    return ''

@app.route('/hat/screen/set')
def hatscreen():
    try:
        value = int(request.args.get('value'))
    except:
        raise InvalidCommand('Bad screen value')
    hat.set_screen(value)
    return ''

@app.route('/hat/screen/print')
def hatscreenprint():
    s = request.args.get('value')
    ppi.puts(s)
    hat.set_screen(1)
    return ''

## LED related

@app.route('/led/set/state')
def ledsetstate():
    try:
        id = int(request.args.get('id'),0)
    except:
        raise InvalidCommand('Bad led id')
    if id < 2 or id > 4:
        raise InvalidCommand('Bad led id')

    try:
        value = int(request.args.get('value'))
    except:
        raise InvalidCommand('Bad led value')

    if value not in [0, 1]:
        raise InvalidCommand('Bad led value')
    leds[id].set_state(value)
    return ''

@app.route('/led/set/count')
def ledsetcount():
    try:
        id = int(request.args.get('id'),0)
    except:
        raise InvalidCommand('Bad led id')
    print(id)
    if id < 2 or id > 4:
        raise InvalidCommand('Bad led id')
    print(id)

    try:
        value = int(request.args.get('value'))
    except:
        raise InvalidCommand('Bad led value')

    if value > 255:
        raise InvalidCommand('Bad led value')
    leds[id].set_count(value)
    return ''

## motor related

@app.route('/motor/get/encoder')
def encget():
    try:
        id = int(request.args.get('id'))
    except:
        raise InvalidCommand('Bad motor id')
    if id not in [0,1]:
        raise InvalidCommand('Bad motor id')
    return str(motors[id].get_encoder())

@app.route('/motor/get/kvi')
def kviget():
    try:
        id = int(request.args.get('id'))
    except:
        raise InvalidCommand('Bad motor id')
    if id not in [0,1]:
        raise InvalidCommand('Bad motor id')
    return str(motors[id].get_kvi())

@app.route('/motor/get/kvp')
def kvpget():
    try:
        id = int(request.args.get('id'))
    except:
        raise InvalidCommand('Bad motor id')
    if id not in [0,1]:
        raise InvalidCommand('Bad motor id')
    return str(motors[id].get_kvp())

@app.route('/motor/set/velocity')
def motorsetvel():
    try:
        id = int(request.args.get('id'))
    except:
        raise InvalidCommand('Bad motor id')
    if id not in [0,1]:
        raise InvalidCommand('Bad motor id')
    try:
        value = int(request.args.get('value'))
    except:
        raise InvalidCommand('Bad motor value')
    motors[id].set_velocity(value)
    return ''
    
@app.route('/motor/set/kvp')
def motorsetkvp():
    try:
        id = int(request.args.get('id'))
    except:
        raise InvalidCommand('Bad motor id')
    if id not in [0,1]:
        raise InvalidCommand('Bad motor id')
    try:
        value = int(request.args.get('value'))
    except:
        raise InvalidCommand('Bad motor value')
    motors[id].set_kvp(value)
    return ''
    
@app.route('/motor/set/kvi')
def motorsetkvi():
    try:
        id = int(request.args.get('id'))
    except:
        raise InvalidCommand('Bad motor id')
    if id not in [0,1]:
        raise InvalidCommand('Bad motor id')
    try:
        value = int(request.args.get('value'))
    except:
        raise InvalidCommand('Bad motor value')
    motors[id].set_kvi(value)
    return ''
    

## robot related (both motors together)

@app.route('/robot/get/encoder')
def getencoders():
    global mLeft, mRight
    ea = mLeft.get_encoder()
    eb = mRight.get_encoder()
    log.debug('--- get encoders: %d %d\n' % (ea,eb))
    return "%d,%d" % (ea, eb)

@app.route('/robot/set/velocity')
def motors():

    # TODO: the trajectory could be done by the pose estimation thread
    dt = 0.05

    # coroutine to do a floating point version of xrange
    def xfrange(start, stop, step):
        i = 0
        while start + i * step < stop:
            yield start + i * step
            i += 1

    # set motor speed using GET side effects
    speeds = request.args.get('value')
    if speeds:
        try:
            speeds = [int(x) for x in speeds.split(',')]
        except:
            raise InvalidCommand('invalid speeds')

        duration = request.args.get('time')
        if duration:
            # a duration was given

            # get duration
            try:
                Ttotal = float(duration)
            except:
                raise InvalidCommand('bad time given')

            # get optional acceleration
            accel = request.args.get('accel')
            if accel:
                try:
                    Taccel = float(accel)
                except:
                    raise InvalidCommand('bad acceleration given')
            else:
                Taccel = 0.0

            # motion time must be greater than twice the acceleration time
            if Ttotal <= Taccel*2:
                raise InvalidCommand('acceleration time too long')

            if Taccel > 0:
                # do the initial speed ramp up
                for t in xfrange(0, Taccel, dt):
                    setspeed(speeds, t/Taccel)
                    time.sleep(dt)

            for t in xfrange(0, Ttotal-Taccel, dt):
                setspeed(speeds, 1.0)
                time.sleep(dt)

            if Taccel > 0:
                # do the final speed ramp down
                for t in xfrange(0, Taccel, dt):
                    setspeed(speeds, (Taccel-t)/Taccel)
                    time.sleep(dt)

            # all stop
            stop_all()

        else:
            # no duration given

            setspeed(speeds)

    return robot_state_json()

@app.route('/robot/stop')
def stop():
    stop_all()
    return robot_state_json()

@app.route('/robot/hw/reset')
def reset_hw():
    multi.clear_data()
    return ''

@app.route('/robot/pose/reset')
def reset_pose():
    global pose_reset

    pose_reset = True
    return ''

"""
" Filters to be used in templates
"   {{ var|hex(n) }}
"   {{ var|float(n) }}
" where n is the number of digits (hex) or decimal places (float)
"""
@app.template_filter("hex")
def hex_filter(v, n):
    return "0x{:0{ndigits}X}".format(v, ndigits=n)

@app.template_filter("float")
def float_filter(v, prec=3):
    return "{:.{ndigits}f}".format(v, ndigits=prec)

"""
" Helper functions
"""
def setspeed(speed, fraction=1.0):
    global mLeft, mRight

    mLeft.set_velocity(int(speed[0]*fraction))
    mRight.set_velocity(int(speed[1]*fraction))

def stop_all():
    global mLeft, mRight

    mLeft.set_velocity(0)
    mRight.set_velocity(0)

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
def HeartBeatThread():
    log.info('Heartbeat thread launched, every %.1f sec' % args.heartbeat_interval)

    led = ppi.LED('AD_LED_R')

    while True:
        led.set_count(200)
        time.sleep(args.heartbeat_interval)
        global is_shutdown
        if is_shutdown:
            break

def IPUpdateThread():
    # magic code from https://stackoverflow.com/questions/7585435/best-way-to-convert-string-to-bytes-in-python-3
    def getHwAddr(ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', bytes(ifname[:15], 'utf-8')))
        return list(info[18:24])


    time.sleep(5)

    eth_ip = None
    wlan_ip = None

    while True:
        log.debug('Checking for IP addresses')
        # set the IP address
        for name in interfaces():
            try:
                if name == 'eth0':
                    ip = ifaddresses(name)[AF_INET][0]['addr']
                    if ip != eth_ip:
                        log.debug('eth0 is %s' % ip)
                        hat.set_ip_eth(ip)
                        eth_ip = ip
                elif name == 'wlan0':
                    ip = ifaddresses(name)[AF_INET][0]['addr']
                    if ip != wlan_ip:
                        log.debug('wlan0 is %s' % ip)
                        hat.set_ip_wlan(ip)
                        wlan_ip = ip
            except KeyError as e:
                log.debug(e)

        # set the MAC address
        # do it every loop because wireless interface can be plugged in/out
        mac = getHwAddr('wlan0')
        hat.set_mac_wlan(mac)

        time.sleep(20)
        global is_shutdown
        if is_shutdown:
            break

"""
" Pose estimation thread
"""
def PoseEstimatorThread():
    global x, y, theta, pose_reset

    log.info('Pose estimator thread launched, running at %.1f Hz' % args.pose_rate)
    dt = 1/args.pose_rate

    W = 0.156  # lateral wheel separation
    wheelDiam = 0.065
    encScale = math.pi * wheelDiam / 384 

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

    # maximum possible encoder change
    #  50Hz servo loop has maximum velocity of 20enc/interval
    #  add a safety factor of 2
    max_step = 50 / pose_estimator_rate * 20 * 2

    while True:
        # check if there's a request to reset the pose estimate
        if pose_reset:
            x = 0
            y = 0
            theta = 0
            pose_reset = False

        # read the encoders 
        new_left = mLeft.get_encoder()
        new_right = mRight.get_encoder()

        # check if bad read value
        if new_left is None or new_right is None:
            log.error('bad encoder read')
            continue

        # compute the difference since last sample and handle 16-bit wrapping
        dL = encoder_difference(new_left, left)
        dR = encoder_difference(new_right, right)

        # handle case where user resets encoder on the PPI board
        if abs(dL) > max_step:
            dL = 0
        if abs(dR) > max_step:
            dR = 0

        # stash the previous values
        left = new_left
        right = new_right

        # compute average and differential wheel motion
        #  this is average and differential wheel speed * dt
        avg = encScale * (dL + dR) / 2
        diff = encScale * (dL - dR)

        # update the state
        #   no need to multiply by dt, it's included already
        theta_old = theta
        theta += diff / W          # update theta
        theta_avg = (theta + theta_old)/2   # average theta over the interval
        x += avg * math.cos(theta_avg)      # update position
        y += avg * math.sin(theta_avg)
        #print('Estimated pose %f %f %f (enc=%f %f)' % (x,y,theta,left,right))
        while theta > 2*math.pi:
            theta -= 2*math.pi
        while theta < -2*math.pi:
            theta += 2*math.pi

        # sleep a bit
        time.sleep(dt)
        global is_shutdown
        if is_shutdown:
            break



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
            pose = True,
            pose_rate = pose_estimator_rate,
            heartbeat = True,
            heartbeat_interval=heartbeat_interval
            )
    parser.add_argument("-d", "--debug", help="log DEBUG messages", 
            dest='log_level',
            action="store_const", const=logging.DEBUG)
    parser.add_argument("-i", "--info", help="log INFO messages", 
            dest='log_level',
            action="store_const", const=logging.INFO)
    parser.add_argument('--log-level',
        dest='log_level',
        type=lambda s: getattr(logging, s.upper()),
        nargs='?',
        help='Set the logging output level to DEBUG|INFO|WARNING|ERROR|CRITICAL')

    parser.add_argument("--no-pose", help="disable pose estimator",
            dest="pose", action="store_false")
    parser.add_argument("--pose-rate", help="set pose estimator rate (Hz)",
            dest="pose_rate", action="store", type=float)
    parser.add_argument("--no-heartbeat", help="disable heart beat",
            dest="heartbeat", action="store_false")
    parser.add_argument("--heartbeat-interval", help="set heartbeat rate (sec)",
            dest="heartbeat_interval", action="store", type=float)

    args = parser.parse_args()

    # everybody uses the Flask logger
    log = logging.getLogger('werkzeug')  # the Flask log channel
    log.setLevel(args.log_level)

    #initialise serial, and retrieve initial values from the Atmega
    ppi.init()

    mLeft = ppi.Motor('AD_MOTOR_L')
    mRight = ppi.Motor('AD_MOTOR_R')
    multi = ppi.Multi('AD_MULTI')
    voltage = ppi.AnalogIn('AD_ADC_V')
    current = ppi.AnalogIn('AD_ADC_C')
    hat = ppi.Hat('AD_HAT')
    led2 = ppi.LED('AD_LED_2')
    led3 = ppi.LED('AD_LED_3')
    led4 = ppi.LED('AD_LED_4')

    mLeft.get_all()
    mRight.get_all()

    leds = [None, None, led2, led3, led4]
    motors = [mLeft, mRight]

    # stash the PenguinPi state, used by the /settings page
    ppi_state = {
            "kvp": mLeft.get_kvp(),
            "kvi": mLeft.get_kvi(),
            "adcpole": voltage.get_pole(),
            "led": hat.get_ledarray(),
            "dip": hat.get_dip(),
            "button": hat.get_button(),
            "led2": led2.get_state(),
            "led3": led3.get_state(),
            "led4": led4.get_state()
            }

    # launch the heartbeat thread
    if args.heartbeat:
        heartbeat_thread = threading.Thread(target=HeartBeatThread, daemon=True)
        heartbeat_thread.start()
    else:
        heartbeat_thread = None

    # launch the pose estimation thread
    if args.pose:
        pose_thread = threading.Thread(target=PoseEstimatorThread, daemon=True)
        pose_thread.start()
    else:
        pose_thread = None

    # launch the IP update thread
    ipupdate_thread = threading.Thread(target=IPUpdateThread, daemon=True)
    ipupdate_thread.start()

    # Get the camera up and running
    try:
        picam2 = picamera2.Picamera2()
        import subprocess
        log_out = subprocess.check_output(['libcamera-hello', '--list-cameras', '-n']).decode('utf-8')
        camera_revision = 0
        for line in log_out.split('\n'):
            if line.startswith('0'):
                camera_revision = line.split(' ')[2]
    except:
        log.fatal("Couldn't open camera connection -- is it being used by another process?")
        exit(1)

    video_config = picam2.create_video_configuration(
        main={"size": (IM_WIDTH, IM_HEIGHT)},
        raw={"size": (1640, 1232)}
    )
    video_config["transform"] = libcamera.Transform(hflip=1, vflip=1)
    picam2.configure(video_config)
    try:
        picam2.set_controls({"FrameRate": 30})
        picam2.set_controls({"AfMode": libcamera.controls.AfModeEnum.Manual, "LensPosition": 0.5}) #Set fixed focal length for Rpiv3 Camera (1/0.5 or 2m)
    except:
        log.info("Couldn't set Autofocus mode. Assuming PiCameraV2")
    stream_output = StreamingOutput()

    picam2.start_recording(JpegEncoder(), FileOutput(stream_output))

    metadata = picam2.capture_metadata()
    controls = {c: metadata[c] for c in ["ExposureTime", "AnalogueGain", "ColourGains"]}
    log.debug(controls)
    log.debug('white balance mode is ' + str(picam2.camera_controls['AwbMode']))

    # fire up the webserver on a non-priviliged port
    app.jinja_env.lstrip_blocks = True
    app.jinja_env.trim_blocks = True
    app.jinja_env.line_statement_prefix = '#'

    app.run(host='0.0.0.0', port=8080, threaded=True)

    while not is_shutdown:
        pass
