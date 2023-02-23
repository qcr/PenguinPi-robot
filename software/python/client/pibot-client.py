#!/usr/bin/env python
import time
import requests
import sys
from threading import Thread

import cv2
import numpy as np
import argparse


class VideoStreamWidget(object):
    def __init__(self, src=0):
        self.capture = cv2.VideoCapture(src)
        self.frame = None
        print('Opened capture, start thread')
        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        # Read the next frame from the stream in a different thread
        while True:
            if self.capture.isOpened():
                (self.status, self.frame) = self.capture.read()
            time.sleep(.01)

    def show_frame(self):
        # Display frames in main program
        cv2.imshow('frame', self.frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.capture.release()
            cv2.destroyAllWindows()
            exit(1)



class PiBot(object):
    def __init__(self, ip='localhost', port=8080):
        self.ip = ip
        self.port = port
        self.endpoint = 'http://{}:{}'.format(self.ip, self.port)
        self.camera = VideoStreamWidget('{}/camera/get'.format(self.endpoint))
        print('Wait for first camera image')
        while self.camera.frame is None:
            time.sleep(0.1)
        print('Got first camera image')

    def setVelocity(self, motor_left=0, motor_right=0, duration=None, acceleration_time=None):
        try:
            params = [
                'value={},{}'.format(motor_left, motor_right)
            ]

            if duration is not None:
                assert duration > 0, 'duration must be positive'
                assert duration < 20, 'duration must be < network timeout (20 seconds)'

                params.append('time={}'.format(duration))
            
                if acceleration_time is not None:
                    assert acceleration_time < duration / 2.0, 'acceleration_time must be < duration/2'
                    params.append('accel={}'.format(acceleration_time))

            print('{}/robot/set/velocity?{}'.format(self.endpoint, '&'.join(params)))
            resp = requests.get('{}/robot/set/velocity?{}'.format(self.endpoint, '&'.join(params)))
            
            if resp.status_code != 200:
                raise Exception(resp.text)

            return resp.json()
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return None

    def setLED(self, number, state):
        try:
            assert number >= 2 and number <= 4, 'invalid LED number'
          
            requests.get('{}/led/set/state?id={}&value={}'.format(self.endpoint, number, 1 if bool(state) else 0))
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def pulseLED(self, number, duration):
        try:
            assert number >= 2 and number <= 4, 'invalid LED number'
            assert duration > 0 and duration <= 0.255, 'invalid duration'
          
            requests.get('{}/led/set/count?id={}&value={}'.format(self.endpoint, number, duration * 1000))
            return True

        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def getDIP(self):
        try:
            resp = requests.get('{}/hat/dip/get'.format(self.endpoint))
            return int(resp.text)
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False
    
    def getButton(self):
        try:
            resp = requests.get('{}/hat/button/get'.format(self.endpoint))
            return int(resp.text)
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False
   
    def setLEDArray(self, value):
        try:
            requests.get('{}/hat/ledarray/set?value={}'.format(self.endpoint, int(value)))
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def printfOLED(self, text, *args):
        try:
            requests.get('{}/hat/screen/print?value={}'.format(self.endpoint, text % args))
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def setScreen(self, screen):
        try:
            requests.get('{}/hat/screen/set?value={}'.format(self.endpoint, int(screen)))
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False
    
    def stop(self):
        try:
            resp = requests.get('{}/robot/stop'.format(self.endpoint), timeout=1)
            return resp.json()
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return None

    def resetPose(self):
        try:
            resp = requests.get('{}/robot/pose/reset'.format(self.endpoint), timeout=5)
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def resetEncoder(self):
        try:
            resp = requests.get('{}/robot/hw/reset'.format(self.endpoint), timeout=5)
            return True
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return False

    def getImage(self):
        return self.camera.frame

    def getVoltage(self):
        try:
            resp = requests.get('{}/battery/get/voltage'.format(self.endpoint), timeout=1)
            return float(resp.text) / 1000
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return None
    
    def getCurrent(self):
        try:
            resp = requests.get('{}/battery/get/current'.format(self.endpoint), timeout=1)
            return float(resp.text) / 1000
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return None
        
    def getEncoders(self):
        try:
            resp = requests.get('{}/robot/get/encoder'.format(self.endpoint), timeout=1)
            left_enc, right_enc = resp.text.split(",")
            return int(left_enc), int(right_enc)
        except requests.exceptions.Timeout as e:
            print('Timed out attempting to communicate with {}:{}'.format(self.ip, self.port), file=sys.stderr)
            return None
 

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='PiBot client')
    parser.add_argument('--ip', type=str, default='localhost', help='IP address of PiBot')
    parser.add_argument('--port', type=int, default=8080, help='Port of PiBot')
    args = parser.parse_args()

    bot = PiBot(args.ip, args.port)
    img = bot.getImage()
    print("image size %d by %d" % (img.shape[0], img.shape[1]))

    bot.setVelocity(-50, -50)
    time.sleep(2)
    bot.stop()

    bot.setVelocity(-50, 50, duration=2)
    bot.setVelocity(100, 100, 7, 3)

    print(f'Voltage: {bot.getVoltage():.2f}V')
    print(f'Current: {bot.getCurrent():.2f}A')
    encs = bot.getEncoders()
    print(f'Encoder left: {encs[0]}, right: {encs[1]}')

    cv2.imshow('image', img)
    cv2.waitKey(0)

    bot.resetEncoder()
