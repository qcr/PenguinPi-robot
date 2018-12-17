#!/bin/bash
#python3 /home/pi/PenguinPi/EGB439/python/server-camera.py &
#python3 /home/pi/PenguinPi/EGB439/python/server-motors.py &
(cd /home/pi/EGB439/python/flask; python3 ppweb.py &)
python3 /home/pi/EGB439/scripts/GPIOSoftShutdown.py &
