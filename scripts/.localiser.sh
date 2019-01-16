#!/bin/bash
python3 /home/pi/egb439/python/robot/old-servers/server-camera.py &

#(cd /home/pi/egb439/python/localiser; python3 ppweb.py)
# # python3 /home/pi/PenguinPi/EGB439/python/server-motors.py
while true; do
     python3 /home/pi/egb439/scripts/image_loc.py
done
#
