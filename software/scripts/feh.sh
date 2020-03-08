#!/bin/bash
# python3 /home/pi/egb439/python/robot/old-servers/server-camera.py &

# uncomment to reactivate localiser - disabled for 2nd half of sem

# (cd /home/pi/egb439/python/localiser; python3 localiser.py) & 

# sleep 20s;
export DISPLAY=:0.0;  
(cd /home/pi/penguinpi-robot/software/python/localiser; feh current.png --reload 0.1)

