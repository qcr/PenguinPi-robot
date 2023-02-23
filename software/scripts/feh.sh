#!/bin/bash

# uncomment to reactivate localiser - disabled for 2nd half of sem
# (cd /home/pi/PenguinPi-robot/software/python/localiser; python3 localiser.py) & 

# sleep 20s;
export DISPLAY=:0.0;  
(cd /home/pi/PenguinPi-robot/software/python/localiser; feh current.png --reload 0.1)

