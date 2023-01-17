#!/bin/bash
(cd /home/pi/PenguinPi-robot/software/python/robot; python ppweb.py &)
python /home/pi/PenguinPi-robot/software/scripts/GPIOSoftShutdown.py &
