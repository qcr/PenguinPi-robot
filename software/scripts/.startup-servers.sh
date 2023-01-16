#!/bin/bash
(cd /home/pi/PenguinPi-robot/software/python/robot; python3 ppweb.py &)
python3 /home/pi/PenguinPi-robot/software/scripts/GPIOSoftShutdown.py &
