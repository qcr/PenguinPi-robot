#!/bin/bash
SHUTTER_SPEED=$(cat shutterspeed)
echo "Starting raspivid with shutter speed ${SHUTTER_SPEED}"
raspivid -sa -$SHUTTER_SPEED -h 480 -w 640 -l -o tcp://0.0.0.0:3333
echo $! > /var/www/penguinpi/videostream.pid