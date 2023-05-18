#!/bin/bash

#Check if we are running with sudo
if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

echo "Disabling PenguinPi Webserver Launch on Startup"

rm /etc/init.d/penguin-webserver #Copy penguin web server to init.d

service penguin-webserver stop
systemctl daemon-reload

echo "Removed!"