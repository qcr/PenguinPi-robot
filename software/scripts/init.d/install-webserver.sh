#!/bin/bash

#Check if we are running with sudo
if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

echo "Enabling PenguinPi Webserver Launch on Startup"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) #Find dir of this script
START_FILE="$SCRIPT_DIR/../.startup-servers.sh"

echo "Path to startup script: $START_FILE"

while read line; do
   echo ${line//FILE_PATH/$START_FILE}

done < $SCRIPT_DIR/penguin-webserver > $SCRIPT_DIR/penguin-webserver_pathed
chmod +x $SCRIPT_DIR/penguin-webserver_pathed

cp $SCRIPT_DIR/penguin-webserver_pathed /etc/init.d/penguin-webserver #Copy penguin web server to init.d

update-rc.d penguin-webserver defaults  #update
service penguin-webserver start #start

rm $SCRIPT_DIR/penguin-webserver_pathed

echo "Enabled"