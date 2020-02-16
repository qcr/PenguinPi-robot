#!/bin/bash

set -e

APP=penguinpi
WEB_DIR=/var/www/$APP
RUNFILE_DIR=/var/run/$APP
WEB_USER=www-data
INSTALL_DIR=/usr/local
USER=$(whoami)

debug=FALSE

echo "Checking permissions..."

if groups $USER | grep -q '\b$WEB_USER\b'; then
    echo "User ${USER} already in group in $WEB_USER"
else
    echo "Adding user ${USER} to $WEB_USER..."
    sudo usermod -a -G $WEB_USER $USER
fi

echo "Setting up server directory structure in $WEB_DIR..."
sudo rm -r -f $WEB_DIR
sudo mkdir -p $WEB_DIR

for f in /var/www /etc/nginx /etc/php $WEB_DIR
do
    echo "Adding $f r/w permissions for $WEB_USER..."
    sudo chgrp -R $WEB_USER $f
    sudo chown -R $WEB_USER: $f
    sudo chmod -R 2775 $f
done

arch=$(dpkg --print-architecture)
echo "Detected architecture $arch"

if [ $arch = "armhf" ]; then 
    echo "Compiling with camera module"
    camera=TRUE
else 
    echo "Compiling without camera module"
    camera=FALSE
fi

mkdir -p build
cd build
cmake -DCAMERA=$camera -DDEBUG=$debug -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR ..
make

echo "Installing into $INSTALL_DIR"
sudo make install

echo "Setting up startup service..."
cd ..
SERVICE_FILE=localiser.service
sudo cp config/$SERVICE_FILE /etc/systemd/system/$SERVICE_FILE
sudo chmod 644 /etc/systemd/system/$SERVICE_FILE

echo "Killing any open TCP ports..."
#fuser -k 631/tcp

echo "Restarting systemd service..."
sudo systemctl daemon-reload
sudo systemctl stop localiser
sudo systemctl start localiser