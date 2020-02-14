#!/bin/bash

set -e

APP=penguinpi
WEB_DIR=/var/www/$APP
RUNFILE_DIR=/var/run/$APP
WEB_USER=www-data
INSTALL_DIR=/usr/local
USER=$(whoami)
SOURCES_DIR=$PWD
PROFILE=TRUE
debug=FALSE

arch=$(dpkg --print-architecture)
echo "Detected architecture $arch"

echo "Checking permissions..."

for group in $WEB_USER video
do
    if groups $SUDO_USER | grep -q '\b$group\b'; then
        echo "User ${SUDO_USER} already in group in $group"
    else
        echo "Adding user ${SUDO_USER} to $group..."
        sudo usermod -a -G $group $SUDO_USER
    fi
done

echo "Adding ${WEB_USER} to video group..."
sudo usermod -a -G video $WEB_USER

echo "Setting up server directory structure in $WEB_DIR..."
sudo rm -r -f $WEB_DIR
sudo mkdir -p $WEB_DIR

echo "Creating runfile directory in " $RUNFILE_DIR
mkdir -p $RUNFILE_DIR

echo "Copying php settings..."
PHP_VERSION=$(php -r "echo PHP_VERSION;" | grep --only-matching --perl-regexp "7.\d+")
cp config/php.ini /etc/php/$PHP_VERSION/fpm/php.ini

echo "Copying nginx configuration..."
REPLACE_PHP="s/<PHP_VERSION>/${PHP_VERSION}/g"
sed -i $REPLACE_PHP config/default
cp config/default /etc/nginx/sites-available/default

echo "Copying application..."
cp -r www/* $WEB_DIR

for f in /var/www /etc/nginx /etc/php $WEB_DIR
do
    echo "Adding $f r/w permissions for $WEB_USER..."
    sudo chgrp -R $WEB_USER $f
    sudo chown -R $WEB_USER: $f
    sudo chmod -R 2775 $f
done

echo "Restarting nginx.."
sudo service nginx reload

if [ $arch = "armhf" ]; then 
    echo "Compiling with camera module"
    camera=TRUE
else 
    echo "Compiling without camera module"
    camera=FALSE
fi

BUILD_DIR=$(mktemp -d -t)
echo "Created temporary directory " $BUILD_DIR
cp -r ./src $BUILD_DIR
cp -r ./include $BUILD_DIR
cp CMakeLists.txt $BUILD_DIR
cd $BUILD_DIR
mkdir build 
cd build
cmake -DCAMERA=$camera -DDEBUG=$debug -DPROFILE=$PROFILE -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR ..
make

echo "Installing into $INSTALL_DIR"
sudo make install

echo "Going back into " $SOURCES_DIR
cd $SOURCES_DIR

echo "Setting up startup service..."
SERVICE_FILE=localiser.service
sudo cp config/$SERVICE_FILE /etc/systemd/system/$SERVICE_FILE
sudo chmod 644 /etc/systemd/system/$SERVICE_FILE

echo "Starting systemd service..."
sudo systemctl daemon-reload
sudo systemctl stop localiser
sudo systemctl start localiser
sudo systemctl enable localiser

