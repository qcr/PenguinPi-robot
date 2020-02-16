#!/bin/bash

set -e

APP=penguinpi
WEB_DIR=/var/www/$APP
RUNFILE_DIR=/var/run/$APP
WEB_USER=www-data
INSTALL_DIR=/usr/local

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

install_deps=0
debug=FALSE

case $key in
    -n|--no-skip-deps)
    #EXTENSION="$2"
    install_deps=1
    shift 
    ;;
    -d|--debug)
    #EXTENSION="$2"
    debug=TRUE
    shift 
    ;;
    --default)
    DEFAULT=YES
    shift # past argument
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

if [ $install_deps ]; then
    echo "Installing packages..."
    pkg_list="nginx net-tools libfcgi libfcgi-dev build-essential libc-dev libboost-all-dev php php-fpm php-mysql cmake git \
    libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev \
    libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev libcanberra-gtk-module libcanberra-gtk3-module"

    for package in $pkg_list
    do
        if sudo apt -y install $package >/dev/null 2> /dev/null; then
            echo $package
        else
            echo "$package **** FAILED ****"
        fi
    done
fi

echo "Checking permissions..."

if groups $SUDO_USER | grep -q '\b$WEB_USER\b'; then
    echo "User ${SUDO_USER} already in group in $WEB_USER"
else
    echo "Adding user ${SUDO_USER} to $WEB_USER..."
    sudo usermod -a -G $WEB_USER $SUDO_USER
fi

for f in /var/www /etc/nginx /etc/php
do
    echo "Adding $f r/w permissions for $WEB_USER..."
    sudo chgrp -R $WEB_USER $f
    sudo chown -R $WEB_USER: $f
    sudo chmod -R 2775 $f
done

echo "Setting up server directory structure in $WEB_DIR..."
rm -r -f $WEB_DIR
mkdir -p $WEB_DIR/camera/get
mkdir -p $WEB_DIR/pose/get
mkdir -p $WEB_DIR/console
sudo chown $WEB_USER:$WEB_USER $WEB_DIR
chmod -R g+rwx $WEB_DIR

echo "Copying nginx configuration..."
cp server/default /etc/nginx/sites-available/default

echo "Copying endpoints..."
cp server/pose/pose_get.php $WEB_DIR/pose/get/index.php
cp server/console/* $WEB_DIR/console

echo "Copying php settings..."
PHP_VERSION=$(php -r "echo PHP_VERSION;" | grep --only-matching --perl-regexp "7.\d+")
cp server/php.ini /etc/php/$PHP_VERSION/fpm/php.ini

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
sudo cp $SERVICE_FILE /etc/systemd/system/$SERVICE_FILE
sudo chmod 644 /etc/systemd/system/$SERVICE_FILE

echo "Killing any open TCP ports..."
fuser -k 631/tcp

echo "Restarting systemd service..."
sudo systemctl daemon-reload
sudo systemctl start localiser