#!/bin/bash
set -e

APP=penguinpi
WEB_DIR=/var/www/$APP
RUNFILE_DIR=/var/run/$APP
WEB_USER=www-data

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -s|--skip-pkg-install)
    #EXTENSION="$2"
    skip_deps=1
    shift # past argument
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

if [ $skip_deps ]; then
    echo "Skipping deps"
else
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

echo "Making directory in /var/run for PID and socket files..."
runfile_dir=/var/run/penguinpi
sudo rm -r -f $RUNFILE_DIR
sudo mkdir $RUNFILE_DIR
sudo chown $WEB_USER:$WEB_USER $RUNFILE_DIR
chmod -R g+rwx $RUNFILE_DIR

echo "Done! Run ./build.sh (without sudo)"




