#!/bin/bash
set -e

echo "Installing packages..."
pkg_list="nginx net-tools libfcgi libfcgi-dev build-essential libc-dev libboost-all-dev php7.2 php-fpm php-mysql cmake git \
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

username=$USER
echo "Checking permissions..."

if getent group www-data | grep -q "\b${username}\b"; then
    echo "User in www-data"
else
    echo "Adding user to www-data..."
    sudo usermod -a -G www-data $username
fi

for f in /var/www /etc/nginx /etc/php
do
    echo "Adding $f r/w permissions for www-data..."
    sudo chgrp -R www-data $f
    sudo chown -R www-data: $f
    sudo chmod -R 2775 $f
done

