#!/bin/bash
set -e

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

echo "Adding user ${USER} to www-data..."
sudo usermod -a -G www-data $USER

for f in /var/www /etc/nginx /etc/php
do
    echo "Adding $f r/w permissions for www-data..."
    sudo chgrp -R www-data $f
    sudo chown -R www-data: $f
    sudo chmod -R 2775 $f
done

