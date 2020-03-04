#!/bin/bash

set -e

APP=penguinpi
WEB_DIR=/var/www/$APP
WEB_USER=www-data
BIN_INSTALL_DIR=/usr/local
SCRIPT_INSTALL_DIR=/opt/penguinpi
USER=$(whoami)
SOURCES_DIR=$PWD
PROFILE=FALSE
debug=FALSE
BUILD=TRUE
SET_PERMISSIONS=TRUE

arch=$(dpkg --print-architecture)
echo "Detected architecture $arch"

if [ $arch = "armhf" ]; then 
    echo "Detected arm architecture"
else 
    echo "Detected other architecture, assuming build on desktop PC"
fi

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -s|--skip-build)
    BUILD=FALSE
    shift # past argument
    ;;
    -p|--skip-permissions)
    SET_PERMISSIONS=FALSE
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


if [ $SET_PERMISSIONS = "TRUE" ]; then 

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

fi 

echo "Setting up server directory structure in $WEB_DIR..."
sudo rm -r -f $WEB_DIR
sudo mkdir -p $WEB_DIR

echo "Copying php settings..."
PHP_VERSION=$(php -r "echo PHP_VERSION;" | grep --only-matching --perl-regexp "7.\d+")
cp config/php.ini /etc/php/$PHP_VERSION/fpm/php.ini

echo "Copying nginx configuration..."
REPLACE_PHP="s/<PHP_VERSION>/${PHP_VERSION}/g"
cp config/default /etc/nginx/sites-available/default
sed -i $REPLACE_PHP /etc/nginx/sites-available/default

echo "Copying application..."
cp -r www/* $WEB_DIR

for f in /var/www /etc/nginx /etc/php $WEB_DIR $SCRIPT_INSTALL_DIR
do
    echo "Adding $f r/w permissions for $WEB_USER..."
    sudo chgrp -R $WEB_USER $f
    sudo chown -R $WEB_USER: $f
    sudo chmod -R 2775 $f
done

echo "Restarting nginx.."
sudo service nginx reload








