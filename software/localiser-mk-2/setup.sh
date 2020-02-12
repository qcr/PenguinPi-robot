#!/bin/bash
set -e

CGI_PORT=9000
WEBAPP_LOCATION=/var/www/EGB439
HOST=127.0.0.1

echo "Setting up server directory structure in $WEBAPP_LOCATION..."
sudo rm -r -f $WEBAPP_LOCATION
sudo mkdir -p $WEBAPP_LOCATION
sudo mkdir -p $WEBAPP_LOCATION/camera/get
sudo mkdir -p $WEBAPP_LOCATION/console

echo "Copying nginx configuration..."
sudo cp server/default /etc/nginx/sites-available/default

echo "Copying web console..."
sudo cp server/console/* $WEBAPP_LOCATION/console

# Copy php settings
sudo cp server/php.ini /etc/php/7.2/fpm/php.ini

#echo "Creating build directory ..."
#mkdir -p build
#echo "Building all targets..."
#cd build 
#cmake ..
#make clean
#make


echo "DONE"
