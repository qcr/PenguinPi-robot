#!/bin/bash
set -e

CGI_PORT=9000
WEBAPP_LOCATION=/var/www/EGB439
HOST=127.0.0.1

echo "Setting up server directory structure in $WEBAPP_LOCATION..."
sudo mkdir -p $WEBAPP_LOCATION
sudo rm -r $WEBAPP_LOCATION/*

echo "Copying nginx configuration..."
sudo cp server/default /etc/nginx/sites-available/default

# Copy php settings
sudo cp server/php.ini /etc/php/7.2/fpm/php.ini
sudo cp server/index.php $WEBAPP_LOCATION

echo "Creating build directory ..."
mkdir -p build
echo "Building all targets..."
cd build 
cmake ..
make clean
make

echo "Starting server..."
sudo /etc/init.d/nginx stop
sudo /etc/init.d/nginx start 

echo "Starting localiser with dummy image..."
./localiser ../testing/Flask-desktop-testbed/camv2img.jpg &

echo "Killing and restarting service on $CGI_PORT..." 
fuser -k $CGI_PORT/tcp
sleep 3
cgi-fcgi -start -connect $HOST:$CGI_PORT $0/build/cgi_app



echo "DONE"