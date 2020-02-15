#!/bin/bash

CGI_PORT=9000
WEBAPP_LOCATION=/var/www/EGB439
HOST=127.0.0.1

echo "Killing any cgi service on port 9000 ..."
fuser -k 9000/tcp

echo "Setting up server directory structure in $WEBAPP_LOCATION..."
rm -r -f $WEBAPP_LOCATION
mkdir -p $WEBAPP_LOCATION
mkdir -p $WEBAPP_LOCATION/camera/get
mkdir -p $WEBAPP_LOCATION/console

touch $WEBAPP_LOCATION/console/localiser_PID.txt

echo "Copying nginx configuration..."
cp server/default /etc/nginx/sites-available/default

echo "Copying web console..."
cp server/console/* $WEBAPP_LOCATION/console

# Copy php settings
PHP_VERSION=$(php -r "echo PHP_VERSION;" | grep --only-matching --perl-regexp "7.\d+")
cp server/php.ini /etc/php/$PHP_VERSION/fpm/php.ini

# if [ $# == 0 ]
# then
#     LOCALHOST="172.19.232.12"
# else
#     LOCALHOST=$1
# fi
# echo $LOCALHOST

trap "exit" INT TERM ERR
trap "kill 0" EXIT

echo "Starting localiser..."

./build/localiser > /dev/null 2>&1 & 
localiser_pid=$!
echo "Localiser PID:" $localiser_pid
echo $localiser_pid > /var/www/EGB439/console/localiser_PID.txt
sleep 1 

echo "Restarting cgi script..."
cgi-fcgi -start -connect 127.0.0.1:9000 build/cgi_app

echo "Now serving!"

wait
