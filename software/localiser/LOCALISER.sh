#!/bin/bash

APP=penguinpi
WEB_DIR=/var/www/$APP
RUNFILE_DIR=/var/run/$APP
WEB_USER=www-data

echo "Cleaning up old runfiles..."
rm /var/run/penguinpi/*

echo "Copying nginx configuration..."
cp server/default /etc/nginx/sites-available/default

echo "Copying endpoints..."
cp server/pose/pose_get.php $WEB_DIR/pose/get/index.php
cp server/console/* $WEB_DIR/console

# Copy php settings
PHP_VERSION=$(php -r "echo PHP_VERSION;" | grep --only-matching --perl-regexp "7.\d+")
cp server/php.ini /etc/php/$PHP_VERSION/fpm/php.ini

WEB_GID=$(getent group www-data | awk -F: '{printf "%d", $3}')
SOCKFILE=$RUNFILE_DIR/server.sock

trap "exit" INT TERM ERR
trap "kill 0" EXIT

echo "Starting localiser with web group id " $WEB_GID "..."

./build/localiser $WEB_GID $SOCKFILE > /dev/null 2>&1 & 
localiser_pid=$!
echo "Localiser PID:" $localiser_pid
touch $RUNFILE_DIR/localiser.pid
echo $localiser_pid > $RUNFILE_DIR/localiser.pid
sleep 1 

echo "Now serving!"

wait


