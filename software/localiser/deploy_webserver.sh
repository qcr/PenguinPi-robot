#!/bin/bash

APP=penguinpi
WEB_DIR=/var/www/$APP

mkdir -p $WEB_DIR/pose/get
mkdir -p $WEB_DIR/console
mkdir -p $WEB_DIR/camera/get

echo "Copying nginx configuration..."
cp server/default /etc/nginx/sites-available/default

echo "Copying endpoints..."
cp server/pose/pose_get.php $WEB_DIR/pose/get/index.php
cp server/console/* $WEB_DIR/console
cp server/arena.jpg $WEB_DIR/camera/get/arena.jpg # default image for testing on desktop

echo "Copying php settings..."
PHP_VERSION=$(php -r "echo PHP_VERSION;" | grep --only-matching --perl-regexp "7.\d+")
cp server/php.ini /etc/php/$PHP_VERSION/fpm/php.ini