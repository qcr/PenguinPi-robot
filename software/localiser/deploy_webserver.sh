#!/bin/bash

APP=penguinpi
WEB_DIR=/var/www/$APP
WEB_USER=www-data

mkdir -p $WEB_DIR/pose/get
mkdir -p $WEB_DIR/console
mkdir -p $WEB_DIR/camera/get

echo "Copying nginx configuration..."
cp config/default /etc/nginx/sites-available/default

echo "Copying php settings..."
PHP_VERSION=$(php -r "echo PHP_VERSION;" | grep --only-matching --perl-regexp "7.\d+")
cp config/php.ini /etc/php/$PHP_VERSION/fpm/php.ini

echo "Copying application..."
rm -r $WEB_DIR/*
cp -r server/* $WEB_DIR

echo "Adding r/w permissions for $WEB_USER..."
sudo chgrp -R $WEB_USER $WEB_DIR
sudo chown -R $WEB_USER: $WEB_DIR
sudo chmod -R 2775 $WEB_DIR

echo "Restarting nginx.."
sudo service nginx reload

