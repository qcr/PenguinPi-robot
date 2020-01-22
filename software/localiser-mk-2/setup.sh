#!/bin/bash

CGI_PORT=9000

# Copy web endpoints across
sudo rm -r /var/www/EGB439
sudo cp -r server/EGB439 /var/www

# Copy the nginx conf
sudo cp server/default /etc/nginx/sites-available/default

# Copy php settings
sudo cp server/php.ini /etc/php/7.2/fpm/php.ini

# Start server 
sudo /etc/init.d/nginx start 

# Compile the cgi, kill anything running on the CGI port and restart the service 
fuser -k $CGI_PORT/tcp > /dev/null
cd server/EGB439
make 
cgi-fcgi -start -connect 127.0.0.1:$CGI_PORT ./bin/cgi_app

echo "DONE: Setup and started nginx server, compiled and started CGI app"
