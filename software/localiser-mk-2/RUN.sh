#!/bin/bash

CGI_PORT=9000

# if [ $# == 0 ]
# then
#     LOCALHOST="172.19.232.12"
# else
#     LOCALHOST=$1
# fi
# echo $LOCALHOST

sudo systemctl restart php7.2-fpm

sudo /etc/init.d/nginx stop
fuser -k 9000/tcp
sudo /etc/init.d/nginx start

sudo cgi-fcgi -start -connect 127.0.0.1:9000 build/cgi_app

sudo ./build/localiser
# Get PID of the localiser 
echo $! > server/console/localiser_PID.txt

