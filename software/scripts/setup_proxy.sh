#!/bin/bash
curl -x http://wproxy.qut.edu.au:3128 -I http://www.github.com -s -f -o /dev/null
if [ $? -eq 0 ]; then
    echo "Proxy is up"
    git config --global http.proxy http://wproxy.qut.edu.au:3128
    git config --global https.proxy https://wproxy.qut.edu.au:3128
else
    echo "Proxy is down"
    git config --global --unset http.proxy
    git config --global --unset https.proxy
fi
