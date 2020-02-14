#!/bin/bash
set -e

arch=$(dpkg --print-architecture)
echo "Detected architecture $arch"

if [ $arch = "armhf" ]; then 
    echo "Compiling with camera module"
    camera="TRUE"
else 
    echo "Compiling without camera module"
    camera="FALSE"
fi

mkdir -p build
cd build
cmake -DCAMERA=$CAMERA ..
make