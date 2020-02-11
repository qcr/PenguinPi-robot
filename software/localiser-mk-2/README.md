# Localiser with Nginx + C++

A version of the EGB439 localiser that uses compiled binaries for speed and shared memory constructs for safety.
These instructions are for building on the target raspberry pi.
Future project: set up toolchain for cross-compilation.

### Prerequisites

Install nginx server and PHP

``` 
$ sudo apt update
$ sudo apt install nginx libfcgi libfcgi-dev build-essential libc-dev libboost-all-dev
```

Install OpenCV C++ libraries including contrib.

```
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```

Get the source:

```
cd ~/Downloads
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```

Configure and build. 

```
cd opencv-<version>
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
make -j7
```

Install as system library

```
sudo make install
```

Install raspicam library, must be installed as system library and on cmake path:

```
cd ~/Downloads
git clone https://github.com/cedricve/raspicam .
cd raspicam
mkdir build
cd build
cmake ..
make
sudo make install
```


### Installing

Build the server programs. On rpi you must do this as root.

```
cd <localiser mk 2 location>
mkdir build
cd build
sudo cmake ..
sudo make
sudo make install
```

Copy web server files across.

```
$ sudo ./setup.sh
```

### Usage

Start the server and cgi endpoint

```
sudo ./RUN.sh
```

Check the server is running on port 8080
Note: to avoid conflict with old localiser, server has been set to port 8008 for now.

```
sudo netstat -tlpn| grep nginx
``` 

Check endpoints

``` 
wget <host>:8080/pose/get
wget <host>:8080/camera/get
```


## Authors

* **Jenna Riseley** - *Initial work* - [QUT Bitbucket](https://bitbucket.org/%7B7370add8-cb2c-4301-b546-7bfd62304e14%7D/)

## Acknowledgments

* Thanks to Sandra Rós Hrefnu-Jónsdóttir [github](https://gist.github.com/chronicall)




