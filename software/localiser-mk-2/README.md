# Project Title

A version of the EGB439 localiser that uses compiled binaries for speed and shared memory constructs for safety.

## Getting Started

Intended to run on a raspberry pi.

Install nginx server and PHP

``` 
$ sudo apt update
$ sudo apt install nginx php7.2 php7.2-fpm libfcgi libfcgi-dev build-essential libc-dev libboost-all-dev

```

Install OpenCV 4.2 [from sources](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)

Copy config and web files across

```
$ sudo ./setup.sh
```

Build the localiser

```
mkdir build && cd build
cmake ..
make
``

Start the server

```
$ sudo /etc/init.d/nginx start
```

Check the server is running on port 8080

```
$ sudo netstat -tlpn| grep nginx
``` 

Check php is running

```
$ sudo systemctl status php7.2-fpm
```

TODO finish setting up server

### Prerequisites

Install OpenCV C++ libraries including contrib.

Install requirements: 

```
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```

Get the source:

```
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



### Installing

### Usage

To kill a process running on a port (eg the CGI server):

```
fuser -k 9000/tcp
```


## Authors

* **Jenna Riseley** - *Initial work* - [QUT Bitbucket](https://bitbucket.org/%7B7370add8-cb2c-4301-b546-7bfd62304e14%7D/)

## Acknowledgments

* Thanks to Sandra Rós Hrefnu-Jónsdóttir [github](https://gist.github.com/chronicall)




