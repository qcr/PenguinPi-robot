# Project Title

A version of the EGB439 localiser that uses compiled binaries for speed and shared memory constructs for safety.

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

Copy config and web files across and build the server programs

```
$ sudo ./setup.sh
```

### Usage

Start the server

```
$ sudo /etc/init.d/nginx stop
$ sudo /etc/init.d/nginx start 
```

Check the server is running on port 8080

```
$ sudo netstat -tlpn| grep nginx
``` 

Run the localiser with a test image

```
./localiser ../testing/Flask-desktop-testbed/camv2img.jpg &
```

Kill any services running on port 9000 and start the CGI script

```
fuser -k 9000/tcp
cgi-fcgi -start -connect $HOST:$CGI_PORT $0/build/cgi_app
```


## Authors

* **Jenna Riseley** - *Initial work* - [QUT Bitbucket](https://bitbucket.org/%7B7370add8-cb2c-4301-b546-7bfd62304e14%7D/)

## Acknowledgments

* Thanks to Sandra Rós Hrefnu-Jónsdóttir [github](https://gist.github.com/chronicall)




