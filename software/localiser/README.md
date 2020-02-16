# Localiser with Nginx + C++

A version of the EGB439 localiser that uses compiled binaries for speed and shared memory constructs for safety.
These instructions will work on both the raspberry pi and a desktop computer.
If used on a desktop computer, localiser will use a static test image.

### Prerequisites

Install OpenCV C++ libraries including contrib.
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


Configure as root:

```
sudo ./configure.sh
```

Or to skip installing dependencies, use the -s flag:

```
sudo ./configure.sh -s
```

Build as regular user:

```
./build.sh
```

### Usage

Start the server and cgi endpoint (do not need to be root):

```
./LOCALISER.sh
```


### Troubleshooting


Check the server is running on port 8080.

Note: to avoid conflict with old localiser, server has been set to port 8008 for now.

```
sudo netstat -tlpn| grep nginx
``` 

Check the http service is running on port 8080 (8008) and the CGI script on 9000

```
nmap 127.0.0.1
```

Check endpoints

``` 
wget <host>:8080/pose/get
wget <host>:8080/camera/get
```

Restart the server:

```
sudo systemctl restart php7.2-fpm  # could be different php version
sudo /etc/init.d/nginx stop
sudo /etc/init.d/nginx start
```




## Authors

* **Jenna Riseley** - *Initial work* - [QUT Bitbucket](https://bitbucket.org/%7B7370add8-cb2c-4301-b546-7bfd62304e14%7D/)

## Acknowledgments

* Thanks to Sandra Rós Hrefnu-Jónsdóttir [github](https://gist.github.com/chronicall)




