# MATLAB interface to PenguinPi

The code here allows you to control the robot and access its camera from the MATLAB environment.  You need to have the file
```PiBot.m``` in the current directory or on your MATLAB path.  The Raspberry Pi needs to be running the Python server code [ppweb.py](../python/robot/ppweb.py) (this happens automatically at startup).

## Establishing a connection
We start by connecting to the remote Raspberry Pi
```
pb = PiBot('192.168.1.141');
```
which requires the IP address of the RPi on the network.  This might be the one assigned by the WiFi access point in the 
lab or a hardwired address 192.168.0.100 for the wired ethernet port. 
The result is an object that is a connector to the remote RPi.

## Obtaining camera images
To obtain an image is simply
```
img = pb.getImageFromCamera();
```
which is an RGB image with uint8 pixel data.  Typically acquiring an images takes 100-200ms.

### IP camera package
To obtain images, you need the [MATLAB Support Package for IP Cameras](https://au.mathworks.com/help/supportpkg/ipcamera/index.html). Typically installation is straightforward - MATLAB will simply ask you to install the package when you try to use the `ipcam()` function and you do not have the support package installed. In case you get errors that "There was a problem downloading the support package", please use the [Download Support Packages](https://au.mathworks.com/support/install/support-software-downloader.html) application on a different machine and follow the instructions.

## Motor control
### Access the motor encoders
```
ticks = pb.getMotorTicks();
```
which returns a 2-vector containing the integer tick counts for motor A and motor B.  These are in units of degrees of wheel rotation and wrap around at 2^15/2.1333

### Set the speed
```
pb.setMotorSpeeds(-50,-50);
```
where the two arguments are the speed of motor A and B respectively.  WHAT ARE THE UNITS HERE?

### Stop motors
```
pb.setMotorSpeeds(0,0);
```

## LED control
The PenguinPi board has a 2-digit 7-segment display. You can write a hex number (0-255) to that by
```
pb.setDisplayValue(32);
```
which will display as 20 in hex.  You can change the display base to unsigned decimal by
```
pb.setDisplayMode('u');
```
and now the display will show 32 in decimal.  The decimal point is lit to indicate decimal mode.
Unsigned decimal numbers are in the range (0-99).
There is also a signed decimal display mode for numbers in the range (-9 to +9)
```
pb.setDisplayMode('d');
```
