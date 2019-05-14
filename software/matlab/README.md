The code here allows you to control the robot and access its camera from the MATLAB environment.  You need to have the file
```PiBot.m``` in the current directory or on your MATLAB path.  The Raspberry Pi needs to be running the Python server code (```server-camera.py``` and ```server-motor.py```) in the folder above.

We start by connecting to the remote Raspberry Pi
```
pb = PiBot('192.168.1.141');
```
which requires the IP address of the RPi on the network.  This might be the one assigned by the WiFi access point in the 
lab or a hardwired address 192.168.0.100 for the wired ethernet port. 
The result is an object that is a connector to the remote RPi.

To obtain an image is simply
```
img = pb.getImageFromCamera();
```
which is an RGB image with uint8 pixel data.  Typically acquiring an images takes 100-200ms.

To access the motor encoders
```
ticks = pb.getMotorTicks();
```
which returns a 2-vector containing the integer tick counts for motor A and motor B.  These are in units of degrees of wheel rotation and wrap around at 2^15/2.1333

To set the speed of the motors is simply
```
pb.setMotorSpeeds(-50,-50);
```
where the two arguments are the speed of motor A and B respectively.  WHAT ARE THE UNITS HERE?

To stop the motors
```
pb.setMotorSpeeds(0,0);
```

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
