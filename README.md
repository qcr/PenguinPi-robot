PenguinPi Robot
===============

# Synopsis

This site describes an open-source low-cost robot for educational purposes: hardware and software.

![PenguinPi robot](software/doc/P1080377.JPG)

The robot is used for teaching mobile robotics, robotic vision within QUT's EGB439 Advanced Robotics unit.

Key features of the robot are:

* low cost, less than AUD 500
* Raspberry Pi 3B computer with a color camera
* an i/o board with embedded processor that interfaces with motors and provides a simple UI (20x4 OLED display, 4xpushbuttons and various other LEDs)
* a MATLAB-based API, MATLAB code running on a host computer communicates over WiFi with a server running on the robot
* ability to run code onboard written in Python or C++

Example of MATLAB API usage:
```matlab
robot = PiBot('192.168.3.4')
robot.setVelocity(20, -20); % set left & right wheel speeds to +20 and -20 respectively
robot.stop(); % stop the robot moving
batteryVoltage = pb.getVoltage(); % get battery voltage
pb.setLed(2, true);  % turn on the green LED
pb.printfOLED('hello world'); % write string to the OLED display
im = pb.getImage(); % get RGB image from the camera
```

![PenguinPi robot](software/doc/montage-annotated.png)

# Mechanical design

The robot has two independently controllable wheels driven by geared DC brushmotors, and a single castor.  The frame is built from laser cut acrylic.

An [animation of the assembly](https://youtu.be/HddVRaQIhIY).

