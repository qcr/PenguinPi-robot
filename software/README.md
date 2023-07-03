# Welcome to EGB439 Advanced Robotics
Included in this repository is the Python, MATLAB and C code used to operate the PenguinPi using the Raspbian Bullseye (64-bit) operating system.

## Folder structure
The main folders are:
  - [python/robot](./python/robot) Python server that runs on the Raspberry Pi and accepts web service requests. There should be no need to touch this for you.
  - [matlab](./matlab) runs on your computer and talks to the Python server running on the robot.
  - [python/client](./python/client) is an unsupported alternative to the MATLAB client.
  - [atmelStudio](./atmelStudio) C code that runs on the Atmel processor (on the i/o board) that connects the Pi to the robot hardware. There should be no need to touch this for you.

## Startup scripts
The scripts that automatically launch on the robot when starting up are:
* [ppweb.py](./python/robot/ppweb.py) - this is the webserver that MATLAB communicates with.
* [GPIOSoftShutdown.py](./python/GPIOSoftShutdown.py) - this allows you to safely shut down the PenguinPi by means of pressing a button.

## Raspberry pi login
* Username: `pi`
* Password: `PenguinPi` - see [instruction to change](https://www.raspberrypi.com/documentation/computers/configuration.html#change-the-default-password)
* You may want to [change your hostname](https://www.howtogeek.com/167195/how-to-change-your-raspberry-pi-or-other-linux-devices-hostname/). If you wish to change this please speak to your tutor.

## Connecting to your Pi
### The easy way
The easiest way is to connect via a screen and keyboard/mouse. 

### Via Wifi
You can also connect to the PI over the network using ssh over WiFi or the Ethernet cable. See the [README-WiFi.md](README-WiFi.md).

### Using an ethernet cable
The ethernet port can be used. It will be assigned an IP address from the DHCP server in your network (e.g. of the QUT DHCP server). The IP will be shown on the Pi's screen.

### Internet access for the PenguinPi using the QUT Enterprise Network
If you can connect to your PenguinPi on the QUT network, but cannot access internet, you have to use the IAClient. You can set it up by using `~/InternetAccessClient_Linux_ARM32v71-RaspberryPi_v4.0.250_QUT/IAClientConfigCmd` and entering your username and password. If the domain is empty, please enter `qut.edu.au`. You can then run `~/InternetAccessClient_Linux_ARM32v71-RaspberryPi_v4.0.250_QUT/IAClient` in a terminal and keep it open (or run in the background by appending ` &` after the command).

### Old instructions, please ignore or use with caution
<details><summary>More details, but use with caution!</summary>
If you need the IP address to stay the same, it is highly recommended you don't use a static IP, but try using a hostname. If you still must have a static IP address, do not edit the `/etc/network/interfaces` file. Since Raspian Jessie, you should use the `/etc/dhcpcd.conf` file to set a static IP address; to do so, follow these [instructions](https://raspberrypi.stackexchange.com/questions/37920/how-do-i-set-up-networking-wifi-static-ip-address/74428#74428) or [these ones](https://www.tomshardware.com/how-to/static-ip-raspberry-pi).

If you want to access the internet over the PenguinPis WiFi hotspot (using the raspberry pi as a WiFi router), you will have to enable IP forwarding. This is already enabled by default.
</details>

## Deploying a PenguinPi
- Flash the HAT: `cd ~/PenguinPi-robot/software/atmelStudio && make` (Steven Bulmer to update instructions)
- Set up the network with the correct MAC for the robot: `cd ~/PenguinPi-robot/software/scripts && sudo update_networking_script.sh`
- Reboot the PenguinPi
- On your local machine, run the `motortest.m` and `cameratest.m` scripts (MATLAB) or the `test_camera_motors.py` script (Python). Note that you will need to update the IP.
