#Welcome to EGB439 Advanced Robotics
  - Included in this repository is python, matlab and c code used to operate the
    PenguinPi using the Raspbian Jessie operating system. Please note all code
    stored here will restore you robot to default and changes will be lost
  - if you wish to use a different operating system please speak to your tutor
  
Folders
  - ```matlab``` runs on your computer and talks to the robot
  - ```python``` Python 3 code that runs on the Raspberry Pi
  - ```atmelstudio``` C code that runs on the Atmel processor on the shield that connects the Pi to the robot hardware

##Defaults
###Raspberry pi login
  username: pi
  Password: raspberry - see instruction to change
  https://www.raspberrypi.org/documentation/linux/usage/users.md

Change your host name https://www.howtogeek.com/167195/how-to-change-your-raspberry-pi-or-other-linux-devices-hostname/

server-camera.py, server-motors_fixed.py and GPIOSoftShutdown.py launch at
start up if you wish to change this please speak to your tutor

##Network setup
This needs to be done on the Pi either by SSH or connecting it to a screen and key board
Open in nano (text editor) and make the following changes - sudo nano /etc/network/interfaces

###ENB439_TpLink_Config
```
allow-hotplug wlan0 
auto wlan0
iface wlan0 inet dhcp
    wpa-ap-scan 1
    wpa-scan-ssid 1
    wpa-ssid "ENB439"
    wpa-proto RSN
    wpa-pairwise TKIP
    wpa-key-mgmt WPA-PSK
    wpa-psk "enb439123"
```

###ENB439-2_Linksys_Config
```
allow-hotplug wlan0 
auto wlan0
iface wlan0 inet dhcp
    wpa-ssid "EGB439-2"
    wpa-psk "egb439123"
```

###ENB439-3 QUT Network with internet access
```
allow-hotplug wlan0 
auto wlan0
iface wlan0 inet dhcp
    wpa-ssid "EGB439-3"
    wpa-psk "egb439123"
```

###Mobile Tethering
```
allow-hotplug wlan0
auto wlan0
iface wlan0 inet dhcp
    wpa-ap-scan 1
    wpa-scan-ssid 1
    wpa-ssid "networkName"
    wpa-proto RSN
    wpa-pairwise CCMP
    wpa-key-mgmt WPA-PSK
    wpa-psk "password"
```

###Tethering
```
auto eth0
iface eth0 inet static
    address 192.168.0.100
    netmask 255.255.255.0

```

##Connecting to the internet 
####(Comment out the fixed IP address address settings and include the line)
```
iface eth0 inet manual
```
After configuring interfaces restart the Pi with sudo reboot and run
```
./InternetAccessClient/IAClientConfigCmd
```
Step through the instructions adding your user name and password when prompted and leaving the other fields unmodified.
```
Authentication server [http://ias-services.qut.edu.au/IAServer/]:
Username: nxxxxxx
Password: ***
Domain [qut.edu.au]:
```
Run the comand. 
```
./InternetAccessClient/IAClient
```
This will notify of a secessful connection when finished note this may take a few minuets on the first connection. When connected press ctrl c to close the program.
This last step will be repeted each time a connection is needed
