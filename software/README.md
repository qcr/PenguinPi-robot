# Welcome to EGB439 Advanced Robotics
  - Included in this repository is python, matlab and c code used to operate the
    PenguinPi using the Raspbian Jessie operating system. Please note all code
    stored here will restore you robot to default and changes will be lost
  - if you wish to use a different operating system please speak to your tutor
  
Folders
  - ```matlab``` runs on your computer and talks to the robot
  - ```python``` Python 3 code that runs on the Raspberry Pi
  - ```atmelstudio``` C code that runs on the Atmel processor on the shield that connects the Pi to the robot hardware

## Defaults
### Raspberry pi login
* Username: `pi`
* Password: `PenguinPi` - see instruction to change
  https://www.raspberrypi.org/documentation/linux/usage/users.md

Change your host name https://www.howtogeek.com/167195/how-to-change-your-raspberry-pi-or-other-linux-devices-hostname/

The scripts that Launch on startup are:
* `server-camera.py`
* `server-motors_fixed.py`
* `GPIOSoftShutdown.py`

if you wish to change this please speak to your tutor

## Network setup
The easiest way is to connect a screen and keyboard. Otherwise connect over the network using ssh.

If you are on S9, it will automatically connect to the EGB439 wifi. 

Otherwise, your PenguinPi will make a hotspot if it can't find any wifi networks to connect to. You will know it has created a hotspot by the ip address shown on the lcd screen: `192.168.50.5`.

### Setting up wifi networks to remember
The `wpa_supplicant.conf` file stores the details about the wifi networks to connect to.
Edit this file (using nano, or your prefered text editor):
```shell
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

After some setup values, each network's details will be in a network 'block'. A recommended configuration is below (and included in the files):
```python
country=AU
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

# Your mobile hotspot. You can fall back on this if everything else fails.
network={
    ssid="my_mobile_hotspot"
    psk=9187c952cf11306f12f9e421b6605f531d5b481c3840b823dff4e846fbedb84a
    priority=4
}

# Your home network.
network={
    ssid="my_home_wifi"
    psk=bacfce796f8677ff7356ce26bb55c235c091602f61dcfb53987879826523fde5
    priority=3
}

# The EGB439 network on S9
network={
    ssid="EGB439"
    psk="egb439123"
    priority=2
}

# Your QUT student wireless network access to use elsewhere around campus
network={
    ssid="QUT"
    priority=1
    key_mgmt=WPA-EAP
    pairwise=CCMP
    eap=PEAP
    identity="n0000001"
    password=hash:e5c53515388155f527917a9ee5231538
    phase1="peaplabel=0"
    phase2="auth=MSCHAPV2"
}
```
Note:
* The EGB439 wifi password is stored in plain text (notice the quote marks ""). This is ok for that network, but not for your passwords for your QUT login. The other networks have a lot of hex characters. We will do this for your wifi in the next section.
* The priority allows you to preference one network over the other, in the case there are multiple available. The higher priority networks will be chosen first.


### Hiding your passwords
For personal networks, you can use the `wpa_passphrase` tool.
```shell
wpa_passphrase my_mobile_hotspot my_password
```
will output a network block:
```python
network={
	ssid="my_mobile_hotspot"
	#psk="my_password"
	psk=bebb15a62c377f565f9e17280df9e2b9ece627325be93653c0b98556c9216f49
}
```
Change your password (the `psk=` line) in the `/etc/wpa_supplicant/wpa_supplicant.conf` file to the new value.


For the QUT enterprise network, hash your password with:
```shell
echo -n 'YOUR_REAL_PASSWORD' | iconv -t utf16le | openssl md4
```

This will output 
```shell
(stdin)= e5c53515388155f527917a9ee5231538
```

Ignore the `(stdin)= ` and copy and paste the hashed password into the QUT network block. Ensure it begins with `password=hash:`. The final result will be similar to `password=hash:e5c53515388155f527917a9ee5231538`.

Save your changes to the `wpa_supplicant.conf` file.

Now we need to erase the history so that noone can get your password by looking at the commands you typed.

Find the history file and erase all lines containing your passwords.
```shell
nano ~/.zsh_history
```
Save the file. The passwords are still accessable by pressing the up arrow until you close the terminal.
Reboot your Raspberry Pi (`sudo reboot`).

## Connecting to the hotspot
The ssid of the hotspot is set to `penguinpi:xx:xx:xx` where `xx:xx:xx` will correspond to the end of your MAC address.

If you wish to change this, edit the `/etc/hostapd/hostapd.conf` file and change the following option: `ssid=myNewHotSp0t`.

The default password is `egb439123`. It can be also changed in the `hostapd.conf` file.

The IP address of the robot will be `192.168.50.5`. This will also be the default gateway IP for any device connecting to the hotspot.

## Using ethernet cable
The ethernet port can be used. 

It will be assigned an IP address from the DHCP server. If you need the IP address to stay the same, it is highly recommended you don't use a static IP, but try using a hostname. 

If you still must have a static IP address, do not edit the `/etc/network/interfaces` file. Since Raspian Jessie, you should use the `/etc/dhcpcd.conf` file to set a static IP address.

Follow the instructions here: https://raspberrypi.stackexchange.com/questions/37920/how-do-i-set-up-networking-wifi-static-ip-address/74428#74428 

If you want access the internet over the PenguinPis wifi hotspot (using the raspberry pi as a wifi router, you will have to enable IP forwarding.

Edit the `sysctl.conf` file:
```shell
sudo nano /etc/sysctl.conf
```
look for the line
```shell
# Uncomment the next line to enable packet forwarding for IPv4
#net.ipv4.ip_forward=1
```
and remove the # so it is
```shell
# Uncomment the next line to enable packet forwarding for IPv4
net.ipv4.ip_forward=1
```

## Connecting to the internet 
Run:
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
