# Connecting to the PenguinPi via WiFi
* If you are on S9, the PenguinPi will automatically connect to the EGB439 wifi. You can then connect to the IP shown on the LCD screen.
* The Pi will also connect to other WiFi networks that you can specify - see instructions below.
* Otherwise, your PenguinPi will make a hotspot if it can't find any wifi networks to connect to. You will know it has created a hotspot by the ip address shown on the lcd screen: `192.168.50.5`. The MAC address will be part of the network name - please refer to more details below.

### Connecting to the hotspot
The ssid of the hotspot is set to `penguinpi:xx:xx:xx` where `xx:xx:xx` will correspond to the end of your MAC address. The MAC address is shown on the Pi's LCD screen.

If you wish to change this, edit the `/etc/hostapd/hostapd.conf` file and change the following option: `ssid=myNewHotSp0t`.

### Setting up WiFi networks to remember
If you want to connect to e.g. your home network, you can use the `wpa_supplicant.conf` file that stores the details about the WiFi networks to connect to.
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


### Hiding your WiFi passwords
#### Personal networks
For personal networks (e.g. your home network), you can use the `wpa_passphrase` tool. Please add a space in front of the `wpa_passphrase` command so it is not being saved to your bash history.
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

#### QUT network
For the QUT enterprise network, you can hash your password. Please put a space in front of `echo` so that the command is not being saved in the bash history.
```shell
 echo -n 'YOUR_REAL_PASSWORD' | iconv -t utf16le | openssl md4
```

This will output 
```shell
(stdin)= e5c53515388155f527917a9ee5231538
```

Ignore the `(stdin)= ` and copy and paste the hashed password into the QUT network block. Ensure it begins with `password=hash:`. The final result will be similar to `password=hash:e5c53515388155f527917a9ee5231538`.

Save your changes to the `wpa_supplicant.conf` file.

#### Removing passwords from your history
If you forgot to put a space in front of the command, you need to erase the history so that noone can get your password by looking at the commands you typed.

Find the history file and erase all lines containing your passwords.
```shell
nano ~/.zsh_history
```
Save the file. The passwords are still accessable by pressing the up arrow until you close the terminal.
The default password is `egb439123`. It can be also changed in the `/etc/hostapd/hostapd.conf` file.

The IP address of the robot will be `192.168.50.5`. This will also be the default gateway IP for any device connecting to the hotspot.
