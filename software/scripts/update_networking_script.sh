#!/bin/bash

# Run this script with sudo!!!
echo $EUID
if [ "$EUID" -ne 0 ]
then 
  echo "Please run as root"
  exit 1
fi

# Using as a guide:
# http://www.raspberryconnect.com/network/item/330-raspberry-pi-auto-wifi-hotspot-switch-internet

echo "Installing required packages\n"
sudo apt-get update

# hostapd hotspot client and dnsmasq lightweight dns server need to be installed.
sudo apt-get install hostapd dnsmasq

# The installers will have set up the programme so they run when the pi is started. 
# For this setup they only need to be started if the home router is not found. 
# So automatic startup needs to be disabled. 
# This is done with the following commands:
sudo systemctl disable hostapd
sudo systemctl disable dnsmasq


########################################################################################
# Hostapd Configuration
echo "Setting up hotspot configuration in hostapd\n"

MAC=$(cat /sys/class/net/wlan0/address)
sudo echo "
#2.4GHz setup wifi 80211 b,g,n
interface=wlan0
driver=nl80211
ssid=penguinpi:${MAC: -8}
hw_mode=g
channel=8
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=egb439123
wpa_key_mgmt=WPA-PSK
wpa_pairwise=CCMP TKIP
rsn_pairwise=CCMP

#80211n -
country_code=AU
ieee80211n=1
ieee80211d=1
" > /etc/hostapd/hostapd.conf


sudo echo '
# Defaults for hostapd initscript
#
# See /usr/share/doc/hostapd/README.Debian for information about alternative
# methods of managing hostapd.
#
# Uncomment and set DAEMON_CONF to the absolute path of a hostapd configuration
# file and hostapd will be started during system boot. An example configuration
# file can be found at /usr/share/doc/hostapd/examples/hostapd.conf.gz
#
DAEMON_CONF="/etc/hostapd/hostapd.conf"

# Additional daemon options to be appended to hostapd command:-
#       -d   show more debug messages (-dd for even more)
#       -K   include key data in debug messages
#       -t   include timestamps in some debug messages
#
# Note that -B (daemon mode) and -P (pidfile) options are automatically
# configured by the init.d script and must not be added to DAEMON_OPTS.
#
#DAEMON_OPTS=""
' > /etc/default/hostapd

########################################################################################
# DNSmasq configuration
echo "Setting up hotspot configuration in dnsmasq\n"

# append the following to the bottom of the conf file
sudo echo '
#AutoHotspot config
interface=wlan0
bind-dynamic 
server=8.8.8.8
domain-needed
bogus-priv
dhcp-range=192.168.50.150,192.168.50.200,255.255.255.0,12h
' >> /etc/dnsmasq.conf


########################################################################################
# Revert the interfaces file back to default
echo "Reverting interfaces file to default. (Backup to /etc/network/interfaces.backup)\n"

sudo cp /etc/network/interfaces /etc/network/interfaces.backup
sudo echo "
# interfaces(5) file used by ifup(8) and ifdown(8)
# Please note that this file is written to be used with dhcpcd
# For static IP, consult /etc/dhcpcd.conf and 'man dhcpcd.conf'
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d
" > /etc/network/interfaces


########################################################################################
# Setup the service to run the autohotspot script
echo "Setting up wifi configuration script to auto detect wireless status\n"

sudo echo '
[Unit]
Description=Automatically generates an internet Hotspot when a valid ssid is not in range
After=multi-user.target
[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/bin/autohotspotN
[Install]
WantedBy=multi-user.target
' > /etc/systemd/system/autohotspot.service


########################################################################################
# Copy the autohotspot script over
sudo cp autohotspotN /usr/bin/autohotspotN

# Ensure the file is executatble
sudo chmod +x /usr/bin/autohotspotN

# Enable the service
sudo systemctl enable autohotspot.service


########################################################################################
# Replace wpa_supplicant.conf with the template file
sudo mv /etc/wpa_supplicant/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf.backup # Backup old file
sudo cp wpa_supplicant_default.conf /etc/wpa_supplicant/wpa_supplicant.conf


########################################################################################
echo "Done! Reboot is required"
exit 0