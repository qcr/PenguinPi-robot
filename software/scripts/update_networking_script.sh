#!/bin/bash

# Run this script with sudo!!!
if [ "$(id -u)" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

########################################################################################
# Hostapd Configuration
echo "Setting up hotspot configuration in hostapd"

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

########################################################################################
# Replace wpa_supplicant.conf with the template file
sudo mv /etc/wpa_supplicant/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf.backup # Backup old file
sudo cp wpa_supplicant_default.conf /etc/wpa_supplicant/wpa_supplicant.conf

########################################################################################
echo "Done! Reboot is required"
exit 0
