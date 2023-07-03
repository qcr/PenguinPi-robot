#!/bin/bash

# remove github token
gh auth logout
# remove IAClient
sudo rm -rf /home/pi/.IAClient/
# remove github global user account and email
rm -rf /home/pi/.gitconfig

echo 'Make sure to remove any network details in /etc/wpa_supplicant/wpa_supplicant.conf'
