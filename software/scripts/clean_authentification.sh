#!/bin/bash

# remove github token
gh auth logout
# remove IAClient
sudo rm -rf /home/pi/.IAClient/
# remove github global user account and email
rm -rf /home/pi/.gitconfig
