#!/bin/bash

sudo cp $(dirname "$0")/10-interbotix-udev.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger
