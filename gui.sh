#!/bin/bash

xhost +local:root

docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb -v /dev/ttyDXL:/dev/ttyDXL -v /dev/ttyACM0:/dev/ttyACM0 --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" aero_test bash
