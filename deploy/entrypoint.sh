#!/bin/bash

source /opt/ros/melodic/setup.bash

cd /ws && catkin_make 

exec $@
