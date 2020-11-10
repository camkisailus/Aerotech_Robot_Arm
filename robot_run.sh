#!/bin/bash
while getopts "l" opt; do
    case ${opt} in
      l)
 		 export LOCAL=true
    esac
done
if [[ -n "${LOCAL}" ]]; then
	roslaunch core main.launch
else
	source /interbotix_ws/devel/setup.bash
	source /ws/devel/setup.bash --extend
	export ROS_PACKAGE_PATH="/interbotix_ws/src:/ws/src:/opt/ros/melodic/share"
	roslaunch core main.launch
fi
