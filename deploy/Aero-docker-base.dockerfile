FROM ros:melodic-robot

# install rosdeps
RUN apt update && apt install -y ros-melodic-moveit-core \
                                    ros-melodic-pcl-conversions \
                                    ros-melodic-tf2-geometry-msgs \
                                    ros-melodic-tf2-eigen \
                                    ros-melodic-cv-bridge \
                                    ros-melodic-interactive-markers \
                                    ros-melodic-moveit-visual-tools \
                                    ros-melodic-image-transport \
                                    ros-melodic-moveit-ros-planning \
                                    ros-melodic-geometric-shapes \
                                    ros-melodic-moveit-ros-planning-interface \
                                    ros-melodic-pcl-ros \
                                    vim \
                                    git \
                                    python-pip \
                                    ros-melodic-realsense2-camera \
                                    ros-melodic-rgbd-launch \
                                    ros-melodic-realsense2-description \
                                    ros-melodic-rosserial-python \
                                    ros-melodic-rosserial-arduino \
                                    ros-melodic-rosserial \
                                    python3-yaml \
                                    python3-pip

# install python deps
RUN pip3 install rospkg catkin_pkg numpy modern_robotics && pip install modern_robotics && pip install opencv-contrib-python

# install interbotix sdk
WORKDIR /
RUN bash -c "source /opt/ros/melodic/setup.bash && \
	mkdir -p /interbotix_ws/src && \
	cd /interbotix_ws && \
	catkin_make && \
	cd /interbotix_ws/src && \
	git clone https://github.com/Interbotix/interbotix_ros_arms.git && \
	cd /interbotix_ws/src/interbotix_ros_arms && \
	git checkout melodic && \
	cd /interbotix_ws && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -r -y && \
	catkin_make"
