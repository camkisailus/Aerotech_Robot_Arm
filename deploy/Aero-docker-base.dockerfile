FROM ros:melodic-robot

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
                                    vim
