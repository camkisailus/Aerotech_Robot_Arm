FROM aero-docker-base

COPY . /ws	
WORKDIR /ws

RUN bash -c " apt update && \
    rosdep install --from-paths src --ignore-src -r -y && \
	source /opt/ros/melodic/setup.bash && \
	catkin_make && \
	echo \"source /opt/ros/melodic/setup.bash\" >> ~/.bashrc "

ENTRYPOINT [ "/ws/deploy/entrypoint.sh" ]
