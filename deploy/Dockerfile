FROM aero-docker-base

COPY . /ws	
WORKDIR /ws

RUN bash -c " ls / && apt update && \
    rosdep install --from-paths src --ignore-src -r -y && \
	source /opt/ros/melodic/setup.bash && \
	catkin_make"

ENTRYPOINT [ "/ws/deploy/entrypoint.sh" ]
