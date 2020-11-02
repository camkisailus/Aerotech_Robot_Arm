FROM aero-docker-base
COPY . /ws
WORKDIR /ws

RUN apt update && \
    rosdep install --from-paths src --ignore-src -r -y

ENTRYPOINT [ "/ws/deploy/entrypoint.sh" ]
