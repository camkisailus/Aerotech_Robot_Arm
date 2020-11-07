#!/bin/bash

# find build context
deploy_dir=$(dirname $0)
build_context=$(dirname $deploy_dir)

# build base image
docker build -f $deploy_dir/Aero-docker-base.dockerfile -t aero-docker-base $build_context
base_build_status="$?"

# build stack
docker build -f $deploy_dir/Dockerfile -t aero_test $build_context
stack_build_status="$?"

# if successful, run the container - else print what failed
if [[ $base_build_status == "0" && $stack_build_status == "0" ]]; then
    printf "Build successful\n"
    docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb -v /dev/ttyDXL:/dev/ttyDXL -v /dev/ttyACM0:/dev/ttyACM0 aero_test bash
else
    if [[ $base_build_status == "0" ]]; then
        printf "ERROR: Stack failed to build\n"
    else
        printf "ERROR: Base image failed to build"
    fi
fi