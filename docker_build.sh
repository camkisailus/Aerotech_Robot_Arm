#!/bin/bash

# build base image and image
docker build -f deploy/Aero-docker-base.dockerfile -t aero-docker-base .
docker build -f deploy/Dockerfile -t aero_test .

# open new shell in container
docker run -it aero_test bash
