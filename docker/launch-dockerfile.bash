#!/bin/bash

xhost + local:docker
sudo docker ps
sudo docker rm cola-cont
sudo docker run -it \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --name cola-cont \
    -v ~/DockerVolumes/COLA:/COLA \
    -v ~/DockerVolumes/.vim:/root/.vim \
    colaimage:latest
