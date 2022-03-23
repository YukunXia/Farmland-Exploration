#!/bin/bash

# These commands were found after endless hours of searching the web. Works on ubuntu 20.04. Not sure about other versions

xhost + local:docker # Give docker gui permissions

sudo cp ~/.Xauthority /root

sudo apt-get install -y \
	libxcb-xinerama0 \
	libqt5gui5 \
	libxcb-randr0 \
	libxcb-xtest0-dev \
	libxcb-xinerama0-dev \
	libxcb-shape0-dev \
	libxkbcommon-x11-dev
	
