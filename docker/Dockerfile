# This docker image is based on the official noetic ros docker file
# https://github.com/osrf/docker_images/blob/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/noetic/ubuntu/focal/ros-base/Dockerfile
FROM ros:noetic-ros-core-focal

# install bootstrap tools
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-wstool \
    python3-rosinstall-generator \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# Allows debconf to run properly. see https://stackoverflow.com/questions/51023312/docker-having-issues-installing-apt-utils
ARG DEBIAN_FRONTEND=noninteractive

# install ros packages, setup environment
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

RUN sudo sh -c 'echo \
    "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros1-latest.list' \
    && sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
    --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt update

RUN apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ackermann-steering-controller \
    ros-noetic-ros-control \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

# Add ros setup to bashrc and source it
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc \
    && apt-get update

# Setup color for PS1
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc

# Fix GUI issues with docker
ARG QT_GRAPHICSSYSTEM="native"

# Install commitizen (which requires npm, which requires Node.js)
# see https://github.com/nodesource/distributions/blob/master/README.md
# see https://www.npmjs.com/package/commitizen
RUN apt-get update \
    && curl -sL https://deb.nodesource.com/setup_15.x | sudo -E bash - \
    && sudo apt-get install -y nodejs \
    && npm install -g commitizen

# Install vs code
# https://linuxize.com/post/how-to-install-visual-studio-code-on-ubuntu-20-04/
RUN apt-get update \
    && sudo apt install -y software-properties-common apt-transport-https wget \
    && wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add - \
    && sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" \
    && sudo apt install -y code

# install helpful software tools
RUN apt-get update && apt-get install -y \
    vim \
    tmux \
    python3 \
    pylint \
    wget \
    git-lfs \
    python-is-python3
