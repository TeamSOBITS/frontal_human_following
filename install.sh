#!/bin/bash

echo "╔══╣ Setup: frontal_human_following (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download ROS packages
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-tf \
    ros-$ROS_DISTRO-nlopt

# Install NLopt
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
cmake . && make && sudo make install

sudo apt-get update
sudo apt-get install -y \
    libnlopt0

# Go back to previous directory
cd ${DIR}


echo "╚══╣ Setup: frontal_human_following (FINISHED) ╠══╝"
