########################################
# Install script for ROS Indigo
# Author: Richard Yang
# Last Updated: Jan 31, 2016
########################################
#!/bin/bash

echo "##########Updating sources##########"
echo "##########(1/8)##########"
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

echo "##########Setting up keys#########"
echo "##########(2/8)##########"
apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

echo "##########Updating packages##########"
echo "##########(3/8)##########"
apt-get update

echo "##########Installing ROS-Indigo##########"
echo "##########(4/8)##########"
apt-get install ros-indigo-desktop-full

echo "##########Initializing rosdep#########"
echo "##########(5/8)##########"
rosdep init

echo "##########Updating rosdep##########"
echo "##########(6/8)##########"
rosdep update

echo "##########Setting up ROS environment#########"
echo "##########(7/8)##########"
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "##########Installing rosinstall#########"
echo "##########(8/8)##########"
apt-get install python-rosinstall

echo "##########Robo-Ops ROS set up complete!##########"
echo "##########To finish set up, run: $ rosdep update ##########"
