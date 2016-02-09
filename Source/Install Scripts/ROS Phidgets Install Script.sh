########################################
# Install script for ROS Phidgets Package
# Author: Richard Yang
# Last Updated: Jan 31, 2016
########################################
#!/bin/bash
./phidget-driver.sh
cd ~/ros_pkg
bzr branch lp:phidgets-ros-pkg
mv phidgets-ros-pkg phidgets
sudo apt-get install ros-indigo-audio-common
sudo apt-get install ros-indigo-navigation
sed -i -- 's/opencv2/cv_bridge/g' ~/ros_pkg/phidgets/manifest.xml
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_pkg" >> ~/.bashrc
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_pkg
rosmake --pre-clean phidgets
echo "##########source ~/.bashrc##########"
