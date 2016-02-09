########################################
# Install script for Phidget Drivers
# Author: Richard Yang
# Last Updated: Jan 31, 2016
########################################
#!/bin/bash
mkdir ~/ros_pkg
cd ~/ros_pkg
mkdir phidget_drivers
cd phidget_drivers
wget http://www.phidgets.com/downloads/libraries/libphidget.tar.gz
tar -xvf libphidget.tar.gz
rm -rf libphidget.tar.gz
cd libphidget-*
./configure
make
sudo make install
sudo cp udev/99-phidgets.rules /etc/udev/rules.d
