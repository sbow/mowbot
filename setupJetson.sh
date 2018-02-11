#!/bin/bash
# Shaun Bowman
# initial jetson tx1 setup

#update apt-get
sudo apt-get update

# setup directories
mkdir ~/git

# update vim
git clone https://github.com/sbow/vim.git ~/git/vim
git clone https://github.com/VundleVim/Vundle.vim.git ~/.vim/bundle/Vundle.vim

# get mowbot project
git clone https://github.com/sbow/mowbot.git ~/git/mowbot

# install ros - tx1 repository from jetsonhacks
git clone https://github.com/jetsonhacks/installROSTX1.git ~/git/installROSTX1
sudo ~/git/installROSTX1/updateRepositories.sh
sudo ~/git/installROSTX1/installROS.sh
#sudo .~/git/installROSTX1/setupCatkinWorkspace.sh
sudo rosdep fix-permissions
rosdep update
#source /opt/ros/kinetic/setup.bash
sudo apt-get install ros-kinetic-tutorials
sudo apt-get install ros-kinetic-rqt
sudo apt-get install ros-kenetic-rqt-common-plugins

# get rplidar ROS package
git clone https://github.com/robopeak/rplidar_ros.git ~/git/rplidar_ros

# get libi2c-dev for accelerometer
sudo apt-get install libi2c-dev
sudo apt-get install build-essential qt5-default qtcreator
git clone https://github.com/jetsonhacks/RTIMULib.git ~/git/RTIMULib

# get stdr_simulator & turtle_teleop (robot simulator & keyboard interface)
# note: some errors in apt-get teleop, but seems to work alight
# more setup needed to use teleop, create package http://wiki.ros.org/ROS/Tutorials/CreatingPackage
# setup package http://wiki.ros.org/ROS/Tutorials/CreatingPackage
sudo apt-get install ros-kinetic-stdr-simulator
sudo apt-get install ros-kinetic-turtlebot-teleop
