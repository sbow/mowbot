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
sudo ~/git/installROSTX1/installROS.sh
#sudo .~/git/installROSTX1/setupCatkinWorkspace.sh
sudo ~/git/installROSTX1/updateRepositories.sh
sudo rosdep fix-permissions
rosdep update
#source /opt/ros/kinetic/setup.bash
sudo apt-get install ros-kinetic-tutorials
sudo apt-get install ros-kinetic-rqt
sudo apt-get install ros-kenetic-rqt-common-plugins

