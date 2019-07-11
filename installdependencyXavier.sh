#!/bin/bash
# Install the dependencies of silva package
# Copyright (c) 2018-19 ustyui
# MIT License

INSTALL_DIR=$PWD
cd ..
cd ..
ROS_WORKSPACE=$PWD
cd $INSTALL_DIR
source $ROS_WORKSPACE/devel/setup.bash

# is this AGX Xavier?
source scripts/jetson_variables.sh
#Print Jetson version
echo "$JETSON_DESCRIPTION"

# Error out if something is wrong
set -e

# Add armhf architecture and java package for AITalk
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install libc6:armhf libncurses5:armhf libstdc++6:armhf  
sudo apt-get install nkf
sudo apt-get install ant

if [ "$JETSON_BOARD" == "AGX Xavier" ] ; then 
  sudo -H apt-get install python-pip -y
  # install respeaker pre-dependencies
  sudo -H apt-get install libportaudio2 -y
  sudo -H apt-get install python-pyaudio
  # change mod for all of the python scripts
  cd $INSTALL_DIR/silva_beta/src
  sudo chmod u+x *.py
  cd $INSTALL_DIR/ibuki_extra/src
  sudo chmod u+x *.py
  # install the dependencies of respeaker
  cd $ROS_WORKSPACE
  rosdep install --from-paths src -i -r -n -y
  cd $INSTALL_DIR/respeaker_ros/cfg
  sudo chmod u+x *.cfg
  cd $ROS_WORKSPACE
  catkin_make
  # restart bashrc
  source ~/.bashrc
  source $ROS_WORKSPACE/devel/setup.bash
  roscd respeaker_ros
  sudo cp -f $(rospack find respeaker_ros)/config/60-respeaker.rules /etc/udev/rules.d/60-respeaker.rules
  sudo systemctl restart udev
  sudo pip install -r requirements.txt
  
else "This is not AGX Xavier!!"
fi

echo "ALL PRE-CONFIGURATION HAS BEEN SET."
