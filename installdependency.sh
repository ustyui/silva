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

set -e

if [ "$JETSON_BOARD" != "AGX Xavier" ] ; then 
  # some fresh dependencies
  sudo -H apt-get install python-pip -y
  # install respeaker pre-dependencies
  sudo -H apt-get install libportaudio2 -y
  sudo -H apt-get install python-pyaudio -y
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

  set -e
  
else "Setup is cancelled because of unknown issues"
fi

echo "ALL PRE-CONFIGURATION HAS BEEN SET."
