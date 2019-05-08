<p align="center">
  <img width="191" src="/ibukiDEVGuide/assets/images/logo.png">
</p>

**Silva** is an android operation framework generates **lifelike android motion** by using a developed framework by merging multiple motion generator models.

|ROS Version|Kinetic|Melodic|
|  :---:|  :---:|  :---:|
|Build Status|[![Build Status](https://travis-ci.org/ustyui/silva.svg?branch=master)](https://travis-ci.org/ustyui/silva)|[![Build Status](https://travis-ci.org/ustyui/silva.svg?branch=master)](https://travis-ci.org/ustyui/silva)|  

It is authored by [**Shiqi Yu**](https://shiqi-yu.com/), **Naoki Ise**, **Yifei Wang** and [**Yoshihiro Nakata**](http://yoshihiro-nakata.sakura.ne.jp/). Currently, it is being maintained by [**Shiqi Yu**](https://shiqi-yu.com/). To use **silva**, A ROS environment under linux is necessary. For detail handbooks, please refer to [**ibukiDEVGuide**](/ibukiDEVGuide).

<p align="center">
    <img src="ibukiDEVGuide/assets/images/motion_upper_body.gif", width="320">
    <br>
    <sup>The mobile android <a href="https://eng.irl.sys.es.osaka-u.ac.jp/projects/ibuki" target="_blank">ibuki</a> showing conversation postures using silva.</sup>
</p>

## Excepted Features
The beta version of silva has been released, the whole project is still under daily development. Silva is designed to have the following expected features:

* Compatibily: Given the robot with limited diversity, the framework can enable the lifelike motion charactistics of the robot.
* Tolerance: For the faults and errors from the inputs or the framework itself, the behavior of the robot will be constrainted in common 
* Synthesis: The framework can adapt as many kinds of existing motion models as it can.
* Easy Modularize: Users can easy design the modules and add them to the existing frameworks.

## Configuration
**OS**: Ubuntu 18.04 on Nvidia AGX Xaiver suggested

## Contents
1. [Installation](#installation)
2. [Quick Start](#quick-start)
3. [Citation](#citation)

## Installation
```
cd ~/some_path
git clone https://github.com/ustyui/silva.git
cd ~/catkin_ws
ln -s ~/some_path/silva ~/catkin_ws/src/silva
cd ~/catkin_ws/src/silva/
./installdependencyXavier.sh
catkin_make
```
## Quick Start
### System Boot (Robot Side)
```
roslaunch silva_beta beta.launch
```

### Idle Motion (Robot Side)
```
roslaunch silva_beta tanh.launch
```

### Midi Controller Interface (Command PC Side)
```
roslaunch silva_beta controlpanel.launch
```

### GUI Usage (On Command PC Side)
```
rosrun silva_beta debug_gui.py
```

### Humanoid Simulation Model Mapping 
see docs in /doc/data_input_format.csv

To run the example, run
```
rosrun silva_beta HSM_csv.py lookaround
```
You can monitor the output using
```
rostopic echo /silva/slave_local/operation
```
### Messages
Silva has different type of messages based on its framework.

You can check the message type by using
```
rostopic list /silva
```

## Citation
Please cite the following papers in your publications if it helps your research:

