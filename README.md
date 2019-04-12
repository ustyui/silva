# silva
silva android system framework based on ROS

master branch:[![Build Status](https://travis-ci.org/ustyui/silva.svg?branch=master)](https://travis-ci.org/ustyui/silva)

## Usage
The silva android operation framework generates android motion by using a developed framework by merging multiple motion generator models.
## Dependencies
- [ROS](http://www.ros.org)
- Catkin workspace
- Pandas

## Building 
```
cd ~/some_path
git clone https://github.com/ustyui/silva.git
cd ~/catkin_ws
ln -s ~/some_path/silva ~/catkin_ws/src/silva
cd ~/catkin_ws/src/silva/src/
./installdependencyXavier.sh
catkin_make
```
## How to Use
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
