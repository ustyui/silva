# Initial Setup & Configuration

We recommend that developers obtain the basic equipment described below (or similar) and use a default configuration.

## Basic equipment

The equipment below is highly recommended:

* A development computer:
  * with Ubuntu Linux 18.04 or later
* A command center device:
  * with Ubuntu Linux 18.04 or later
* A korg nanokontrol2 midi controller
* A joystick game controller
  * PS4 game controller with wireless USB adapter
  * xbox360 standard game controller

## Robot configuration

### Hardware
silva currently support multiple android driver protocols, users can also create their own protocols in order to drive their robots.

Before running silva core, please make sure that the driver protocol can correctly drive the actuators of your robot.

> **Tip** [*mbed complier*](https://os.mbed.com) is required for robot microcontroler configuration when using ibuki.

To configure ibuki:

1. Open mbed complier for obtaining complied ibuki's driver program.
1. [Basic Configuration](/en/config.md) explains how to perform basic configuration.
1. [Parameter Configuration](/en/parameters.md) explains how you can find and modify individual parameters.

## A Quick Start on Ibuki Gazebo Simulator
User can use the following commands to run silva basic functions to give ibuki a quick start:

### System Boot
```
roslaunch ibuki_gazebo ibuki_silva.launch
```

### Idle Motion
```
roslaunch silva_core idle_ibuki.launch
```

### Parameters
Use `rosparam set` to change the parameters on ibuki gazebo.
You can try:

| parameter name | fucntion | correspondent value|
|-----|-----------------|----|
| WEIGHT_HW_ADJUST   | if you are using hardware devices(midi, joystick) controlling the weights. | 0: use software 1: use hardware|
| WEIGHT_IDLE   | The weight of idle node.    | 0 to 1|
| WEIGHT_SLAVE  | The weight of slave node.   | 0 to 1|

Use:
```
rosparam set /WEIGHT_HW_ADJUST 0
```
to activate software weight control.
```
rosparam set /WEIGHT_IDLE 0.3
```
to change the weight of idle motion. After this manipulation, you can observe that in gazebo simulator the robot is randomly moving the upper-body.

### GUI
```
rosrun silva_core silva.py
```
for Graphical user interface. In this interface you can use the slider bar to change the joint value in slave node.





to change the idle weight.
