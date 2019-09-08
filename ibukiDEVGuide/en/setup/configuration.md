# Robot Configuration before starting silva

Users need to prepare several configurations when connecting your robot to silva.
We try to keep these procedures simple, so that users could focus on designing higher level of AI without focusing on debugging.

To make the procedure easy to understood, we use a opensource humanoid platform Darwin to illustrate how the configuration work.
The demonstration of the robot body will be shown in gazebo simulator, as a large-community robot simulator.

## 1. Create a .map init file
By adapting silva to different robot interfaces a .map file is needed to tell silva that what should be the robot's **initial pose** is.<br>
It means that users use this file to initialzie input values which can make the robot show a human-liked static posture.<br>

As mechanically, every robot has its **default morphology**, which includes:
* Designed to minimize the potential energy,
* Compliant to the gravity,
* Designed human-liked,
* Randomly Positioned,
* etc.

In Darwin the robot **default morphology** shows like this:
<p align="center">
  <img width="500" src="/ibukiDEVGuide/assets/images/Darwin/default.png">
</p>

In silva, as the system is used to make human-liked motion behaviors,
in the beginning, this file is used for silva to initialize the robot.

The format of .map file is as follows: (You can check the file in silva_core/params/darwin.map)<br>
**Note:** Please name the file as yourrobotname.map
```
------config----------
robotname   yourrobotname
driveunits  numberofdofs
------defaults--------
1   j_shoulder_r    0
2   j_shoulder_l    0
3   j_pan   0
4   j_tilt  0
5   j_high_arm_l    1
6   j_low_arm_l 0
...
12  j_gripper_r -1
```
**robotname** is the name of your robot, it is suggested to use all lower case letters.
The robotname variable will help silva to identify the robot and find other config files for the system.<br>
**driveunits** is the number of degrees of freedom(DoFs) of the robot.
In Darwin robot, there are 12 DoFs for silva to drive.<br>
**defaults**
The format of defaults are written as below:
```
<serialnumber></tab><jointname></tab><initpos>
```
The values of initpos are decided to ensure the robot whole body is posed at a user defined posture.
Each time the system starts, the posture of the robot will be controlled according to the values of initpos.

The **init file** will finally make the robot posture as below (with the help of silva):
<p align="center">
  <img width="500" src="/ibukiDEVGuide/assets/images/Darwin/init.png">
</p>

## 2. Create a .map limit file
In silva, the joint position of the robot will be limited by controlling the output of silva.
The format of .map limit file is as follows: (Also check it in silva_core/params/darwin_limit.map)<br>
**Note:** Please name the file as limit_yourrobotname.map
```
------config----------
robotname   yourrobotname
driveunits  numberofdofs
------limits--------
1   j_shoulder_r    -1  1
2   j_shoulder_l    -1  1
3   j_pan   -1  1
4   j_tilt  -1  1
5   j_high_arm_l    -2  2
6   j_low_arm_l -2  2
...
12  j_gripper_r -1  1
```
The only difference with .map init file is that after every jointname there are two values.
The first one is the lower limit and the second one is the upper limit.

The format of limits are written as below:
```
<serialnumber></tab><jointname></tab><lowerlimit></tab><upperlimit>
```

## 3. Create a parameter .yaml file
A .yaml file is needed to save some important **static parameters** for a robot. It may includes:
* Control rates,
* IP/ports, serial port names,
* Control sequences, clocks,
* etc.

The format of .map file is as follows: (You can check the file in silva_core/params/darwin.yaml)<br>
**Note:** Please name the file as yourrobotname.yaml
```
Config:
  robotname: darwin
  driveunits: 12
  stacks: 4

Rates:
  default: 30
  broadcast: 2
  mixer: 40
  gui: 50
  idle: 30
  reflex: 30
  slave: 30
  auto: 30
  actuator: 50
  feedback: 25

SequenceOfJoints:
  neck: 0
  arml: 1
  armr: 2

```
A .yaml file must have Config table ,Rates table and SequenceOfJoints table, as necessary **static parameters** for silva.
For other **static parameters**, developers can add or delete for actual usage, as different robot need to be programmed into a different **adapter interface**.<bn>

**Config**: Similar in .map **init file**, need to note that **stack** stands for
when controlling the robot, how many DoFs in a group is for one **acutator interface node**.
For example if the stack number is 4, in each **actuator interface node**, the control DoFs are 4.

The number of **SequenceOfJoints** equals to `driveunits/stacks`.
It means the corresponding serialnumber for each **acutator interface node**.
For the usage, see **message processing**.

**Rates** is the publish rate for the nodes of the silva system.
For the node descriptions, see **system architecture**<bn>
**default**: If there is no correspondent node name in the Rate table, the node will adapte default value as the rate.
**broadcast**: The broadcast message rate.


