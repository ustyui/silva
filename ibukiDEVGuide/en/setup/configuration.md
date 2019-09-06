# Robot Configuration before starting silva

Users need to prepare several configurations when connecting your robot to silva.
We try to keep these procedures simple, so that users could focus on designing higher level of AI without focusing on debugging.

To make the procedure easy to understood, we use a opensource humanoid platform Darwin to illustrate how the configuration work.
The demonstration of the robot body will be shown in gazebo simulator, as a large-community robot simulator.

## Create a .map init file
By adapting silva to different robot interfaces a .map file is needed to tell silva that what should be the robot's initial pose is.<br>
It means that users use this file to initialzie input values which can make the robot show a human-liked static posture.<br>

As mechanically, every robot has its default morphology, which includes:
* Designed to minimize the potential energy,
* Compliant to the gravity,
* Designed human-liked,
* Randomly Positioned,
* etc.

In silva, as the system is used to make human-liked motion behaviors,
in the beginning, this file is used for silva to initialize the robot.

The format of .map file is as follows: (You can check the file in silva_core/params/darwin.map)
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


