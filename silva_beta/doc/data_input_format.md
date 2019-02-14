# Humanoid Simulation Model Input Format

## Coordination Frame
There are 47 degrees of freedom and _ main joints in ibuki-v2. 

The coordination frame of each joint is set as follows. 

**Note: The axis which face you is Y axis, and all of the directions of the rotations obeys the following coordination frame.**

**the direction which is the same with the coordinate frame is marked as +1(positive direction). Otherwise, it is marked as -1(negative direction)**

**For example, when ibuki faces you, the neck looks the right side of the image, the direction is +1**

<img src="https://github.com/ustyui/ibuki-img/blob/master/Picture1.jpg?raw=true" width="200" ><img src="https://github.com/ustyui/ibuki-img/blob/master/Picture2.png?raw=true" width="330" >


## Simulation Model Initial Posture and Joint Mapping of Ibuki
### Simulation Model Initial Posture
Note that the ibuki is lack of some joints comparing with the simulation model. When generating input files, these effects should be considered.
**RA1 and RA0 should be considered as one rigid, there is only one joint between N0N1N2.**

<img src="https://github.com/ustyui/ibuki-img/blob/master/Picture3.png?raw=true" width="800" >

### Joint Mapping
The joints of ibuki and their array SNs has been shown in the below table. 
To map your simulation model joints with ibuki's joints, do as follows:

* Look at the Initial Posture graph, remember the name of the joint (e.g. **ARMR_MIDDLE**)
* Decide the orientation. (e.g. **PITCH**). 
* Combine the name and the orientation together with `_+<name>+_+<orientation>`. (e.g. **_ARMR_MIDDLE_PITCH**)
* Check the table below and find the array SN. 

**Note: you may not find your joint in the table below, such as _ARMR_MIDDLE_ROLL. This is because ibuki does not have this DoF. If this happens, please fix that DOF to constant.**

**Note: For those name do not end with ROLL, PITCH, YAW(mostly in head parts), they end with the actual meanings of the part. Please match it according to the actual situations.**

| name | array# |name| array #|name|array#|
----|----|----|----|----|----
| _SHOULDER_RIGHT_PITCH | 0 | _SHOULDER_LEFT_PITCH  | 1 | _NECK_UNIQUE_ROLL | 2 |
| _NECK_UNIQUE_YAW | 3 | _NECK_UNIQUE_PITCH	| 4 | _ARML_UPPER_ROLL | 5 |
| _ARML_UPPER_YAW	| 6 | _ARML_MIDDLE_PITCH | 7 | _ARML_MIDDLE_YAW	| 8 |
| _ARML_EDGE_ROLL | 9	| _ARMR_UPPER_ROLL | 10 |	_ARMR_UPPER_YAW	| 11 |
| _ARMR_MIDDLE_PITCH | 12 |	_ARMR_MIDDLE_YAW | 13 |	_ARMR_EDGE_ROLL | 14 |
| _HANDL_THUMB_PITCH | 15 |	_HANDL_INDEX_PITCH | 16 |	_HANDL_MIDDLE_PITCH | 17 |
| _HANDL_RING_PITCH | 18 | _HANDL_LITTLE_PITCH | 19 |	_HANDR_THUMB_PITCH | 20 |
| _HANDR_INDEX_PITCH | 21 |	_HANDR_MIDDLE_PITCH | 22 | _HANDR_RING_PITCH | 23 |
| _HANDR_LITTLE_PITCH | 24 |	_HEADL_OPHRYON_VERT | 25 |	_HEADL_EYELIDU_VERT | 26 |
| _HEADL_EYELIDD_VERT | 27 |	_HEADL_FACER_TENSE | 28 |	_HEADL_FACEL_TENSE | 29 |
| _HEADC_EYEBROWR_VERT | 30 |	_HEADC_EYEBROWL_VERT | 31 |	_HEADC_EYER_YAW | 32 |
| _HEADC_EYEL_YAW | 33 |	_HEADC_EYES_PITCH | 34 |	_HEADR_LIPUP_REACH | 35 |
| _HEADR_LIPDOWN_REACH | 36 |	_HEADR_MOUTHR_OPEN | 37 |	_HEADR_MOUTHL_OPEN | 38 |
| _HEADR_CHIN_ROLL | 39 |	_HIP_UNIQUE_PITCH | 40 |	_HIP_UNIQUE_ROLL | 41 |
| _HIP_UNIQUE_YAW | 42 |	_HIP_NULL_NULL  | 43 |  _HIP_NULL_NULL  | 44 |
|_WHEEL_UNIQUE_RIGHT | 45 |	_WHEEL_UNIQUE_LEFT | 46 | _WHEEL_UNIQUE_LINEAR | 47 |
|_WHEEL_UNIQUE_BREAK | 48 | _WHEEL_NULL_NULL | 49 |

## Input Data Format
### .csv file 
Download [https://github.com/ustyui/ibuki-v2/blob/master/doc/data_format.csv] for better understanding.

### ibuki jointname
1st row. **Don't edit**.
### Time (s)
1st column. **Current time(time passed from the very beginning)**, NOT the time interval from the last frame. e.g., 4.2 at line 9 means 4.2 seconds from the beginning of the motion.

Please keep the time intervals bigger(or equals to) than 0.02s.

### Unit(degree) of the value 
This .csv file use **rad** as input value. 

**Current rad apart from the initial posture** is needed.

<img src="https://github.com/ustyui/ibuki-img/blob/master/degree_def.png?raw=true" width="330" >

For example, if _ARMR_MIDDLE_PITCH rotates for +0.5 rad from the initial posture, then element for this line should be `line[12+1] = 0.5`. It is recommended to keep the numbers within 5 digits (e.g. 0.1234).
### Direction of the value
Please refer to the coordination frame above.

In the coordinate frame, 

the direction the same with the arrow (**unclockwise towards the postive axis**) is positive direction.
In this case, you should input your value as a positive value.

the direction the same with the arrow (**unclockwise towards the postive axis**) is negative direction.
In this case, you should input your value as a negative value.

