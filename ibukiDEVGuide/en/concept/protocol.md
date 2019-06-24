# EVANS message on ibuki
EVANS is the basic message exchanged between nodes of silva.
It is a defined message of ROS format.

## Format
Header header

string name

uint8 seq

uint8 msgid

int16[] payload


> under construction.

**header: ROS header.**

**name: a direct way to distinguish message type, the label of the message.**
Now the availible names are listed below:

* **hsmap** : The message used in Humanoid Simulation Mapping function.

**seq: priority of the message, varies from 0 to 99.**
The table of priority is shown under the table.

| seq | priorty         | always seen in |
|-----|-----------------|----------------|
| 0   | without priorty |                |
| 1   | 1st priorty     |                |
| 2   | level 2 priorty |                |
| 3   | level 3 priorty |                |
| ... | ...             |                |
| 99  | last priorty    |                |

**msgid: message id. Different message with different functions has different msgid.**

**payload: The motion command payload of the message. Now there are 3 types of payload format.**

## Message Index
### EVANS Message
| message name | name | seq | msgid | payload | related nodes | function |
|--------------|------|-----|-------|---------|---------------|----------|
| /silva/joint_local/default | default | 0 | 0 | 1*50 array | pmemory, I, R, S, A|announce default(initial) posture of the robot |
| /silva/joint_local/fusion | fusion | 0 |  0 | 1*50 array | pmemory, joint_interface | send the mixed output to joint drivers |
| /silva/joint_local/idle | idle |　1|　0 | 1*50 array　| pmemory, I | send the idle output to the mixed node |
| /silva/joint_local/reflex | reflex |　2|　0 | 1*50 array　| pmemory, R | send the idle output to the mixed node |
| /silva/joint_local/slave | slave |　3|　0 | 1*50 array　| pmemory, S | send the idle output to the mixed node |
| /silva/joint_local/auto | auto |　4|　0 | 1*50 array　| pmemory, A | send the idle output to the mixed node |
| /silva/idle_local/intention |(bodypart) | 0 | 1 | 1*5 array | <ibuki_extra>, I | | 
| /silva/idle_local/ch0 |(bodypart) | 0 | 1 | 1*5 array | <ibuki_extra>, I | | 
| /silva/reflex_local/ch0 |(bodypart)|0|0|1*5 array | R, internal sensor, silva GUI | get the potential meter feedback (angle) |
| /silva/reflex_local/ch1 |(bodypart)|0|0|1*5 array | R, internal sensor | get the current meter feedback (current) |
| /silva/reflex_local/ch2 |(bodypart)|0|0|1*5 array | R, sample program | used for experiment YG |
| /silva/slave_local/intention |(bodypart)|0|3|1*5 array | S,  |  |
| /silva/slave_local/operation ||0|0|1*50 array | S, silva GUI |  |
| /silva/slave_local/decision ||0|0|1*50 array | S,  |  |
| /silva/slave_local/walking |(bodypart)|0|3|1*5 array | S,  |  |
| /silva/slave_local/hsm |hsmap|0|3|1*50 array | S, pmemory |  |
| /silva/auto_local/ch0 | (bodypart) | 0 | 0 | 1*5 array |  auto, <ibuki_extra> | receive idle motion |
| /silva/auto_local/ch1 | (bodypart) | 0 | 0 | 1*5 array |  auto, <realsense> | used for experiment IS |
| /silva/auto_local/ch2 | (bodypart) | 0 | 0 | 1*5 array |  auto, <rplidar> | used for experiment IS |
| /silva/auto_local/ch3 | (bodypart) | 0 | 0 | 1*5 array |  auto, developing | used for experiment IS |


### Float32MultiArray
| message name | data | related nodes | function |
|--------------|------|-----|-------|
| /silva/joint_local/states | 1*4 array | pmemory, silva GUI | send the states obtained from control panel |

### String
| message name | data | related nodes | function |
|--------------|------|-----|-------|
| /silva/speech_global/jp | String | motion_carlos, silva GUI | send the speech to speech motion node |
