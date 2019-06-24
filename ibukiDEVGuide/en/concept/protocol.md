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
| message name | name | seq | msgid | payload | related nodes | function |
|--------------|------|-----|-------|---------|---------------|----------|
| /silva/joint_local/default | default | 0 | 0 | 1*50 array | pmemory, I, R, S, A|announce default(initial) posture of the robot |
| /silva/joint_local/fusion | fusion | 0 |  0 | 1*50 array | pmemory, joint_interface | send the mixed output to joint drivers |
| /silva/reflex_local/ch0 |(bodypart)|0|0|1*5 array | R, internal sensor | get the potential meter feedback (angle) |
| /silva/reflex_local/ch0 |(bodypart)|0|0|1*50 array | R, internal sensor | get the current meter feedback (current) |
| /silva/idle_local/intention | | 0 | 0 | 
| /silva/auto_local/ch3 | (bodypart) | 0 | 0 | 1*5 array |  auto, developing | developing |
