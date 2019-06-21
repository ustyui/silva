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

seq: priority of the message, varies from 0 to 99
msgid: message id. Different message with different functions has different msgid
payload: The motion command payload of the message. Now there are 3 types of payload format.
