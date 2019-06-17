# EVANS message on ibuki
The most basic message exchanged between nodes of silva.

## Format
Header header

string name

uint8 seq

uint8 msgid

int16[] payload


> under construction.

name: a direct way to distinguish message type
seq: priority of the message, varies from 0 to 99
msgid: message id. Different message with different functions has different msgid
payload: The motion command payload of the message. Now there are 3 types of payload format.
