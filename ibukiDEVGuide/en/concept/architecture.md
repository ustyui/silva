# Silva Architectural Overview (Editing, not finished)

## High-Level Software Architecture{#architecture}
The diagram below provides a detailed overview of the building blocks of silva. The top part of the diagram contains framework blocks.

### Joint Stack
The following diagram shows an overview of the building blocks of the joint stack. It contains the full pipeline from sensors, down to the motor or servo control(Actuators).

## Update Rates
Typically the drivers define how fast a module updates. The fusion module integrate messages and publish with 40Hz.

The message update rates can be inspected in real-time on the system by running ```rosrun silva_beta top```.

