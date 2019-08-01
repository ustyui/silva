# Initial Setup & Configuration

We recommend that developers obtain the basic equipment described below (or similar) and use a default configuration.

## Basic equipment

The equipment below is highly recommended:

* A development computer:
  * XPS13 with Ubuntu Linux 16.04 or later
* A command center device:
  * Lenovo with Ubuntu Linux 16.04 or later
* A korg nanokontrol2 midi controller
* A joystick game controller
  * PS4 game controller with wireless USB adapter
  * xbox360 standard game controller

## Robot configuration

> **Tip** [*mbed complier*](https://os.mbed.com) is required for robot microcontroler configuration.

To configure ibuki:

1. Open mbed complier for obtaining complied ibuki's driver program.
1. [Basic Configuration](/en/config.md) explains how to perform basic configuration.
1. [Parameter Configuration](/en/parameters.md) explains how you can find and modify individual parameters.

## Build Hardware Environment on Jetson AGX Xavier
Step 1. Download NVIDIA sdk manager, and connect the Xavier device according to the instruction on the webpage.

Step 2. Set the Xavier to Force Recovery Mode and Install the Image using NVIDIA AGX Xavier according to the instructions.

Step 3. After the Installation is finished, launch Xavier and change the overall performance with `sudo nvpmodel -m 0`

Step 4. Build libreaslsense by using the instruction on BuildLibrealsenseXavier.

Step 5.
