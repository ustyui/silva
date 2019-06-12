#!/usr/bin/env python
# -*- coding: utf-8 -*-
device_name = 'hip'
dir_front = 5000

"ibuki utils"

import transformations as tform

"ros modules"
import rospy
from silva_beta.msg import Evans
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Imu

"UDP modules"
import socket
import math

joint_default = [0,0,0,0,0]
high_limit = 200
low_limit = -200

"INITIALIZATION:default must be defined!"
class Joystick(object):
    def __init__(self):
        
        # env var
        self._pub_msg = Evans()
        # payload
        self._payload = [0, 0, 0, 0, 0]
	self.last_payload = [0, 0, 0, 0, 0]
        self.last_ax = 0.0
        self.key = None
        self.string = ''
        self.jointnow1, self.jointnow2, self.jointnow3 = 0,0,0   
        self.loop_rate = rospy.Rate(20)
	self.Kp = 4
	self.Kd = 3.5
	self.Ki = 0.3
	self.integral = 0

        #Publisher
        self.pub = rospy.Publisher('/silva/auto_local/ch3', Evans, queue_size =25)

        self.deviation = [0.0, 0.0, 0.0, 0.0, 0.0]

        #Subscriber
        rospy.Subscriber('/imu/data_raw1',Imu, self.imu_callback)
        
    def imu_callback(self, msg):
#        print("get")
	self.integral += (msg.linear_acceleration.x - self.last_ax)
	self.last_payload = self._payload
        if (msg.linear_acceleration.x > 0.1) or (msg.linear_acceleration.x < -0.1):
		self._payload[1] = int(self.Kp * msg.linear_acceleration.x + self.Kd * (msg.linear_acceleration.x - self.last_ax) + self.Ki * self.integral)
		self.last_ax = msg.linear_acceleration.x
	else:
		self._payload[1] = self.last_payload[1]
	if self._payload[1] > high_limit:
		self._payload[1] = high_limit
	if self._payload[1] < low_limit:
		self._payload[1] = low_limit
	print(self._payload)

    def start(self):
        rospy.loginfo("In attesa")
        
        while not rospy.is_shutdown():

            # make message 
#            self._payload = [200, 200, self.speed_r, self.speed_l, 600]
            tform.make_message(self._pub_msg, 4, 'hip', 6, self._payload)
            # publish
            self.pub.publish(self._pub_msg)
            
            #self.pub.publish(message)
            self.loop_rate.sleep()

    def fake(self,msg):
        print('hello')


if __name__ == "__main__":
    "motor command socket"
    motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#    wheelsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#    rtclient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #motorsock.bind((ip_master, port_drive_headl))
    "Send To Mbed"
    rospy.init_node('imu_control'+device_name) #changed by ise

    joystickA = Joystick()
    joystickA.start()

