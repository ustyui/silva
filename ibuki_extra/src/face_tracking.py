#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on June 24 2019

@author: ise
"""
device_name = 'neck'
dir_front = 5000

"ibuki utils"

import transformations as tform
import numpy as np
"ros modules"
import rospy
from silva_beta.msg import Evans
from std_msgs.msg import String, Int32,Int32MultiArray

"UDP modules"
import socket
import math

joint_default = [0,0,0,0,0]

"INITIALIZATION:default must be defined!"
class Joystick(object):
    def __init__(self):
        
        # env var
        self._pub_msg = Evans()
        # payload
        self._payload = [0, 0, 0, 0, 0]
        
        self.key = None
        self.string = ''
        self.eye_width = 0
	self.eye_height = 0
	self.neck_pos = [0,0,0,0,0]#shoulder*2,r,y,p
        self.loop_rate = rospy.Rate(50)
        #Publisher
	self.pub = rospy.Publisher('/silva/auto_local/ch3', Evans, queue_size =25)
	rospy.Subscriber('/face_pos',Int32MultiArray, self.callback)
	rospy.Subscriber('/silva/reflex_local/feedback',Evans,self.position)
        
    def callback(self,msg):
	weight_w = 0.45
	weight_h = 0.7
	self.eye_width = weight_w *(-1)* msg.data[2]
	self.eye_height = weight_h * (-1) * msg.data[3]

    def position(self,msg):
	self.neck_pos[0] = msg.payload[0]
	self.neck_pos[1] = msg.payload[1]
	self.neck_pos[2] = msg.payload[2]
	self.neck_pos[3] = msg.payload[3]
	self.neck_pos[4] = msg.payload[4]

    def start(self):
        rospy.loginfo("In attesa")
        
        while not rospy.is_shutdown():

            # make message 
            self._payload = [self.neck_pos[0], self.neck_pos[1], self.neck_pos[2], self.neck_pos[3] + self.eye_width, self.neck_pos[4] + self.eye_height]
            tform.make_message(self._pub_msg, 0, 'neck', 0, self._payload)
            # publish
            self.pub.publish(self._pub_msg)
            
            #self.pub.publish(message)
            self.loop_rate.sleep()

    def fake(self,msg):
        print('hello')


if __name__ == "__main__":
    rospy.init_node('Artificial_potential'+device_name) #changed by ise

    joystickA = Joystick()
    joystickA.start()

