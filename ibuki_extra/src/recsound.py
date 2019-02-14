#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 28 21:27:27 2019
respeaker module
@author: nvidia
"""
# response band : leftside: -90 -180 180 90
# message /sound_direction (/sound_loc)
# moveaxis: eye, neck, hip
# choose to react: prbability 0.05-0.10?

# robot need to move quickly

### modules ###

import rospy

from silva_beta.msg import Evans
from std_msgs.msg import Int32
from numpy import random as rd

import transformations as tform

### environment variables ###
_RATE = 20

### recsound class ###
class recsound():
    
# initialization
    def __init__(self):
        
        # var
        self._direction = 180
        
        self._neck = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._headc = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._hip = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self._payload_headc = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._payload_neck = [0,0, 0.0, 0.0, 0.0, 0.0]
        self._payload_hip = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.human = 0
        
        # messages
        self._pub_neck = Evans()
        self._pub_hip = Evans()
        self._pub_headc = Evans()
        
        # publisher
        self.pub = rospy.Publisher('/silva/auto_local/ch1', Evans, queue_size=10)
        
        # subscriber
        self.sub = rospy.Subscriber('/sound_direction', Int32, self.direct_cb)
        self.human_sub = rospy.Subscriber('/isHuman', Int32, self.updown)
        
    ### callback functions ###
    def direct_cb(self, msg):
        
        if (msg.data>-180 and msg.data<-90):
            # ltside
            self._direction = msg.data + 90
        if (msg.data>90 and msg.data<180):
            # rtside
            self._direction = msg.data - 90 # hip for 1.0
            
        # because I only want it to trig occasionly, so I put main func here
        if rd.rand()<1.0:
            # hip move
            self._hip[2] = 0.8*self._direction
            # neck move
            self._neck[3] = -1.0*self._direction
            if self.human == 1:
                self._neck[4] = -80
            else:
                self._neck[4] = 0
            # eye move
            self._headc[2] = -0.8* self._direction
            self._headc[3] = -0.8* self._direction
            
    def updown(self, msg):
        if msg.data == 1:
            self.human = 1
        else:
            self.human = 0       
            
    ### message make phase ###
    
    ### ros start function ###
    def start(self):
        loop_rate = rospy.Rate(_RATE)
        
        while not rospy.is_shutdown():
            
            self._payload_neck = self._neck
            self._payload_headc = self._headc
            self._payload_hip = self._hip
            
            tform.make_message(self._pub_neck, 4, 'neck', 2, self._payload_neck)
            self.pub.publish(self._pub_neck)
            
            tform.make_message(self._pub_headc, 4, 'headc', 2, self._payload_headc)
            self.pub.publish(self._pub_headc)
            
            tform.make_message(self._pub_hip, 4, 'hip', 2, self._payload_hip)
            self.pub.publish(self._pub_hip)
            
            #print(self._payload_neck)
            print self._payload_hip
            
        loop_rate.sleep()
### main loop ###
if __name__ == "__main__":
    
    # poseblock class, in slave
    Spose = recsound()
    
    # init nodes
    nh = rospy.init_node("REC")
    
    # start node
    Spose.start()
