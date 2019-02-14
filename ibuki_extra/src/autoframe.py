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

import transformations as tform

### environment variables ###
_RATE = 20

### recsound class ###
class recsound():
    
# initialization
    def __init__(self):
        
        # var
        self._direction = 180
        
        self._neck = 0.0
        
        self._payload_headc = [0.0,0.0,0.0,0.0]
        self._payload_neck = [0,0,]
        self._payload_hip = []
        
        # messages
        self._pub_msg = Evans()
        
        # publisher
        self.pub = rospy.Publisher('/silva/auto_local/ch1', Evans, queue_size=10)
        
        # subscriber
        self.sub = rospy.Subscriber('/sound_direction', Int32, self.direct_cb)

    ### callback functions ###
    def direct_cb(self, msg):
        
        self._direction = msg.data
        
    

    ## main function ##
    def react(self):
        if self._direction < 180:
            self._neck = 20.0
            
    ### message make phase ###
    
    ### ros start function ###
    def start(self):
        loop_rate = rospy.Rate(_RATE)
        
        while not rospy.is_shutdown():
            self.react()
            self._payload_neck = [0,0,0,self._neck,0]
            tform.make_message(self._pub_msg, 4, 'neck', 2, self._payload_neck)
            self.pub.publish(self._pub_msg)
            #print(self._payload_neck)
            print self._direction
            
        loop_rate.sleep()
### main loop ###
if __name__ == "__main__":
    
    # poseblock class, in slave
    Spose = recsound()
    
    # init nodes
    nh = rospy.init_node("REC")
    
    # start node
    Spose.start()