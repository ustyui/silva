#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon May 27 18:10:40 2019

@author: nvidia
"""

import rospy
import math
from silva_beta.msg import Evans
import transformations as tform

class Joints():
    #init
    def __init__(self):
        self._payload = []
        # zero 
        tform.set_zeros(self._payload)
        
        # message
        self.msg = Evans()
        
        # publisher
        self.pub = rospy.Publisher('/silva/reflex_local/ch2', Evans, queue_size=10)
        
        # subs
        self.sub = rospy.Subscriber('/silva/reflex_local/feedback', Evans, self.callback)
        
        
    def makemotion(self):
        # make motion
	A = 100
	p = 0.1
	phi = math.pi * 0	

        self._payload[0] = 100*math.sin(0.00277778*math.pi*self.encoder_feedback)
        self._payload[1] = - 100*math.sin(0.00277778*math.pi*self.encoder_feedback)
        self._payload[40] = p*A*math.sin(0.00277778*math.pi*self.encoder_feedback)
        self._payload[45] = - A*math.sin(2*0.00277778*math.pi*self.encoder_feedback-phi)

    def callback(self, msg):
        self.encoder_feedback = msg.payload[0]
        
        self.makemotion()
        
    def start(self):
        loop_rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            tform.make_message(self.msg, 1, 'gait', 1, self._payload)
            
            self.pub.publish(self.msg)
            print self._payload
            
            loop_rate.sleep()
        
if __name__ == "__main__":
    
    # poseblock class, in reflex
    Rpose = Joints()
    
    # init nodes
    nh = rospy.init_node("test")
    
    # start node
    Rpose.start()
        

        
    
    
        
