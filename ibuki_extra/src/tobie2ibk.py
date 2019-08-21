#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 21 17:48:04 2019

@author: ustyui
"""

import rospy
from silva_beta.msg import Evans
from std_msgs.msg import Float32MultiArray
import transformations as utils

_RATE = 100

# class tobies
class Tobie():
    #initial ros messages 
    def __init__(self):
        self._posx = 0.0
        self._posy = 0.0
        
        self._payload = []
        
        utils.set_zeros(self._payload)
        
        # message
        self._pub_msg = Evans()
        
        # publishers
        self.pub = rospy.Publisher('/silva/auto_local/ch3', Evans, queue_size=10)
        
        self.sub = rospy.Subscriber('eye_tracking', Float32MultiArray, self.tobie_cb)
        
    # callback function
    def tobie_cb(self, msg):
        self._posx = msg.data[0]
        self._posy = msg.data[1]
        
    def start(self):
        rospy.loginfo("Tobie")
        loop_rate = rospy.Rate(_RATE)
        
        while not rospy.is_shutdown():
            
            #make payload
            # I Said Calculation
            self._payload[32] = int(self._posx*400) -200
            self._payload[33] = self._payload[32]
            # eye pitch, head_c 
            self._payload[34] = int(self._posy*300)-150
            
            utils.make_message(self._pub_msg, 4, 'tobie', 4, self._payload)
            
            self.pub.publish(self._pub_msg)
            
            loop_rate.sleep()
            
if __name__ == "__main__":
    
    # tobie
    tobie = Tobie()
    
    nh = rospy.init_node("Tobie_Adapter")
    
    tobie.start()
    
    