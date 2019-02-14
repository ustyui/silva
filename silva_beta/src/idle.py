#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  6 22:30:32 2019
# Idle Motion
# library succeed from Auto 

@author: ustyui
"""
import rospy
from silva_beta.msg import Evans

import os, threading

import transformations as tform
### environment variables ###

_RATE = 30 # ros rate

# TODO: change this to a file load function
seq_of_jointname = {'neck':0,
                    'arml':1,
                    'armr':2,
                    'handl':3,
                    'handr':4,
                    'headl':5,
                    'headc':6,
                    'headr':7,
                    'hip':8,
                    'wheel':9}
                    
# initialize

class poseblock():
    def __init__(self):
        
        # number of driveunits
        
        # global variables
        self._rel = []
        self._bias = []
        self._payload = []
        self._default = []
        
        # messages
        self._default_rec = Evans()
        self._pub_msg = Evans()
        self._intention = Evans()
        
        # publishers
        self.pub = rospy.Publisher('/silva/joint_local/idle', Evans, queue_size=10)
        
        # subscribers
        self.sub_int = rospy.Subscriber('/silva/idle_local/intention', Evans, self.intention_cb)
        self.sub_default = rospy.Subscriber('/silva/joint_local/default', Evans, self.default_cb)
        
        tform.set_zeros(self._default)
        tform.set_zeros(self._rel)

    ### callback functions ###

    def intention_cb(self, msg):
        self._intention = msg

        # cut method : from where
        _cut = seq_of_jointname[msg.name]
        
        # get the payload
        _payload = msg.payload
        
        # if the msgid = 1 then to relative
        if msg.msgid == 1:
            
            # place payload to the cut place
            for _idx in range (0, len(_payload)):
                self._rel[_cut*5 + _idx] = _payload[_idx]
        
    def default_cb(self, msg):
        # callback default
        self._default_rec = msg
        self._default = msg.payload
        
    def set_msg_from_pos(self):
        # add intentions to default
        self._payload = self._rel
        
    def make_message(self):
        # make message
        self._pub_msg.header.stamp = rospy.Time.now()
        self._pub_msg.seq = 1
        self._pub_msg.name = 'idle'
        self._pub_msg.msgid = 0
        self._pub_msg.payload = self._payload
    
    def start(self):
        rospy.loginfo("silva_IDLE")
        
        loop_rate = rospy.Rate(_RATE)
        
        while not rospy.is_shutdown():
            
            # set message from callback
            self.set_msg_from_pos()
            
            # make the message
            self.make_message()
            
            # publish the message
            self.pub.publish(self._pub_msg)
            
            loop_rate.sleep()
        
        
if __name__ == "__main__":
    
    # poseblock class, in idle
    Ipose = poseblock()
    
    # init nodes
    nh = rospy.init_node("filter_IDLE")
    
    # start node
    Ipose.start()