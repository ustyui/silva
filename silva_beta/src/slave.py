#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  6 23:19:55 2019
# Slave Motion
# library succeed from Auto 
# TODO: create a /demo channel

@author: ustyui
"""

import rospy
from silva_beta.msg import Evans
from sensor_msgs.msg import Joy

import os, threading
import numpy as np

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
        
        # global variables
        self._rel = []
        self._bias = [[],[],[],[],[],[]]
        self._payload = []
        self._default = []
        self._gain = 1.0
        
        # messages
        self._default_rec = Evans()
        self._pub_msg = Evans()
        self._intention = Evans()
        
        # publishers
        self.pub = rospy.Publisher('/silva/joint_local/slave', Evans, queue_size=10)
        
        # subscribers
        self.sub_int = rospy.Subscriber('/silva/slave_local/intention', Evans, self.intention_cb)
        self.sub_opt = rospy.Subscriber('/silva/slave_local/operation', Evans, self.operation_cb) 
        self.sub_dec = rospy.Subscriber('/silva/slave_local/decision', Evans, self.decision_cb)
        self.sub_wlk = rospy.Subscriber('/silva/slave_local/walking', Evans, self.walking_cb)
        self.sub_hsm = rospy.Subscriber('/silva/slave_local/hsm', Evans, self.hsm_cb)
        self.sub_dem = rospy.Subscriber('/silva/slave_local/demo', Evans, self.demo_cb)
        
        self.sub_default = rospy.Subscriber('/silva/joint_local/default', Evans, self.default_cb)
        self.sub_gain = rospy.Subscriber('/joy', Joy, self.joy_cb)
        
        tform.set_zeros(self._default)
        tform.set_zeros(self._rel)
        for i in range(len(self._bias)):
            tform.set_zeros(self._bias[i])

    ### callback functions ###
    def joy_cb(self, msg):
        if msg.header.frame_id == "main":
            self._gain = 3*msg.axes[14]+4.0
        
    def operation_cb(self, msg):
        # TODO: if there is any HSM message, stop debug gui

        
        self._bias[0] = list(msg.payload)

    def intention_cb(self, msg):
        self._intention = msg

        # cut method : from where
        _cut = seq_of_jointname[msg.name]
        
        # get the payload
        _payload = msg.payload
        
        # if the msgid = 3 then to relative
        if msg.msgid == 3:
            
            # place payload to the cut place
            for _idx in range (0, len(_payload)):
                self._bias[0][_cut*5 + _idx] = _payload[_idx]
                
    def decision_cb(self, msg):
        
        self._bias[2] = list(np.array(msg.payload))
        
    def walking_cb(self, msg):
        _cut = seq_of_jointname[msg.name]
        _payload = msg.payload
        # place payload to the cut place
        for _idx in range (0, len(_payload)):
            self._bias[3][_cut*5 + _idx] = _payload[_idx]
            
    def hsm_cb(self, msg):
        
        self._bias[4] = list(np.array(msg.payload))
        
    def demo_cb(self, msg):
        
        self._bias[5] = list(np.array(msg.payload))
        
    def default_cb(self, msg):
        # callback default
        self._default_rec = msg
        self._default = msg.payload
    
    ### merge bias ###    
    def set_msg_from_pos(self):
        
        mult_ch = np.array(self._bias)
        self._rel = mult_ch[0] + mult_ch[1] + mult_ch[2]+ mult_ch[3] +mult_ch[4]+mult_ch[5]
        
        self._payload = self._rel
        
    
    def start(self):
        rospy.loginfo("silva_SLAVE")
        
        loop_rate = rospy.Rate(_RATE)
        
        while not rospy.is_shutdown():
            
            # set message from callback
            self.set_msg_from_pos()
            
            # make the message
            tform.make_message(self._pub_msg, 3, 'slave', 0, self._payload)
            
            # publish the message
            self.pub.publish(self._pub_msg)
            
            
            # debug
            
            # print type(self._rel)
            loop_rate.sleep()
        
        
if __name__ == "__main__":
    
    # poseblock class, in slave
    Spose = poseblock()
    
    # init nodes
    nh = rospy.init_node("filter_SLAVE")
    
    # start node
    Spose.start()